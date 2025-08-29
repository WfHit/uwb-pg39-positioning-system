/**
 * @file    ds_twr.c
 * @brief   Double-Sided Two-Way Ranging (DS-TWR) implementation
 * 
 * This file implements the DS-TWR protocol for high-precision distance measurement
 * between UWB devices. DS-TWR provides improved accuracy over single-sided ranging
 * by using multiple message exchanges to cancel out clock drift effects.
 * 
 * DS-TWR Protocol Sequence:
 * 1. POLL message (Tag -> Anchor)
 * 2. RESPONSE message (Anchor -> Tag) 
 * 3. FINAL message (Tag -> Anchor)
 * 
 * @author  UWB PG3.9 project
 * @date    2024
 */

#include "ds_twr.h"
#include "filter.h"
#include "../../drivers/decawave/deca_device_api.h"
#include "../../drivers/decawave/deca_regs.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/*============================================================================
 * GLOBAL VARIABLES
 *============================================================================*/

/** @brief System calculation state machine flag for DS-TWR process */
uint8_t SYS_Calculate_ACTIVE_FLAG = 0;

/** @brief DWM1000 communication transmit data buffer */
uint8_t DS_send_msg[DS_TX_BUF_LEN];

/** @brief DWM1000 communication receive data buffer */  
uint8_t DS_rx_buffer[DS_RX_BUF_LEN];

/*============================================================================
 * PRIVATE CONSTANTS
 *============================================================================*/

/** @brief DS-TWR protocol identifiers */
#define DS_TWR_POLL_MSG_ID      0xAB    ///< POLL message identifier
#define DS_TWR_RESPONSE_MSG_ID  0xBC    ///< RESPONSE message identifier  
#define DS_TWR_FINAL_MSG_ID     0xCD    ///< FINAL message identifier

/** @brief Timeout and state values */
#define DS_TWR_RX_TIMEOUT_UUS   9500    ///< Receive timeout in UWB microseconds
#define DS_TWR_STATE_IDLE       0       ///< State machine idle state
#define DS_TWR_STATE_TX_WAIT    1       ///< Waiting for transmission complete
#define DS_TWR_STATE_RX_WAIT    2       ///< Waiting for reception complete

/*============================================================================
 * DS-TWR COMMUNICATION FUNCTIONS
 *============================================================================*/



/**
 * @brief ��վ���б�ǩ�����в��
 * @param A_ID ���ͷ�ID 
 * @param B_ID ���շ�ID
 * @param MODE ����ģʽ 
 * @param ret  ״ָ̬ʾ 0�ޱ仯 1������ -1���ʧ�� -2�����;��ʱ 
 */
int32_t DW1000send(uint8_t A_ID,uint8_t B_ID,uint8_t MODE, int8_t *ret) //����ģʽ
{
	uint16_t result_dist;   
	uint32_t i;
	if(SYS_Calculate_ACTIVE_FLAG==0)
	{
		memset(DS_send_msg,0,sizeof(DS_send_msg));

		DS_send_msg[0] =  A_ID;	//UWB POLL ������
		DS_send_msg[1] =  B_ID;//UWB Fianl ������
		DS_send_msg[2] = frame_seq_nb;
		DS_send_msg[3] = 0XAB; 
		DS_send_msg[4] = MODE;    //��ʼ��			  

		//�ϴζ�λ�Ƿ�ɹ�
		DS_send_msg[5] = Cal_data[B_ID].Cal_Flag >> 16 & 0x01;
		DS_send_msg[6]=Cal_data[B_ID].x>>8;		//������һ�ε�������Ϣ
		DS_send_msg[7]=Cal_data[B_ID].x&0x00FF;		//������һ�ε�������Ϣ				
		DS_send_msg[8]=Cal_data[B_ID].y>>8;		//������һ�ε�������Ϣ
		DS_send_msg[9]=Cal_data[B_ID].y&0x00FF;		//������һ�ε�������Ϣ
		DS_send_msg[10]=Cal_data[B_ID].z>>8;		//������һ�ε�������Ϣ
		DS_send_msg[11]=Cal_data[B_ID].z&0x00FF;		//������һ�ε�������Ϣ	
		
		//�ϴβ���Ƿ�ɹ�
		DS_send_msg[12] = Cal_data[B_ID].Cal_Flag >> 8 & 0x00FF;
		DS_send_msg[13] = Cal_data[B_ID].Cal_Flag & 0x00FF;		
		for(i=0;i<ANCHOR_LIST_COUNT;i++)
		{
			DS_send_msg[14 + i * 2]=Cal_data[B_ID].Dist[i]>>8;		//������һ�εľ�����Ϣ
			DS_send_msg[15 + i * 2]=Cal_data[B_ID].Dist[i]&0x00FF;		//������һ�εľ�����Ϣ
		}								
							
		dwt_writetxdata(DS_POLL_LEN, DS_send_msg, 0);//��Poll�����ݴ���DW1000�����ڿ�������ʱ����ȥ
		dwt_writetxfctrl(DS_POLL_LEN, 0, 1);//���ó������������ݳ���
		dwt_setrxaftertxdelay(0);
		dwt_setrxtimeout(9500);						//���ý��ճ�ʱʱ��

		dwt_starttx(DWT_START_TX_IMMEDIATE| DWT_RESPONSE_EXPECTED);//�������ͣ�������ɺ�ȴ�һ��ʱ�俪�����գ��ȴ�ʱ����dwt_setrxaftertxdelay������;	
		SYS_Calculate_ACTIVE_FLAG=1;
	}			
	if(SYS_Calculate_ACTIVE_FLAG==1)
	{			
//		printf("%d\n",SYS_Calculate_ACTIVE_FLAG); 
		if((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))//���ϲ�ѯоƬ״ֱ̬���ɹ����ջ��߷�������
		{
			SYS_Calculate_ACTIVE_FLAG=2;
		}
		else return 0;
	}
	if(SYS_Calculate_ACTIVE_FLAG==2)
	{
//				printf("%d\n",SYS_Calculate_ACTIVE_FLAG); 
		if(frame_seq_nb<0xFF)
			frame_seq_nb++;
		else 
			frame_seq_nb=0;
		if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)//����ɹ�����
		{									
			SYS_Calculate_ACTIVE_FLAG=3;
		}
		else 
		{
			/* Clear RX error events in the DW1000 status register. */
			dwt_write32bitreg(SYS_STATUS_ID,SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);	
	//          	dwt_rxreset();
			if(status_reg & SYS_STATUS_ALL_RX_TO)
				*ret = -2;  //�������β�������;ʧ��						
			SYS_Calculate_ACTIVE_FLAG=0;
			return	0;	
		}
	}
						
	if(SYS_Calculate_ACTIVE_FLAG==3)
	{
//		printf("%d\n",SYS_Calculate_ACTIVE_FLAG); 
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_TXFRS_BIT_MASK);//����Ĵ�����־λ
		frame_len = dwt_read32bitreg(RX_FINFO_ID) & FRAME_LEN_MAX;	//��ý��յ������ݳ���
		dwt_readrxdata(DS_rx_buffer, frame_len, 0);   //��ȡ��������
		
		if ((DS_rx_buffer[3]==0xBC)&&((DS_rx_buffer[0]==B_ID)&&(DS_rx_buffer[1]==A_ID)))//�жϽ��յ��������Ƿ���response����
		{  
			SYS_Calculate_ACTIVE_FLAG=4;
		}
		else 
		{	
			SYS_Calculate_ACTIVE_FLAG=0;
			return 0;							
		}		
	}
	if(SYS_Calculate_ACTIVE_FLAG==4)
	{ 
		uint8_t send_len = DS_FIX_BUF_LEN;
		memcpy(DS_send_msg,DS_rx_buffer,DS_TX_BUF_LEN);
		
		Time_ts[0] = get_tx_timestamp_u64();										//���POLL����ʱ��T1
		Time_ts[3] = get_rx_timestamp_u64();										//���RESPONSE����ʱ��T4
		final_msg_set_ts(&DS_send_msg[4],Time_ts[0]);     //��T1д�뷢������
		final_msg_set_ts(&DS_send_msg[16],Time_ts[3]);    //��T4д�뷢������
		DS_send_msg[0] =  A_ID;	//������ID
		DS_send_msg[1] =  B_ID; //������ID
		DS_send_msg[3] =  0XCD; 
		DS_send_msg[2] = frame_seq_nb;
		
		DS_send_msg[28] = Uwb_commu_helper_ptr->Sender.Data_commu_En;
		if(Uwb_commu_helper_ptr->Sender.Data_commu_En && B_ID == Uwb_commu_helper_ptr->Sender.Data_commu_RevID)
		{							
			DS_send_msg[29] = Uwb_commu_helper_ptr->Sender.Data_commu_len;
			memcpy(&DS_send_msg[30],Uwb_commu_helper_ptr->Sender.DataBuff,Uwb_commu_helper_ptr->Sender.Data_commu_len);			
			Uwb_commu_helper_ptr->Sender.Data_commu_En = 0;
			send_len += Uwb_commu_helper_ptr->Sender.Data_commu_len;
		}
		else
		{
			DS_send_msg[29] = 0;
		}
		
		dwt_writetxdata(send_len, DS_send_msg, 0);//����������д��DW1000
		dwt_writetxfctrl(send_len, 0, 1);//�趨�������ݳ���
		dwt_setrxaftertxdelay(0);
		dwt_setrxtimeout(9500);						//���ý��ճ�ʱʱ��
		dwt_starttx(DWT_START_TX_IMMEDIATE| DWT_RESPONSE_EXPECTED);//�趨Ϊ���ͺ����̴򿪽���
		SYS_Calculate_ACTIVE_FLAG=5;						
	}
	
	if(SYS_Calculate_ACTIVE_FLAG==5)						
	{
		if ((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))//���ϲ�ѯоƬ״ֱ̬���ɹ����ջ��߷�������
		{ 
			SYS_Calculate_ACTIVE_FLAG=6;
		}
		else return 0;
	}
	
	if(SYS_Calculate_ACTIVE_FLAG==6)
	{	
		if(frame_seq_nb<0xFF)
			frame_seq_nb++;
		else 
			frame_seq_nb=0;
		
		if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)//����ɹ�����
		{	
			SYS_Calculate_ACTIVE_FLAG=7;	
		}
		else 
		{
			dwt_write32bitreg(SYS_STATUS_ID,SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);	
//			dwt_rxreset();
			if(status_reg & SYS_STATUS_ALL_RX_TO)
			 *ret = -2;  //�������β�������;ʧ��							
			SYS_Calculate_ACTIVE_FLAG=0;								
			return	0;								
		}
	}
	if(SYS_Calculate_ACTIVE_FLAG==7)
	{
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_TXFRS_BIT_MASK);//����Ĵ�����־λ
		frame_len = dwt_read32bitreg(RX_FINFO_ID) & FRAME_LEN_MAX;	//��ý��յ������ݳ���
		dwt_readrxdata(DS_rx_buffer, frame_len, 0);   //��ȡ��������						
		if ((DS_rx_buffer[3]==0xDE)&&((DS_rx_buffer[0]==B_ID)&&(DS_rx_buffer[1]==A_ID)))//�жϽ��յ��������Ƿ���response����
		{
			dwt_readdiagnostics(&rx_diag);                            //��ȡ���ν����ź�ǿ����Ϣ
			SYS_Calculate_ACTIVE_FLAG=8;
		}
		else 
		{
			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);	
			SYS_Calculate_ACTIVE_FLAG=0;
			return 0;
		}
	}
	if(SYS_Calculate_ACTIVE_FLAG==8)
	{
		uint32_t Time_ts_F[6];				
		
		//�յ�����͸��������
		if(DS_rx_buffer[28] == 1)
		{
			Uwb_commu_helper_ptr->Recver.Data_Has_recv = 1;
			Uwb_commu_helper_ptr->Recver.Data_commu_len = DS_rx_buffer[29];
			memcpy(Uwb_commu_helper_ptr->Recver.DataBuff,&DS_rx_buffer[30],Uwb_commu_helper_ptr->Recver.Data_commu_len);
		}
		
		final_msg_get_ts(&DS_rx_buffer[4], &Time_ts_F[0]);
		final_msg_get_ts(&DS_rx_buffer[8], &Time_ts_F[1]);
		final_msg_get_ts(&DS_rx_buffer[12], &Time_ts_F[2]);
		final_msg_get_ts(&DS_rx_buffer[16], &Time_ts_F[3]);
		final_msg_get_ts(&DS_rx_buffer[24], &Time_ts_F[5]);									
		Time_ts[0]= (uint32)Time_ts_F[0];
		Time_ts[1]= (uint32)Time_ts_F[1];
		Time_ts[2]= (uint32)Time_ts_F[2];
		Time_ts[3]= (uint32)Time_ts_F[3];
		Time_ts[4]= (uint32)get_tx_timestamp_u64();	
		Time_ts[5]= (uint32)Time_ts_F[5];

		if(Twr_CalDist(B_ID,&result_dist) == 0)
		{
			//����ʧ��
			SYS_Calculate_ACTIVE_FLAG=0;
			*ret = -1;
			return result_dist;
		}
		else
		{
			SYS_Calculate_ACTIVE_FLAG=0;
			*ret = 1;
			return result_dist;
		}										    								
	}				
	return 0;   
}

