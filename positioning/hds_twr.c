#include "hds_twr.h"
#include "filter.h"
#include <stdint.h>
#include <stdbool.h>

uint8_t SYS_ANC_RESP_FLAG = 0;                                //��վ��ʱ����Resp����־λ
uint8_t SYS_ANC_FINAL_FLAG = 0;                               //��վ����Final����־λ

uint16_t Dis_cal = 0;                                       //�����õľ���
uint8_t frame_now;                                            //��ǰͨѶ����֡
uint16_t Anc_recvFinal_timeflag = 0;

#define Anchor_delay_base  0x1E700                       //��վ��ʱ���ͻ��� 0x3CE00 ��ʱΪ1ms 0x1E700 ��ʱΪ500us  



u8 TX_ANC_INFORM_BUFF[TX_ANC_INFORM_LEN];                //��վ����Inform������
u8 TX_TAG_POLL_BUFF[TX_TAG_POLL_LEN];                    //��ǩ����Poll������
u8 TX_ANC_RESP_BUFF[TX_ANC_RESP_LEN];                    //��վ����Resp������
u8 TX_TAG_FINAL_BUFF[TX_TAG_FINAL_LEN];                  //��ǩ����Final������
u8 TX_ANC_REQ_BUFF[TX_ANC_REQ_LEN];                      //��վ����Request������  ����վ
u8 TX_ANC_REPLY_BUFF[TX_ANC_REPLY_LEN];                  //��վ����Reply������  �λ�վ
u8 HDS_rx_buffer[RX_MAX_LEN];                                  //���ջ���


/*! ------------------------------------------------------------------------------------------------------------------
 * @fn HDS_TWR_Send_Resp(u8 A_ID, u8 B_ID, u8 En)
 *
 * @brief  ��վ��ʱ����Resp��
 *
 * output parameters
   @param A_ID  ���ͷ���վID
   @param B_ID  ���ձ�ǩ��ID
   @param En    �Ƿ�ʹ������͸��
	
 * returns 0������δ��� 1�����ͳɹ� 2������ʧ��
 */
uint8_t HDS_TWR_Send_Resp(uint8_t A_ID, uint8_t B_ID, uint8_t En)
{
	uint32_t delay_time;
	if(SYS_ANC_RESP_FLAG == 0)
	{
		int ret;
		uint8_t anc_order = 0;		
		uint8_t send_len = TX_ANC_RESP_FIX_LEN;
		
		memset(TX_ANC_RESP_BUFF,0,sizeof(TX_ANC_RESP_BUFF));
		TX_ANC_RESP_BUFF[0] = A_ID;
		TX_ANC_RESP_BUFF[1] = B_ID;
		TX_ANC_RESP_BUFF[2] = frame_seq_nb;
		TX_ANC_RESP_BUFF[3] = 0xBC;
		TX_ANC_RESP_BUFF[4] = En;                          
		
		if(A_ID == 0xFF)
			anc_order = 1;
		else		
			anc_order = 17 - (0xFF - A_ID);                  //����ID����ȡ��ʱ����˳��
		
		if(En == 1)
		{
			TX_ANC_RESP_BUFF[5] = Uwb_commu_helper_ptr->Sender.Data_commu_len;
			memcpy(&TX_ANC_RESP_BUFF[6],Uwb_commu_helper_ptr->Sender.DataBuff,Uwb_commu_helper_ptr->Sender.Data_commu_len);
			send_len += Uwb_commu_helper_ptr->Sender.Data_commu_len;
		}
		
		delay_time = dwt_readsystimestamphi32();           //��ȡ�ϴν���ʱ��
		delay_time += Anchor_delay_base * anc_order;		       //��ʱ����
		dwt_setdelayedtrxtime(delay_time);
		dwt_writetxdata(send_len, TX_ANC_RESP_BUFF, 0);     //�����ݴ���DW1000�����ڿ�������ʱ����ȥ
		dwt_writetxfctrl(send_len, 0, 1);                      //���ó������������ݳ���
		dwt_setrxaftertxdelay(0);				
		ret = dwt_starttx(DWT_START_TX_DELAYED);                            //������ʱ����
		if(ret == DWT_SUCCESS)
			SYS_ANC_RESP_FLAG=1;
		else  //����ʧ��
			return 2;
	}
	
	if(SYS_ANC_RESP_FLAG == 1)
	{
		if((dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK))
		{
			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);               //�����־
			SYS_ANC_RESP_FLAG = 0;
			return 1;
		}	
	}
	
	return 0;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn HDS_TWR_Recv_FinalAndCal(uint8_t A_ID ,uint8_t B_ID)
 *
 * @brief  ���ձ�ǩFinal�����������
 *
 * output parameters
   @param A_ID  ���ͷ���վID
   @param B_ID  ���ձ�ǩ��ID
	 @param recv_timeout ��վ���ճ�ʱʱ��0.1msΪ��λ 
 * returns 0������δ��� 1������ɹ� 2���������ʧ��
 */
uint8_t HDS_TWR_Recv_FinalAndCal(uint8_t A_ID ,uint8_t B_ID, uint16_t recv_timeout)
{
	
	if(SYS_ANC_FINAL_FLAG == 0)
	{
		Anc_recvFinal_timeflag = 0;
		dwt_forcetrxoff();		
		dwt_setrxtimeout(0);    //�趨���ճ�ʱʱ�䣬0λû�г�ʱʱ��
		dwt_rxenable(0);
		SYS_ANC_FINAL_FLAG = 1;
	}
	
	if(Anc_recvFinal_timeflag > recv_timeout)
	{
		Anc_recvFinal_timeflag = 0;
		SYS_ANC_FINAL_FLAG = 0;
//		dwt_rxreset();
		dwt_forcetrxoff();
		return 2;
	}
	
	if(SYS_ANC_FINAL_FLAG == 1)
	{
		if((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR))//���ϲ�ѯоƬ״ֱ̬�����ճɹ����߳��ִ���
		{
			SYS_ANC_FINAL_FLAG = 2;
		}
		else return 0;
	}
	
	if(SYS_ANC_FINAL_FLAG == 2)  //�ɹ����ջ���ճ�ʱ
	{
		if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)//�ɹ�����
		{
			SYS_ANC_FINAL_FLAG = 3;
		}
		else
		{
			/* Clear RX error events in the DW1000 status register. */
			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
			SYS_ANC_FINAL_FLAG=0;							
//				if(status_reg & SYS_STATUS_ALL_RX_TO)
//				  return 2;
//			  else
//          return 0;	
		}
	}
		
	if(SYS_ANC_FINAL_FLAG == 3)
	{
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);            //�����־λ
		frame_len = dwt_read32bitreg(RX_FINFO_ID) & FRAME_LEN_MAX;    			//��ý������ݳ���
		dwt_readrxdata(HDS_rx_buffer, frame_len, 0);                                  //��ȡ��������
		if(HDS_rx_buffer[0] == B_ID && HDS_rx_buffer[3] == 0xCD)
		{
			SYS_ANC_FINAL_FLAG = 4;
		}		
		else if(HDS_rx_buffer[3] == 0xDE && HDS_rx_buffer[0] == 0xFF)
		{
			SYS_ANC_FINAL_FLAG = 0;
			Anc_recvFinal_timeflag = 0;
			return 2;
		}
		else
		{
			SYS_ANC_FINAL_FLAG = 0;		
		}			  
	}
		
	if(SYS_ANC_FINAL_FLAG == 4)
	{
		uint8_t Anc_order = 0;
		uint8_t i;
		//����������ն�Ӧ��ʱ���
		if(A_ID != 0xFF)
			Anc_order = ANCHOR_LIST_COUNT - (0xFF - A_ID);
		
		Time_ts[2] = (uint32)get_tx_timestamp_u64();              //��ȡT2 ��վ����Respʱ���
		final_msg_get_ts(&HDS_rx_buffer[4 + Anc_order*4],&Time_ts[3]);  //��ȡT3 ��ǩ����Respʱ���
		final_msg_get_ts(&HDS_rx_buffer[68],&Time_ts[0]);               //��ȡT0 ��ǩ����Pollʱ���
		final_msg_get_ts(&HDS_rx_buffer[72],&Time_ts[4]);               //��ȡT4 ��ǩ����Finalʱ��� ��Ϊ����ʱ��
		Time_ts[5] = (uint32)get_rx_timestamp_u64();              //��ȡT5 ��վ����Finalʱ���
		dwt_readdiagnostics(&rx_diag);                        //��ȡ���ν����ź�ǿ����Ϣ
		SYS_ANC_FINAL_FLAG = 5;
	}
		
	if(SYS_ANC_FINAL_FLAG == 5)                                 //�������
	{
		if(Twr_CalDist(B_ID,&Dis_cal) == -1)
		{
			//����ʧ��
			SYS_ANC_FINAL_FLAG = 0;
			return 2;				
		}
		else
		{
			SYS_ANC_FINAL_FLAG = 0;
			return 1;
		}
								
	}
	return 0;			
}


