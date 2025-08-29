#include "twr.h"
#include "filter.h"

/* Global variables for positioning calculations */
uint32_t Calculate_FLAG = 0;  ///< Bitfield indicating which anchors have valid calculations

/* Hold copy of status register state here for reference, so reader can examine it at a breakpoint. */
uint32_t status_reg = 0;
uint32_t frame_len = 0;        		//DWM1000�շ����ݰ����Ȼ���	

uint32_t Time_ts[6];  					             //����ʱ�仺���¼
uint32_t Dist_Cal_All[ANCHOR_LIST_COUNT] = {0};        //��ű��β�������

Cal_data_t Cal_data[TAG_USE_MAX_NUM];

uint32_t frame_seq_nb = 0;                   //ͨѶ����֡��
dwt_rxdiag_t rx_diag;                      //�����ź������Ϣ

char Tag_Usart_Str[TAG_USART_BUF_MAXLEN] = {'0'};
char Tag_ouput_dist_str[TAG_USART_BUF_DIST_LEN] = {'0'};
char Tag_ouput_rtls_str[TAG_USART_BUF_RTLS_LEN] = {'0'};

/******************************************************************************
												    �����ݰ���ȡʱ���
*******************************************************************************/
void final_msg_get_ts(const uint8_t *ts_field, uint32_t *ts)
{
    int i;
    *ts = 0;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        *ts += ts_field[i] << (i * 8);
    }
}
/******************************************************************************
												    �����ݰ���ȡ����
*******************************************************************************/
void final_msg_get_dist(const uint8_t *ts_field, uint32_t *dist)
{
    int i;
    *dist = 0;
    for (i = 0; i < 4; i++)
    {
        *dist += ts_field[i] << (i * 8);
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_tx_timestamp_u64()
 *
 * @brief Get the TX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
 uint64 get_tx_timestamp_u64(void)
{
    uint8_t ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_timestamp_u64()
 *
 * @brief Get the RX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
 uint64 get_rx_timestamp_u64(void)
{
    uint8_t ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_set_ts()
 *
 * @brief Fill a given timestamp field in the final message with the given value. In the timestamp fields of the final
 *        message, the least significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to fill
 *         ts  timestamp value
 *
 * @return none
 */
 void final_msg_set_ts(uint8_t *ts_field, uint32_t ts)
{
    int i;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        ts_field[i] = (uint8_t) ts;
        ts >>= 8;
    }
}
 void final_msg_set_dist(uint8_t *ts_field, uint32_t dist)
{
    int i;
    for (i = 0; i < 4; i++)
    {
        ts_field[i] = (uint8_t) dist;
        dist >>= 8;
    }
}

void Prepare_ascii_dist_output(Cal_data_t* now_data)
{
	char dist_str[25];
	uint8_t i;
	memset(Tag_ouput_dist_str,0,sizeof(Tag_ouput_dist_str));
	strcpy(Tag_ouput_dist_str,"Dist: ");
	for(i = 0;i < ANCHOR_LIST_COUNT;i++)
	{
		memset(dist_str,0,sizeof(dist_str));
//		sprintf(dist_str,"Anchor%c: %d cm %s, ",0x41 + i, now_data->Dist[i], (now_data->Cal_Flag >> i & 0x01) == 1 ? "Yes" : "No");
		if(i != ANCHOR_LIST_COUNT - 1)
		{
			if((now_data->Cal_Flag >> i & 0x01) == 1)
			{
				sprintf(dist_str,"Anc%c: %d cm , ",0x41 + i, now_data->Dist[i]);
			}
			else
			{
				sprintf(dist_str,"Anc%c: %d cm , ",0x41 + i, -1);
			}
		}
		else
		{
			if((now_data->Cal_Flag >> i & 0x01) == 1)
			{
				sprintf(dist_str,"Anc%c: %d cm\r\n",0x41 + i, now_data->Dist[i]);
			}
			else
			{
				sprintf(dist_str,"Anc%c: %d cm\r\n",0x41 + i, -1);
			}
		}	
		strcat(Tag_ouput_dist_str, dist_str);
	}
}

void Prepare_ascii_rtls_output(Cal_data_t* now_data, uint8_t mode)
{
	memset(Tag_ouput_rtls_str,0,sizeof(Tag_ouput_rtls_str));
	if(mode == 1)  //��ά��λģʽ
	{
		sprintf(Tag_ouput_rtls_str,"Rtls:X = %d cm , Y = %d cm %s\r\n",now_data->x,now_data->y,(now_data->Cal_Flag >> 16 & 0x01) == 1 ? "Yes" : "No");
	}
	else if(mode == 2)  //��ά��λģʽ
	{
		sprintf(Tag_ouput_rtls_str,"Rtls:X = %d cm , Y = %d cm, Z = %d cm %s\r\n",now_data->x, now_data->y, now_data->z, (now_data->Cal_Flag >> 16 & 0x01) == 1 ? "Yes" : "No");
	}
	
}

void Prepare_tag_result_output(Cal_data_t* now_data, uint8_t format, uint8_t mode)
{
	memset(Tag_Usart_Str,0,sizeof(Tag_Usart_Str));
	switch(mode)
	{
		case 0:  //���ģʽ
		{
			if(format & TAG_OUTPUT_DIST)
			{
				Prepare_ascii_dist_output(now_data);
				strcpy(Tag_Usart_Str,Tag_ouput_dist_str);
			}
			break;
		}
		case 1:  //��άģʽ
		case 2:  //��άģʽ
		{
			if(format & TAG_OUTPUT_RTLS)
			{
				Prepare_ascii_rtls_output(now_data, mode);
				strcpy(Tag_Usart_Str,Tag_ouput_rtls_str);
			}
			if(format & TAG_OUTPUT_DIST)
			{
				Prepare_ascii_dist_output(now_data);
				strcat(Tag_Usart_Str,Tag_ouput_dist_str);
			}
		}
		default:break;
	}	
}


uint8_t Twr_Algorithm(uint16_t *cal_dist)
{
	double Ra, Rb, Da, Db;
	int64_t tof_dtu;
	double dist_temp;
	uint16_t temp = 0;
	uint8_t i;
	
	for(i=0;i<6;i++)
	{
		if(Time_ts[i] == 0)
			return 0;
	}
	
	Ra = (double)(Time_ts[3] - Time_ts[0]);//Tround1 = T4 - T1  
	Rb = (double)(Time_ts[5] - Time_ts[2]);//Tround2 = T6 - T3 
	Da = (double)(Time_ts[4] - Time_ts[3]);//Treply2 = T5 - T4  
	Db = (double)(Time_ts[2] - Time_ts[1]);//Treply1 = T3 - T2 	
	tof_dtu = (Ra * Rb - Da * Db) / (Ra + Rb + Da + Db);    //���㹫ʽ
	dist_temp = tof_dtu * FLIGHT_OF_UWBTIME;                 //����=����*����ʱ��
	
	if(dist_temp < 0)
	{		
		if(dist_temp < -0.55)  //������ ����
		{			
			return 0;
		}
		else  //��Ϊ�ȽϽӽ����µļ���ʱ������� �������Ϊ0
		{
			dist_temp = 0;
			*cal_dist = temp;
			return 1;
		}				
	}
	temp = dist_temp * 100;                               //ת����λΪcm�ľ���
	*cal_dist = temp;
	return 1;
}



int16_t Twr_CalDist(uint8_t tag_id, uint16_t *cal_dist)
{
	float temp = 0.0f;
	if(Twr_Algorithm(cal_dist) == 0)
	{
		return 0;
	}
	temp = *cal_dist;
	*cal_dist = Average_ex(tag_id,temp);  //�����˲�
	return 1;
}

int16_t Range_CalDist(uint16_t *cal_dist)
{
	if(Twr_Algorithm(cal_dist) == 0)
	{
		return -1;
	}
	return 0;
}





