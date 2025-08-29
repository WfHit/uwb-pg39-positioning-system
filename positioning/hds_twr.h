#ifndef __HDS_TWR_H
#define __HDS_TWR_H

#include "twr.h"
#include "../core/data_types.h"
#include <stdint.h>

#define ANCHOR_WAITFINAL_MAX 120                         //��վ�ȴ�resp�����ʱ�� 0.1msΪ��λ

#define TX_ANC_INFORM_LEN 50
extern u8 TX_ANC_INFORM_BUFF[TX_ANC_INFORM_LEN];         //��վ����Inform������

#define TX_TAG_POLL_FIX_LEN 8
#define TX_TAG_POLL_LEN TX_TAG_POLL_FIX_LEN + UWB_COMMU_DATA_MAXLEN
extern uint8_t TX_TAG_POLL_BUFF[TX_TAG_POLL_LEN];             //��ǩ����Poll������

#define TX_ANC_RESP_FIX_LEN 8
#define TX_ANC_RESP_LEN TX_ANC_RESP_FIX_LEN + UWB_COMMU_DATA_MAXLEN
extern uint8_t TX_ANC_RESP_BUFF[TX_ANC_RESP_LEN];             //��վ����Resp������

#define TX_TAG_FINAL_LEN 78
extern uint8_t TX_TAG_FINAL_BUFF[TX_TAG_FINAL_LEN];           //��ǩ����Final������

#define TX_ANC_REQ_LEN 6
extern uint8_t TX_ANC_REQ_BUFF[TX_ANC_REQ_LEN];               //��վ����Request������  ����վ

#define TX_ANC_REPLY_LEN 9
extern uint8_t TX_ANC_REPLY_BUFF[TX_ANC_REPLY_LEN];           //��վ����Reply������  �λ�վ

#define RX_MAX_LEN 127
extern uint8_t HDS_rx_buffer[RX_MAX_LEN];                           //���ջ���

extern uint16_t Dis_cal;
extern uint8_t frame_now;
extern uint16_t Anc_recvFinal_timeflag;

extern uint8_t SYS_ANC_RESP_FLAG;
extern uint8_t SYS_ANC_FINAL_FLAG;
extern uint8_t SYS_ANC_DELAY_SEND_FLAG;


uint8_t HDS_TWR_Send_Resp(uint8_t A_ID, uint8_t B_ID, uint8_t En);
uint8_t HDS_TWR_Recv_FinalAndCal(uint8_t A_ID ,uint8_t B_ID, uint16_t recv_timeout);

void Mode_MainAnchor_HDS(void);
void Mode_Tag_HDS(void);
void Mode_Sub_Anchor_HDS(void);

#endif
