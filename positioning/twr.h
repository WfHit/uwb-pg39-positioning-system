#ifndef __TWR_H
#define __TWR_H

#include "../core/data_types.h"
#include "../core/system_config.h"
#include <stdint.h>
#include <stdbool.h>

typedef int64_t int64;
typedef uint64_t uint64;



/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
UWB΢�루UUS�����豸ʱ�䵥λ��DTU��Լ15.65 ps����ת��ϵ����
 * 1 uus = 512 / 499.2 �s and 1 �s = 499.2 * 128 dtu. 
  1 UUS��512��499.2��1��499.2��128 DTU��*/
#define UUS_TO_DWT_TIME 65536

#define SPEED_OF_LIGHT 299702547

#define FLIGHT_OF_UWBTIME  SPEED_OF_LIGHT * DWT_TIME_UNITS

#define FINAL_MSG_TS_LEN 4  //ʱ�����ݳ���

#define FRAME_LEN_MAX      (127)

#define FRAME_LEN_MAX_EX   (1023)

typedef struct 
{
	uint32_t Cal_Flag;  //���ɹ���־λ ��8λ 1����λ�ɹ� ��0-7��1�ֱ����A-H��վ���ɹ�
	int16_t x;         //�������x���� ��λcm
	int16_t y;         //�������y���� ��λcm
	int16_t z;         //�������z���� ��λcm
	uint16_t Dist[ANCHOR_LIST_COUNT];     //��ñ�ǩ��A-H��վ�ľ���
}Cal_data_t;

extern uint32_t status_reg;
extern uint32_t frame_len;        		//DWM1000�շ����ݰ����Ȼ���	
extern uint32_t frame_seq_nb;          //֡���кţ�ÿ�δ���������

extern uint32_t Time_ts[6];  					                                   //����ʱ�仺���¼
//extern u16   Cal_Last_XYZ[100][3];                               //�ش�����ʹ�õĻ��棬���ڴλ�վ������վģʽ��õı�ǩ������һ�λش�����ǩ
//extern uint32_t Cal_Last_Dist[100][8];        //���100����ǩ��8����վ���ֵ
extern uint32_t Dist_Cal_All[ANCHOR_LIST_COUNT];        //��ű��β�������
extern Cal_data_t Cal_data[TAG_USE_MAX_NUM];

extern dwt_rxdiag_t rx_diag;                                           //������Ϣ

#define TAG_USART_BUF_DIST_LEN 350
#define TAG_USART_BUF_RTLS_LEN 60
#define TAG_USART_BUF_MAXLEN TAG_USART_BUF_DIST_LEN + TAG_USART_BUF_RTLS_LEN
extern char Tag_Usart_Str[TAG_USART_BUF_MAXLEN];
void Prepare_tag_result_output(Cal_data_t* now_data, uint8_t format, uint8_t mode);

void final_msg_get_ts(const uint8_t *ts_field, uint32_t *ts);
void final_msg_get_dist(const uint8_t *ts_field, uint32_t *dist);
uint64 get_tx_timestamp_u64(void);
uint64 get_rx_timestamp_u64(void);
void final_msg_set_ts(uint8_t *ts_field, uint32_t ts);
void final_msg_set_dist(uint8_t *ts_field, uint32_t dist);
int16_t Twr_CalDist(uint8_t tag_id,uint16_t *cal_dist);
int16_t Range_CalDist(uint16_t *cal_dist);


#endif

