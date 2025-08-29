#include "filter.h"
#include <stdint.h>
#include <stdbool.h>

Tag_t tag_DistList[TAG_USE_MAX_NUM];

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief ����ȥ��ֵȡ��ֵ�˲�
 *
 * input parameters
 * @param index    ��ǩID 
 * @param dist_now ���β�þ��� 
 * output parameters
 * �˲���ľ���ֵ
 */
float Average_ex(int index, float dist_now)
{
	float result = 0;

	float max,min;
	char i;
	Tag_t *t = &tag_DistList[index];
	
	if(t->Dist_index >= 3)
	 t->Dist_index = 0;

	t->Dist[t->Dist_index] = dist_now;  //���뱾�β��ֵ
	
	t->Dist_index = t->Dist_index + 1;
	
	//�ҵ������Сֵ
	max = t->Dist[0];
	min = max;
	result = max;
	for(i = 1;i < 3;i++)
	{
		if(max < t->Dist[i])
			max = t->Dist[i];
		if(min > t->Dist[i])
			min = t->Dist[i];
		result += t->Dist[i];
	}
	return (result - max - min);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief �������˲� ģ��Ϊ����λ�ú���һ����ͬ
 *
 * input parameters
 * @param ResrcData    ���μ���õ�λ������
 * @param ProcessNiose_Q �趨�Ĺ������� ����������Q
 * @param ProcessNiose_Q �趨�Ĳ������� ����������R
 * @param idx ��ǩID
 * @param mode �˲����� 0�˲�����x���� 1�˲�y���� 2�˲�z����
 * output parameters
 * �˲����Ӧ������ֵ
 */
float KalmanFilter(const float ResrcData,float ProcessNiose_Q,float MeasureNoise_R, char idx, char mode)
{

    float R = MeasureNoise_R;
    float Q = ProcessNiose_Q;
     
	Tag_t *t = &tag_DistList[idx];
	
    float last_data;
    float filter_data;

    float p_mid;
    float p_now;

    float kg;

	//����ģʽд����Ϣ
	if(mode == 0)
	{
		last_data = t->last_x;                       
		p_mid = t->p_last_x + Q;          //Ԥ�Ȿ�����
	}      
	else if(mode == 1)
	{
		last_data = t->last_y; 
		p_mid = t->p_last_y + Q;         //Ԥ�Ȿ�����
	}			
	else if(mode == 2)
	{
		last_data = t->last_z;
		p_mid = t->p_last_z + Q;         //Ԥ�Ȿ�����
	}


	//����
	kg = p_mid/(p_mid+R);             //���±��ο���������    
	filter_data=last_data+kg*(ResrcData-last_data);   //���ݱ��ι۲�ֵԤ�Ȿ�����
	p_now=(1-kg)*p_mid;               //�������  
		
	//�����˲���Ϣ
	if(mode == 0)
	{
		t->last_x = filter_data;                       
		t->p_last_x = p_now;  
	}      
	else if(mode == 1)
	{
		t->last_y = filter_data;                       
		t->p_last_y = p_now;
	}			
	else if(mode == 2)
	{
		t->last_z = filter_data;                       
		t->p_last_z = p_now;
	}	
		
    return filter_data;

}


