#ifndef __Filter_H
#define __Filter_H

#include <stdint.h>
#include <stdbool.h>
#include "../core/data_types.h"

typedef struct 
{
	float Dist[3];
//	double Last_Dist;
	char Dist_index;
	float p_last_x;
	float p_last_y;
	float p_last_z;
	float last_x;
	float last_y;
	float last_z;
}__attribute((packed)) Tag_t;

extern Tag_t tag_DistList[TAG_USE_MAX_NUM];                //��ǩ�˲����뻺��

//double LD(int index, double dist_now);
float Average_ex(int index, float dist_now);
float KalmanFilter(const float ResrcData,float ProcessNiose_Q,float MeasureNoise_R, char idx, char mode);
#endif
