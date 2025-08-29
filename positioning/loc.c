#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "loc.h"
#include "../libraries/math_lib/include/arm_math.h"
#include "array.h"

#define MASS_THRESH 15.0f
#define TAYLOR_2D_THRESH 5.0f
#define TAYLOR_3D_THRESH 5.0f

uint8_t Cal_2D_AllCenterMass(float* Anc_A,float* Anc_B,float* Anc_C, float* Cal_result);

/// <summary>
float Triangle_scale=1.1;    //��վɸѡ����ʹ�õı�������

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief �ж�����Բ�Ƿ��ཻ
 *
 * input parameters
 * @param r1 Բ1�뾶
 * @param r2 Բ2�뾶 
 * @param d  ��ԲԲ�ľ�
 * output parameters
 * 1������Բ�ཻ 0���ཻ
 */
uint8_t Judge_CircleIntersection(double r1, double r2, double d)
{
	 if(r1 + r2 < d) //�������� ��΢�����ܷ�����
	 {
		r1 *= 1.05;
		r2 *= 1.05;
		if (r1 + r2 < d)
			return 0;
	 }
	 if(fabs(r1 - r2) > d) //���ж��Ƿ��ں�
	 {
		 if(r1 > r2)
			r2*=1.05;
		 else
			r1*=1.05;
		 if(fabs(r1 - r2) > d)
			return 0;
	 }
	 return 1;
}


/*! ------------------------------------------------------------------------------------------------------------------
 * @brief ��������Ķ�άƽ���ŷʽ����
 *
 * input parameters
 * @param x1 y1 ��1�Ķ�ά����
 * @param x2 y2 ��2�Ķ�ά����
 * output parameters
 * ����ŷʽ����
 */
double Cal_Dist(double x1, double y1, double x2, double y2)
{
	return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
}


/*! ------------------------------------------------------------------------------------------------------------------
 * @brief �����������άƽ���ŷʽ����
 *
 * input parameters
 * @param x1 y1 z1 ��1����ά����
 * @param x2 y2 z2 ��2����ά����
 * output parameters
 * ����ŷʽ����
 */
double Cal_Dist_3D(double x1, double y1, double z1, double x2, double y2, double z2)
{
	return sqrt(pow((x1-x2),2)+pow((y1-y2),2)+pow((z1-z2),2));
}


/*! ------------------------------------------------------------------------------------------------------------------
 * @brief ���������ڶ�ӦԪ�صĿ������򣨾���ԭ�����ٶȣ�
 *
 * input parameters
 * @param (*sort_data)[2] ��Ҫ�����n*2���� 
           Ҫ�Ƚϵ���[][0]��Ԫ�ش�С�����յõ�[][0]��С�������������
 * @param left_idx  ����߿�ʼ������
 * @param right_idx ���ұ߿�ʼ������
 * output parameters  none 
 */
void Quick_Sort_withdata(uint16_t (*sort_data)[2], uint16_t left_idx, uint16_t right_idx)
{
	if(left_idx >= right_idx)
		return;
	
	uint16_t i = left_idx, j = right_idx;
	uint16_t base = sort_data[left_idx][0];
	uint16_t base_temp = sort_data[left_idx][1];
	while(i < j)
	{
		//���Ҳ����С��base��Ԫ��
		while(i < j && sort_data[j][0] >= base)
			j--;
		//�ҵ���
		if(i < j)
		{
			//��base���ұ��ҵ���ֵ����
			sort_data[i][0] = sort_data[j][0];
			sort_data[i][1] = sort_data[j][1];
			i++;  //i+1��Ϊ�˺��濪ʼ�Ĵ�����ұ�base���ֵ
		}
		//�������Ҵ���base��Ԫ��
		while(i < j && i < right_idx && sort_data[i][0] <= base)  
			i++;
		//�ҵ���
		if(i < j)
		{
			//��base������ҵ���ֵ����
			sort_data[j][0] = sort_data[i][0];
			sort_data[j][1] = sort_data[i][1];
			j--;  //j-1��Ϊ�˺��濪ʼ�Ĵ��ұ��ұ�baseС��ֵ
		}		
	}
	//i=j�� ��һ������ ��base��м�
	sort_data[i][0] = base;
	sort_data[i][1] = base_temp;
	if(i != 0)
		Quick_Sort_withdata(sort_data,left_idx,i - 1);  //���ݹ�
	if(i != right_idx)
		Quick_Sort_withdata(sort_data,i + 1,right_idx); //�Ҳ�ݹ�
}



/*! ------------------------------------------------------------------------------------------------------------------
 * @brief �жϸ�������վ�Ƿ��ʺϽ��ж�ά���� 
 *        �ж��ǲ�����Ч�����Σ�ɸѡ�ų���Ч�����Σ��Ա㱣֤���õ�����
 * input parameters
 * @param *anc1 �������Ļ�վ1��������
 * @param *anc2 �������Ļ�վ2��������
 * @param *anc3 �������Ļ�վ3��������
 * output parameters 
   1�����ڽ��� 0������
 */
uint8_t Judge_2D (float *anc1,float *anc2, float *anc3)//
{
	float Dist_12,Dist_13,Dist_23;	
	Dist_12=Cal_Dist(anc1[0],anc1[1],anc2[0],anc2[1]);//1��2��վ����	
	Dist_13=Cal_Dist(anc1[0],anc1[1],anc3[0],anc3[1]);//1��3��վ����
	Dist_23=Cal_Dist(anc2[0],anc2[1],anc3[0],anc3[1]);//2��3��վ����
	 
	 //��Բ�ཻ���
	if((Dist_12+Dist_13)>(Dist_23*Triangle_scale) && (Dist_12+Dist_23)>(Dist_13*Triangle_scale)  &&  (Dist_13+Dist_23)>(Dist_12*Triangle_scale))
	{
		//�ҵ����߳�
		float max_dist = Dist_12;
		if (max_dist < Dist_13)
			max_dist = Dist_13;
		if (max_dist < Dist_23)
			max_dist = Dist_23;
		//�жϻ�վ��Ӧ�Ĳ��ֵ�Ƿ񶼴��������ı߳�
		//max_dist *= 1.2;
		if (max_dist < anc1[2] && max_dist < anc2[2] && max_dist < anc3[2])
			return 0;
		if(Judge_CircleIntersection(anc1[2],anc2[2],Dist_12) && Judge_CircleIntersection(anc1[2],anc3[2],Dist_13) && Judge_CircleIntersection(anc2[2],anc3[2],Dist_23))
			return 1;
		else
			return 0;
	}	 
	else 
		return 0;
				 
}	

	
/*! ------------------------------------------------------------------------------------------------------------------
 * @brief �жϸ��ĸ���վ�Ƿ��ʺϽ�����ά���㣨5.0�汾���º�ʹ�ã�
 *        �ж��ǲ�����Ч�����Σ�ɸѡ�ų���Ч�����Σ��Ա㱣֤���õ�����
 * input parameters
 * @param x1 y1 z1 ��1����ά���� r1��1���β�����
 * @param x2 y2 z2 ��2����ά���� r2��2���β�����
 * @param x3 y3 z3 ��3����ά���� r3��3���β�����
 * @param x4 y4 z4 ��4����ά���� r4��4���β�����
 * output parameters 
   1�����ڽ��� 0������
 */
uint8_t Judge_3D (float x1, float y1,float z1, float r1, 
				  float x2, float y2, float z2, float r2, 
			      float x3, float y3, float z3, float r3,
				  float x4, float y4, float z4, float r4 ) 
{
	float dist_all_2D[6];
	float dist_all_3D[6];
	uint8_t i;
	float max_dist;

	//�жϻ�վ��ˮƽ�����Ƿ���ÿ�������һ��������
	dist_all_2D[0] = Cal_Dist(x1,y1,x2,y2);//1��2��վ����
	dist_all_2D[1] = Cal_Dist(x1,y1,x3,y3);//1��3��վ����
	dist_all_2D[2] = Cal_Dist(x1,y1,x4,y4);//1��4��վ����
	dist_all_2D[3] = Cal_Dist(x2,y2,x3,y3);//2��3��վ����
	dist_all_2D[4] = Cal_Dist(x2,y2,x4,y4);//2��4��վ����
	dist_all_2D[5] = Cal_Dist(x3,y3,x4,y4);//3��4��վ����

	if(!(dist_all_2D[0]+dist_all_2D[1] > dist_all_2D[3]*Triangle_scale)    //1 2 3��վ�ж�
			&& !(dist_all_2D[0]+dist_all_2D[3] > dist_all_2D[1]*Triangle_scale) 
			&& !(dist_all_2D[1]+dist_all_2D[3] > dist_all_2D[0]*Triangle_scale))
		return 0;

	if(!(dist_all_2D[0]+dist_all_2D[2] > dist_all_2D[4]*Triangle_scale)   //1 2 4��վ�ж�
			&& !(dist_all_2D[0]+dist_all_2D[4] > dist_all_2D[2]*Triangle_scale) 
			&& !(dist_all_2D[2]+dist_all_2D[4] > dist_all_2D[0]*Triangle_scale))
		return 0;	
	
	if(!(dist_all_2D[1]+dist_all_2D[2] > dist_all_2D[5]*Triangle_scale)   //1 3 4��վ�ж�
			&& !(dist_all_2D[1]+dist_all_2D[5] > dist_all_2D[2]*Triangle_scale) 
			&& !(dist_all_2D[2]+dist_all_2D[5] > dist_all_2D[1]*Triangle_scale))
		return 0;	
	
	if(!(dist_all_2D[3]+dist_all_2D[4] > dist_all_2D[5]*Triangle_scale)   // 2 3 4��վ�ж�
			&& !(dist_all_2D[3]+dist_all_2D[5] > dist_all_2D[4]*Triangle_scale) 
			&& !(dist_all_2D[4]+dist_all_2D[5] > dist_all_2D[3]*Triangle_scale))
		return 0;	
	

	//�жϻ�վ���������� ͨ�����ֵ�жϱ�ǩ���Ƿ������ĸ���վ�ڲ�
	dist_all_3D[0] = Cal_Dist_3D(x1,y1,z1,x2,y2,z2);//1��2��վ����
	dist_all_3D[1] = Cal_Dist_3D(x1,y1,z1,x3,y3,z3);//1��3��վ����
	dist_all_3D[2] = Cal_Dist_3D(x1,y1,z1,x4,y4,z4);//1��4��վ����
	dist_all_3D[3] = Cal_Dist_3D(x2,y2,z2,x3,y3,z3);//2��3��վ����
	dist_all_3D[4] = Cal_Dist_3D(x2,y2,z2,x4,y4,z4);//2��4��վ����
	dist_all_3D[5] = Cal_Dist_3D(x3,y3,z3,x4,y4,z4);//3��4��վ����
	
	//�ҵ����ĶԽ���ֵ
	max_dist = dist_all_3D[0];
	for(i=1;i<6;i++)
	{
		if(max_dist < dist_all_3D[i])
			max_dist = dist_all_3D[i];
	}
	
	//�жϻ�վ��Ӧ�Ĳ��ֵ�Ƿ���������ĶԽ��� ��������Ϊ��ǩ�ڻ�վ��Χ������
	if(r1 > max_dist && r2 > max_dist && r3 > max_dist && r4 > max_dist)
		return 0;
	else
		return 1;
}					


/*! ------------------------------------------------------------------------------------------------------------------
 * @brief �ж�����ֱ��ˮƽ�����Ƿ��ཻ��5.0�汾���º��ã�
 * input parameters
 * @param x1 y1  ��1�Ķ�ά����
 * @param x2 y2  ��2�Ķ�ά����
 * @param x3 y3  ��3�Ķ�ά����
 * @param x4 y4  ��4�Ķ�ά����
 * output parameters 
   1�����ཻ 0���ཻ
 */
uint8_t Rtls_Judge_LineIntersect(double x1, double y1, double x2, double y2, double x3, double y3 , double x4, double y4)
{
	//ֱ��L1��(x1,y1)��(x2,y2)����ֱ��
	//ֱ��L2��(x3,y3)��(x4,y4)����ֱ��
	double k1,k2;
	//�ж��Ƿ�����ֱ��ֱ��
	if(x1 == x2 || x3 == x4)
	{
		//�������Ǵ�ֱ��
		if(x1 == x2 && x3 == x4)
			return 0;
		else  //����һ����		
			return 1;				
	}
	else
	{
		//������ֱ��б��
		k1 = (y1 - y2) / (x1 - x2);
		k2 = (y3 - y4) / (x3 - x4);
		if(k1 * k2 < 0)
			return 1;
		else
			return 0;

	}		
}


/*! ------------------------------------------------------------------------------------------------------------------
 * @brief �жϸ��ĸ���վ�Ƿ��ʺϽ�����ά���㣨5.0�汾���º��ã�
 *        ��Judge_3D�Ļ��������ж��ĸ���վ�Ƿ��������ͶԽǰڷ�        
 * input parameters
 * @param *anc0 �������Ļ�վ0��������
 * @param *anc1 �������Ļ�վ1��������
 * @param *anc2 �������Ļ�վ2��������
 * @param *anc3 �������Ļ�վ3��������
 * output parameters 
   1�����ڽ��� 0����
 */
uint8_t Judge_3D_New(float *anc0,float *anc1, float *anc2, float *anc3)
{
//	uint8_t flag = 0;
	uint8_t i,j;

	float ancs[4][3];
	float x0 = anc0[0];
	float y0 = anc0[1];
	float z0 = anc0[2];
	float r0 = anc0[3];
	float x1 = anc1[0];
	float y1 = anc1[1];
	float z1 = anc1[2];
	float r1 = anc1[3];
	float x2 = anc2[0];
	float y2 = anc2[1];
	float z2 = anc2[2];	
	float r2 = anc2[3];
	float x3 = anc3[0];
	float y3 = anc3[1];
	float z3 = anc3[2];	
	float r3 = anc3[3];
  
	ancs[0][0] = x0;
	ancs[0][1] = y0;
	ancs[0][2] = z0;
	ancs[1][0] = x1;
	ancs[1][1] = y1;
	ancs[1][2] = z1;
	ancs[2][0] = x2;
	ancs[2][1] = y2;
	ancs[2][2] = z2;
	ancs[3][0] = x3;
	ancs[3][1] = y3;
	ancs[3][2] = z3;
	
	if(Judge_3D(x0, y0, z0, r0, x1, y1, z1, r1, x2, y2, z2, r2, x3, y3, z3, r3))
	{
		//���ݸ߶ȴӸߵ��������ĸ���վ
		float sort_z[4] = {0};
		float temp;
//		double k = 0;
		uint8_t temp_index;
		uint8_t sort_index[4] = {0,1,2,3};
		sort_z[0]=z0;
		sort_z[1]=z1;
		sort_z[2]=z2;
		sort_z[3]=z3;
		
		
		for(i=0;i<3;i++)
		{
			for(j=i+1;j<4;j++)
			{
				if(sort_z[i]<sort_z[j])
				{
					temp = sort_z[i];
					sort_z[i] = sort_z[j];
					sort_z[j] = temp;
					temp_index = sort_index[i];
					sort_index[i] = sort_index[j];
					sort_index[j] = temp_index;
				}
			}
		}
		
		//ȡ�θ߼��ε�Ҫ����100���ϲſ�������߶Ȳ�����
		if(sort_z[1] - sort_z[2] < 100)
			return 0;
		
		//�������߻�վ�����ͻ�վ����ֱ�ߵ�б�� �ж��Ƿ�����ֱ���ཻ
		if(Rtls_Judge_LineIntersect(ancs[sort_index[0]][0],ancs[sort_index[0]][1],ancs[sort_index[1]][0],ancs[sort_index[1]][1],
			 ancs[sort_index[2]][0],ancs[sort_index[2]][1],ancs[sort_index[3]][0],ancs[sort_index[3]][1]))
			return 1;	
		else
			return 0;
	}	
  return 0;	
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief �Ӽ����վ�л�ȡ��վ����������ľ���
 *
 * input parameters
 * @param (*anc_list)[4] ��վ���ݶ�ά����
 * @param anc_num ��վ����
 * output parameters
 * 
 */
uint16_t Cal_Max_3D_Dist(float (*anc_list)[4], uint8_t anc_num)
{
	uint8_t i = 0, j = 0;
	uint16_t dist_temp = 0;	
	uint16_t max_dist = 0;
	for(i=0;i<anc_num-1;i++)
	{
		for(j=i+1;j<anc_num;j++)
		{
			dist_temp = Cal_Dist_3D(anc_list[i][0],anc_list[i][1],anc_list[i][2],anc_list[j][0],anc_list[j][1],anc_list[j][2]);
			if(dist_temp > max_dist)
			{
				max_dist = dist_temp;
			}
		}
	}
	return max_dist;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief ��ά�������� ����ȫ�����㷨
 *  ��ʽΪAX=B => X = (AT * A)^-1 * AT * B
 *     -             -       -        -       -                    -
 *     | -2xa -2ya 1  |      |    x    |      | da^2 - xa^2 - ya^2  |
 * A = | -2xb -2yb 1  |  X = |    y    |  B = | db^2 - xb^2 - yb^2  |
 *     | -2xc -2yc 1  |      | x^2+y^2 |      | dc^2 - xc^2 - yc^2  |
 *     -             -       -        -       -                     -
 * input parameters
 * @param Anc_A  �������Ļ�վA��������
 * @param Anc_B  �������Ļ�վB��������
 * @param Anc_C  �������Ļ�վC��������
 * @param *Cal_result  ����������
 * output parameters 
   1��������ɹ� 0ʧ��
 */
uint8_t Cal_2D_AllCenterMass(float* Anc_A,float* Anc_B,float* Anc_C, float* Cal_result)
{
	uint8_t cal_ok = 1;
	arm_matrix_instance_f32 A_mat;
	arm_matrix_instance_f32 X_mat;
	arm_matrix_instance_f32 B_mat;
	arm_matrix_instance_f32 AT_mat;
	arm_matrix_instance_f32 ATA_mat;
	arm_matrix_instance_f32 ATA_inv_mat;
	arm_matrix_instance_f32 cal_mat;
	arm_status status = ARM_MATH_SUCCESS;
	//�����Ѿ��̶��˾����С�ͽ��� ����Ҫ�������������
	float32_t A_data[9] =
	{
		-2 * Anc_A[0],-2 * Anc_A[1], 1,
		-2 * Anc_B[0],-2 * Anc_B[1], 1,
		-2 * Anc_C[0],-2 * Anc_C[1], 1,
	};
	float32_t B_data[3] =
	{
		Anc_A[2] * Anc_A[2] - Anc_A[0] * Anc_A[0] - Anc_A[1] * Anc_A[1],
		Anc_B[2] * Anc_B[2] - Anc_B[0] * Anc_B[0] - Anc_B[1] * Anc_B[1],
		Anc_C[2] * Anc_C[2] - Anc_C[0] * Anc_C[0] - Anc_C[1] * Anc_C[1],
	};
	float32_t AT_data[9] ={0};
	float32_t ATA_data[9] ={0};
	float32_t ATA_inv_data[9] ={0};
	float32_t cal_data[9] ={0};
	float32_t X_data[3] ={0};
	
	arm_mat_init_f32(&A_mat, 3, 3, (float32_t *)A_data);
	arm_mat_init_f32(&AT_mat, 3, 3, (float32_t *)AT_data);
	arm_mat_init_f32(&ATA_mat, 3, 3, (float32_t *)ATA_data);
	arm_mat_init_f32(&ATA_inv_mat, 3, 3, (float32_t *)ATA_inv_data);
	arm_mat_init_f32(&cal_mat, 3, 3, (float32_t *)cal_data);
	arm_mat_init_f32(&B_mat, 3, 1 ,(float32_t *)B_data);
	arm_mat_init_f32(&X_mat, 3, 1 ,(float32_t *)X_data);
	
	do
	{
		status = arm_mat_trans_f32(&A_mat,&AT_mat);
		if(status != ARM_MATH_SUCCESS)
		{
			cal_ok = 0;
			break;
		}
		status = arm_mat_mult_f32(&AT_mat,&A_mat,&ATA_mat);
		if(status != ARM_MATH_SUCCESS)
		{
			cal_ok = 0;
			break;
		}
		status = arm_mat_inverse_f32(&ATA_mat,&ATA_inv_mat);
		if(status != ARM_MATH_SUCCESS)
		{
			cal_ok = 0;
			break;
		}
		status = arm_mat_mult_f32(&ATA_inv_mat,&AT_mat,&cal_mat);
		if(status != ARM_MATH_SUCCESS)
		{
			cal_ok = 0;
			break;
		}
		status = arm_mat_mult_f32(&cal_mat,&B_mat,&X_mat);
		if(status != ARM_MATH_SUCCESS)
		{
			cal_ok = 0;
			break;
		}
	}
	while(0);
	
	if(cal_ok)
	{
		Cal_result[0] = X_mat.pData[0];
		Cal_result[1] = X_mat.pData[1];	
		return 1;
	}	
	else
	{
		Cal_result[0] = 0;
		Cal_result[1] = 0;	
		return 0;
	}	
}


/*! ------------------------------------------------------------------------------------------------------------------
 * @brief ����㼯�ϵ�����      
 * input parameters
 * @param (*points)[2] ��ά�㼯��
 * @param len ��ά������
 * @param *result ����ó���Щ�������
 * output parameters 
 * none
 */
void Cal_massCenter(float (*points)[2], uint8_t len, float* result)
{
	uint8_t i;
	float mass[2] = {0.0f};
	for(i=0;i<len;i++)
	{
		mass[0] += points[i][0];
		mass[1] += points[i][1];
	}
	mass[0] /= len;
	mass[1] /= len;
	result[0] = mass[0];
	result[1] = mass[1];
}


/*! ------------------------------------------------------------------------------------------------------------------
 * @brief ����ɸѡ�㷨 �õ���������������̩�������ĳ�ֵ      
 * input parameters
 * @param (*points)[2] ��ά�㼯��
 * @param *result ����ó���Щ�������
 * output parameters 
 * none
 */
void CenterMass_Select(float (*points)[2], uint8_t len, float* result)
{	
	uint8_t i = 0, max_idx = 0;
	float temp_dist, max_dist = 0;	
	float first_mass[2] = {0.0f}, new_mass[2] = {0.0f}, temp[2] = {0.0f};
	Cal_massCenter(points,len,first_mass);
	//�ҳ���������Զ�������
	for(i=0;i < len;i++)
	{
		temp_dist = Cal_Dist(first_mass[0],first_mass[1],points[i][0],points[i][1]);
		if(max_dist < temp_dist)
		{
			max_dist = temp_dist;
			max_idx = i;
		}
	}
	//�����������ų����ټ���һ������
	if(max_idx != len - 1)
	{
		temp[0] = points[max_idx][0];
		temp[1] = points[max_idx][1];
		for(i=max_idx;i<len-1;i++)
		{
			points[i][0] = points[i+1][0];
			points[i][1] = points[i+1][1];
		}
		points[len - 1][0] = temp[0];
		points[len - 1][1] = temp[1];
	}
	
	len--;
	Cal_massCenter(points,len,new_mass);
	if(Cal_Dist(first_mass[0],first_mass[1],new_mass[0],new_mass[1]) < MASS_THRESH)
	{
		//����ų����Ǹ���Զ����������ĺ͵�һ�������������С���趨����ֵ�������һ������
		result[0] = first_mass[0];
		result[1] = first_mass[1];
	}
	else
	{
		if(len == 1)  //�����ʣ1������ ����ȡƽ�����
		{
			result[0] = (first_mass[0] + new_mass[0]) / 2;
			result[1] = (first_mass[1] + new_mass[1]) / 2;
		}
		else  //�ݹ�
			CenterMass_Select(points,len,result);
	}
		
}


/*! ------------------------------------------------------------------------------------------------------------------
 * @brief ��ά��̩�������㷨������ԭ�����ٶȣ�
 * ͨ������õ��������ֵ(x0,y0)�����ʵ�ʵ�k����վ����ǩ����Ϊd(k),���ɱ�ǩ���㵽��k����վ����Ϊg(k)= sqrt((x0 - xk)^2 + (y0 - yk)^2)
 * ��ô����d(k) - g(k) = Error(k)
 * ��Ҫ��Error��С�����Թ��Ƶ���ֵ(x',y'),��g(k)����һ��̩��չ������
 *  ��ʽΪAX=B => X = (AT * A)^-1 * AT * B
 *     -                               -       -    -        -             -
 *     | (x0 - x1)/g(1) (y0 - y1)/g(1) |       | x' |        | d(1) - g(1) |
 * A = |       ...           ...       |   X = |    |    B = |     ...     |
 *     | (x0 - xk)/g(k) (y0 - yk)/g(k) |       | y' |        | d(k) - g(k) |
 *     -                               -       -    -        -             -
 * input parameters
 * @param (*Ancs)[3]  �����������л�վ����+�������� �������� x y dist
 * @param cal_num  �������Ļ�վ����
 * @param x0 y0    ����������ֵ
 * @param *Cal_result  ����������
 * output parameters 
   1��������ɹ� -1ʧ��
 */
int8_t Cal_Taylor_2D(float (*Ancs)[3],const uint16_t cal_num, float x0, float y0, float *Cal_result)
{
	uint16_t i;
	uint8_t cal_ok = 0;
	float a1,a2,dist;
	arm_matrix_instance_f32 A_mat;
	arm_matrix_instance_f32 X_mat;
	arm_matrix_instance_f32 B_mat;
	arm_matrix_instance_f32 AT_mat;
	arm_matrix_instance_f32 ATA_mat;
	arm_matrix_instance_f32 ATA_inv_mat;
	arm_matrix_instance_f32 cal_mat;
	arm_status status = ARM_MATH_SUCCESS;
	
	Array_t A_data = Array_create(cal_num * 2);
	Array_t B_data = Array_create(cal_num);
	Array_t AT_data = Array_create(2 * cal_num);
	Array_t ATA_data = Array_create(2 * 2);
	Array_t ATA_inv_data = Array_create(2 * 2);
	Array_t cal_data = Array_create(2 * cal_num );
	float32_t X_data[2] ={0};
	
	//�����鸳ֵ
	for(i=0;i<cal_num;i++)
	{
		dist = Cal_Dist(x0,y0,Ancs[i][0],Ancs[i][1]);
		a1 = x0 - Ancs[i][0];
		a2 = y0 - Ancs[i][1];
		Array_set(&A_data,i*2,a1 / dist);
		Array_set(&A_data,i*2+1,a2 / dist);
		Array_set(&B_data,i,Ancs[i][2] - dist);
	}
	memset(AT_data.array,0,AT_data.size);
	memset(ATA_data.array,0,ATA_data.size);
	memset(ATA_inv_data.array,0,ATA_inv_data.size);
	memset(cal_data.array,0,cal_data.size);
	
	//�����ʼ��
	arm_mat_init_f32(&A_mat, cal_num, 2, (float32_t *)A_data.array);
	arm_mat_init_f32(&AT_mat, 2, cal_num, (float32_t *)AT_data.array);
	arm_mat_init_f32(&ATA_mat, 2, 2, (float32_t *)ATA_data.array);
	arm_mat_init_f32(&ATA_inv_mat, 2, 2, (float32_t *)ATA_inv_data.array);
	arm_mat_init_f32(&cal_mat, 2, cal_num, (float32_t *)cal_data.array);
	arm_mat_init_f32(&B_mat, cal_num, 1 ,(float32_t *)B_data.array);
	arm_mat_init_f32(&X_mat, 2, 1 ,(float32_t *)X_data);
	
	do
	{
		status = arm_mat_trans_f32(&A_mat,&AT_mat);
		if(status != ARM_MATH_SUCCESS)	
		{
			cal_ok = 0;
			break;	
		}	
		status = arm_mat_mult_f32(&AT_mat,&A_mat,&ATA_mat);
		if(status != ARM_MATH_SUCCESS)
		{
			cal_ok = 0;
			break;	
		}		
		status = arm_mat_inverse_f32(&ATA_mat,&ATA_inv_mat);
		if(status != ARM_MATH_SUCCESS)
		{
			cal_ok = 0;
			break;	
		}		
		status = arm_mat_mult_f32(&ATA_inv_mat,&AT_mat,&cal_mat);
		if(status != ARM_MATH_SUCCESS)
		{
			cal_ok = 0;
			break;	
		}		
		status = arm_mat_mult_f32(&cal_mat,&B_mat,&X_mat);
		if(status != ARM_MATH_SUCCESS)
		{
			cal_ok = 0;
			break;	
		}
		cal_ok = 1;		
	}
	while(0);
		
	Array_free(&A_data);
	Array_free(&AT_data);
	Array_free(&ATA_data);
	Array_free(&ATA_inv_data);
	Array_free(&cal_data);
	Array_free(&B_data);

	if(cal_ok == 1)
	{
		Cal_result[0]=X_mat.pData[0];
		Cal_result[1]=X_mat.pData[1];
		return 1;
	}
	else
	{
		Cal_result[0]=0;
		Cal_result[1]=0;
		return -1;
	}
	
	
}



/*! ------------------------------------------------------------------------------------------------------------------
 * @brief ��ά�������� ��С���˷�����
 *  ��ʽΪAX=B => X = (AT * A)^-1 * AT * B
 *     -                                  -      -   -      -                                                        -
 *     | 2(x0 - x1) 2(y0 - y1) 2(z0 - z1) |      | x |      | x0^2 + y0^2 + z0^2 - d0^2 - x1^2 - y1^2 - z1^2 + d1^2  |
 * A = |     ...        ...        ...    |  X = | y |  B = |                         ...                            |
 *     | 2(x0 - xk) 2(y0 - yk) 2(z0 - zk) |      | z |      | x0^2 + y0^2 + z0^2 - d0^2 - xk^2 - yk^2 - zk^2 + dk^2  |
 *     -                                  -      -   -      -                                                        -
 * input parameters
 * @param (*Ancs)[4]  �����������л�վ����+�������� �������� x y z dist
 * @param cal_num  �������Ļ�վ����
 * @param *Cal_result  ����������
 * output parameters 
   1��������ɹ� 0ʧ��
 */
uint8_t Cal_3D_LeastSquare(const float (*Ancs)[4],const uint8_t cal_num, float* Cal_result)
{
	uint16_t i;
	uint8_t cal_ok = 0;
	float a1,a2,a3,r1;
	uint8_t n = cal_num -1;
	arm_matrix_instance_f32 A_mat;
	arm_matrix_instance_f32 X_mat;
	arm_matrix_instance_f32 B_mat;
	arm_matrix_instance_f32 AT_mat;
	arm_matrix_instance_f32 ATA_mat;
	arm_matrix_instance_f32 ATA_inv_mat;
	arm_matrix_instance_f32 cal_mat;
	arm_status status = ARM_MATH_SUCCESS;
	
	Array_t A_data = Array_create(n * 3);  
	Array_t B_data = Array_create(n);
	Array_t AT_data = Array_create(3 * n);
	Array_t ATA_data = Array_create(3 * 3);
	Array_t ATA_inv_data = Array_create(3 * 3);
	Array_t cal_data = Array_create(3 * n );
	float32_t X_data[3] ={0};
	
	r1 = powf(Ancs[0][0],2) + powf(Ancs[0][1],2) + powf(Ancs[0][2],2) - powf(Ancs[0][3],2);
	//�����鸳ֵ
	for(i=1;i<cal_num;i++)
	{
		a1 = Ancs[0][0] - Ancs[i][0];
		a2 = Ancs[0][1] - Ancs[i][1];
		a3 = Ancs[0][2] - Ancs[i][2];
		
		Array_set(&A_data,(i-1)* 3, a1 * 2);
		Array_set(&A_data,(i-1)* 3 + 1, a2 * 2);
		Array_set(&A_data,(i-1)* 3 + 2, a3 * 2);
		Array_set(&B_data,i-1,r1 - powf(Ancs[i][0],2) - powf(Ancs[i][1],2) - powf(Ancs[i][2],2) + powf(Ancs[i][3],2));
	}
	memset(AT_data.array,0,AT_data.size);
	memset(ATA_data.array,0,ATA_data.size);
	memset(ATA_inv_data.array,0,ATA_inv_data.size);
	memset(cal_data.array,0,cal_data.size);
	
	//�����ʼ��
	arm_mat_init_f32(&A_mat, n, 3, (float32_t *)A_data.array);
	arm_mat_init_f32(&AT_mat, 3, n, (float32_t *)AT_data.array);
	arm_mat_init_f32(&ATA_mat, 3, 3, (float32_t *)ATA_data.array);
	arm_mat_init_f32(&ATA_inv_mat, 3, 3, (float32_t *)ATA_inv_data.array);
	arm_mat_init_f32(&cal_mat, 3, n, (float32_t *)cal_data.array);
	arm_mat_init_f32(&B_mat, n, 1 ,(float32_t *)B_data.array);
	arm_mat_init_f32(&X_mat, 3, 1 ,(float32_t *)X_data);
	
	do
	{
		status = arm_mat_trans_f32(&A_mat,&AT_mat);
		if(status != ARM_MATH_SUCCESS)	
		{
			cal_ok = 0;
			break;	
		}	
		status = arm_mat_mult_f32(&AT_mat,&A_mat,&ATA_mat);
		if(status != ARM_MATH_SUCCESS)
		{
			cal_ok = 0;
			break;	
		}		
		status = arm_mat_inverse_f32(&ATA_mat,&ATA_inv_mat);
		if(status != ARM_MATH_SUCCESS)
		{
			cal_ok = 0;
			break;	
		}		
		status = arm_mat_mult_f32(&ATA_inv_mat,&AT_mat,&cal_mat);
		if(status != ARM_MATH_SUCCESS)
		{
			cal_ok = 0;
			break;	
		}		
		status = arm_mat_mult_f32(&cal_mat,&B_mat,&X_mat);
		if(status != ARM_MATH_SUCCESS)
		{
			cal_ok = 0;
			break;	
		}
		cal_ok = 1;		
	}
	while(0);
	
	//һ��Ҫ�ͷ��ڴ��������
	Array_free(&A_data);
	Array_free(&AT_data);
	Array_free(&ATA_data);
	Array_free(&ATA_inv_data);
	Array_free(&cal_data);
	Array_free(&B_data);

	if(cal_ok == 1)
	{
		Cal_result[0]=X_mat.pData[0];
		Cal_result[1]=X_mat.pData[1];
		Cal_result[2]=X_mat.pData[2];
		return 1;
	}
	else
	{
		Cal_result[0]=0;
		Cal_result[1]=0;
		Cal_result[2]=0;
		return 0;
	}
}



/*! ------------------------------------------------------------------------------------------------------------------
 * @brief ��ά��̩�������㷨������ԭ�����ٶȣ�
 * ͨ������õ��������ֵ(x0,y0,z0)�����ʵ�ʵ�k����վ����ǩ����Ϊd(k),���ɱ�ǩ���㵽��k����վ����Ϊg(k)= sqrt((x0 - xk)^2 + (y0 - yk)^2 + (z0 - zk)^2)
 * ��ô����d(k) - g(k) = Error(k)
 * ��Ҫ��Error��С�����Թ��Ƶ���ֵ(x',y',z'),��g(k)����һ��̩��չ������
 *  ��ʽΪAX=B => X = (AT * A)^-1 * AT * B
 *     -                                              -       -    -        -             -
 *     | (x0 - x1)/g(1) (y0 - y1)/g(1) (z0 - z1)/g(1) |       | x' |        | d(1) - g(1) |
 * A = |       ...           ...             ...      |   X = | y' |    B = |     ...     |
 *     | (x0 - xk)/g(k) (y0 - yk)/g(k) (z0 - zk)/g(k) |       | z' |        | d(k) - g(k) |
 *     -                                              -       -    -        -             -
 * input parameters
 * @param (*Ancs)[4]  �����������л�վ����+�������� �������� x y z dist
 * @param cal_num  �������Ļ�վ����
 * @param x0 y0 z0   ����������ֵ
 * @param *Cal_result  ����������
 * output parameters 
   1��������ɹ� -1ʧ��
 */
int8_t Cal_Taylor_3D(const float (*Ancs)[4],const uint16_t cal_num, float x0, float y0, float z0, float *Cal_result)
{
	uint16_t i;
	uint8_t cal_ok = 0;
	float a1,a2,a3,dist;
	arm_matrix_instance_f32 A_mat;
	arm_matrix_instance_f32 X_mat;
	arm_matrix_instance_f32 B_mat;
	arm_matrix_instance_f32 AT_mat;
	arm_matrix_instance_f32 ATA_mat;
	arm_matrix_instance_f32 ATA_inv_mat;
	arm_matrix_instance_f32 cal_mat;
	arm_status status = ARM_MATH_SUCCESS;
	
	Array_t A_data = Array_create(cal_num * 3);
	Array_t B_data = Array_create(cal_num);
	Array_t AT_data = Array_create(3 * cal_num);
	Array_t ATA_data = Array_create(3 * 3);
	Array_t ATA_inv_data = Array_create(3 * 3);
	Array_t cal_data = Array_create(3 * cal_num );
	float32_t X_data[3] ={0};
	
	//�����鸳ֵ
	for(i=0;i<cal_num;i++)
	{
		dist = Cal_Dist_3D(x0,y0,z0,Ancs[i][0],Ancs[i][1],Ancs[i][2]);
		a1 = x0 - Ancs[i][0];
		a2 = y0 - Ancs[i][1];
		a3 = z0 - Ancs[i][2];
		
		Array_set(&A_data,i*3,a1 / dist);
		Array_set(&A_data,i*3+1,a2 / dist);
		Array_set(&A_data,i*3+2,a3 / dist);
		Array_set(&B_data,i,Ancs[i][3] - dist);
	}
	memset(AT_data.array,0,AT_data.size);
	memset(ATA_data.array,0,ATA_data.size);
	memset(ATA_inv_data.array,0,ATA_inv_data.size);
	memset(cal_data.array,0,cal_data.size);
	
	//�����ʼ��
	arm_mat_init_f32(&A_mat, cal_num, 3, (float32_t *)A_data.array);
	arm_mat_init_f32(&AT_mat, 3, cal_num, (float32_t *)AT_data.array);
	arm_mat_init_f32(&ATA_mat, 3, 3, (float32_t *)ATA_data.array);
	arm_mat_init_f32(&ATA_inv_mat, 3, 3, (float32_t *)ATA_inv_data.array);
	arm_mat_init_f32(&cal_mat, 3, cal_num, (float32_t *)cal_data.array);
	arm_mat_init_f32(&B_mat, cal_num, 1 ,(float32_t *)B_data.array);
	arm_mat_init_f32(&X_mat, 3, 1 ,(float32_t *)X_data);
	
	do
	{
		status = arm_mat_trans_f32(&A_mat,&AT_mat);
		if(status != ARM_MATH_SUCCESS)	
		{
			cal_ok = 0;
			break;	
		}	
		status = arm_mat_mult_f32(&AT_mat,&A_mat,&ATA_mat);
		if(status != ARM_MATH_SUCCESS)
		{
			cal_ok = 0;
			break;	
		}		
		status = arm_mat_inverse_f32(&ATA_mat,&ATA_inv_mat);
		if(status != ARM_MATH_SUCCESS)
		{
			cal_ok = 0;
			break;	
		}		
		status = arm_mat_mult_f32(&ATA_inv_mat,&AT_mat,&cal_mat);
		if(status != ARM_MATH_SUCCESS)
		{
			cal_ok = 0;
			break;	
		}		
		status = arm_mat_mult_f32(&cal_mat,&B_mat,&X_mat);
		if(status != ARM_MATH_SUCCESS)
		{
			cal_ok = 0;
			break;	
		}
		cal_ok = 1;		
	}
	while(0);
		
	//һ��Ҫ�ͷ��ڴ��������
	Array_free(&A_data);
	Array_free(&AT_data);
	Array_free(&ATA_data);
	Array_free(&ATA_inv_data);
	Array_free(&cal_data);
	Array_free(&B_data);

	if(cal_ok == 1)
	{
		Cal_result[0]=X_mat.pData[0];
		Cal_result[1]=X_mat.pData[1];
		Cal_result[2]=X_mat.pData[2];
		return 1;
	}
	else
	{
		Cal_result[0]=0;
		Cal_result[1]=0;
		Cal_result[2]=0;
		return -1;
	}
	
}




/*! ------------------------------------------------------------------------------------------------------------------
 * @brief ��ά���������߼�
 *                
 * input parameters
 * @param *anc_list �����վ�б�
 * @param *point_out ����ó���ǩ����
 * output parameters 
   1����ɹ� 0����ʧ��
 */				
uint8_t Rtls_Cal_2D(Anchor_t *anc_list, float *point_out)
{
	float point_buf[20][2];  //ÿ������վѭ����λ�õ����������ݻ��棬6ȡ3���������Ϊ20
	float point_temp[2] = {0,0};
	float BS_buf_EN [16][3];  //ʵ��ʹ���Ҳ��ɹ��Ļ�վ������
	uint16_t sort_bs[16][2];
	uint8_t BS_cal_error[6] = {0}; //��¼�ĸ���վû�����ڶ�λ ���û�� ����̩��������Ӧ�ü���û�վ��������
	float x=0,y=0;
	float taylor_result[2] = {0.0f};
	int8_t taylor_time = 5, taylor_ok = -1;
			
	uint8_t BS_EN_num = 0, num = 0;   //ʹ���Ҳ��ɹ��Ļ�վ��������
	int8_t i = 0;
	uint8_t E = 0, R = 0, T = 0;
	    
	//��ֵ������������
	for(i = 0;i < ANCHOR_LIST_COUNT;i++)
	{
		Anchor_t *a = &anc_list[i];
		if((Calculate_FLAG>>i)&0x01)
		{
			sort_bs[BS_EN_num][0] = a->dist;
			sort_bs[BS_EN_num][1] = i;
			BS_EN_num++;
		}					
	}
			
	if (BS_EN_num < 3)    //����3����վ���޷���λ                           
		return 0;
	else   
	{
		if(BS_EN_num > 6)  //����6��,���ݲ������С �ų����� �ų�ֱ��ֻ��6����վ�������
		{
			Quick_Sort_withdata(sort_bs,0,BS_EN_num - 1);  //���ٴ�С��������
			BS_EN_num = 6;  //ֻҪǰ����
			for(i=0;i<BS_EN_num;i++)
			{
				uint16_t anc_idx = sort_bs[i][1];
				BS_buf_EN[i][0] = anc_list[anc_idx].x;
				BS_buf_EN[i][1] = anc_list[anc_idx].y;
				BS_buf_EN[i][2] = anc_list[anc_idx].dist;
			}
		}
		else
		{
			//����6��ֱ�Ӹ�ֵ
			for(i=0;i<BS_EN_num;i++)
			{
				 uint16_t anc_idx = sort_bs[i][1];
				 BS_buf_EN[i][0] = anc_list[anc_idx].x;
				 BS_buf_EN[i][1] = anc_list[anc_idx].y;
				 BS_buf_EN[i][2] = anc_list[anc_idx].dist;
			}
		}					 
	}
             
	for (E = 0; E < (BS_EN_num - 2); E++)  //������ʹ�ܵĻ�վÿ�����������ѭ����λ
	{
		for (R = E + 1; R < (BS_EN_num - 1); R++)
		{
			for (T = R + 1; T < BS_EN_num; T++)
			{
				uint8_t flag=0;		
				//								 ERROR_FLAG=0;                     //�����ʱ����Ҫ����һ�´����־λ���൱��ι��	 									
				flag=Judge_2D(BS_buf_EN[E], BS_buf_EN[R],BS_buf_EN[T]);
				if(flag == 1)
				{
					uint8_t calsuccess = 0;
					calsuccess = Cal_2D_AllCenterMass(BS_buf_EN[E], BS_buf_EN[R], BS_buf_EN[T], point_temp);
					if(calsuccess == 1)  //����ɹ������������
					{
						point_buf[num][0] = point_temp[0];
						point_buf[num][1] = point_temp[1];
						num++;
						//�������ڼ���Ļ�վ�����������̩������
						BS_cal_error[E] = 1;
						BS_cal_error[R] = 1;
						BS_cal_error[T] = 1;
					}

				}
			}
		}
	}
				
	 point_out[0] = 0.0;
	 point_out[1] = 0.0;
				 
	 //����ɸѡ
	 if(num > 1)
	 {
		CenterMass_Select(point_buf,num,point_out);
	 }
	 else if(num == 1)
	 {
		point_out[0] = point_buf[0][0];
		point_out[1] = point_buf[0][1]; 
	 }
	 else
		return 0;
			 
//			 for (i = 0; i < num; i++)  //�����м���õ���������Ӵ���point_out
//			 {
//					 point_out[0] += point_buf[i][0];
//					 point_out[1] += point_buf[i][1];
//			 }
//			 if (num != 0)       //ȡƽ��ֵ�����������
//			 {
//					point_out[0] = point_out[0] / num;
//					point_out[1] = point_out[1] / num;
////				 return 1;
//			 }
			 
	/* ��ó�ֵ */ 		 
	x = point_out[0];
	y = point_out[1];
	 
	/* ȥ�����õĻ�վ �����쳣����Ӱ�����̩������ */
	num = BS_EN_num;
	for(i=BS_EN_num-1;i>=0;i--)
	{
		if(BS_cal_error[i] == 0)  //�жϵ�һ�ν�������ж�û�в��øû�վ����
		{
			for(E = i;E < BS_EN_num - 1;E++)
			{
				BS_buf_EN[E][0] = BS_buf_EN[E+1][0];
				BS_buf_EN[E][1] = BS_buf_EN[E+1][1];
				BS_buf_EN[E][2] = BS_buf_EN[E+1][2];
			}
			num--;
		}
	}
	BS_EN_num = num;

	 
	//Taylor����
	taylor_time = 5;
	do
	{
		if(Cal_Taylor_2D(BS_buf_EN,BS_EN_num,x,y,taylor_result) != -1)
		{
			if(fabs(taylor_result[0]) + fabs(taylor_result[1]) < TAYLOR_2D_THRESH)
			{
//				point_out[0] = x + taylor_result[0];
//				point_out[1] = y + taylor_result[1];
				taylor_ok = 1;
				break;
			}
			else
			{
				x += taylor_result[0];
				y += taylor_result[1];
			}						 
		 }
		 else
		 {
			 break;
		 }
	}
	while(taylor_time-- > 0);

	if(taylor_ok == 1)  //�����ɹ� ���ʧ�� ���������ǰ��ֵ
	{
		point_out[0] = x + taylor_result[0];
		point_out[1] = y + taylor_result[1];
	}
	return 1;
			 
//	if(taylor_time <= 0)
//	{
//		point_out[0] = x + taylor_result[0];
//		point_out[1] = y + taylor_result[1];
//		return 1;
//	}
//	else
//	{
//		//taylor����ʧ�� �����һ�μ��������ֵ
//		return 1;
//	}
			 			
}	


/*! ------------------------------------------------------------------------------------------------------------------
 * @brief ��ά���������߼�
 *                
 * input parameters
 * @param *anc_list �����վ�б�
 * @param *point_out ����ó���ǩ����
 * output parameters 
   1����ɹ� 0����ʧ��
 */				
uint8_t Rtls_Cal_3D(Anchor_t *anc_list ,float *point_out)
{
//			double point_buf[70][3];  //ÿ�ĸ���վѭ����λ�õ����������ݻ��棬8ȡ4���������Ϊ70,�ٶȴ�С��89�������
	float BS_buf_EN [16][4];  //ʵ��ʹ���Ҳ��ɵĻ�վ������
	uint8_t BS_EN_num = 0, BS_EN_temp = 0;   //ʹ���Ҳ��ɹ��Ļ�վ��������
	float point_temp[3] = {0.0f}, taylor_result[3] = {0.0f};
	uint16_t sort_bs[16][2];
	int8_t taylor_time = 0, taylor_ok = -1;
	float x = 0,y = 0,z = 0;
	uint16_t dist_3D_anc_max = 0;		
	int i = 0;
	uint8_t q = 0;
//      int E = 0, R = 0, T = 0, K = 0;
	     
	point_out[0] = 0.0f;
	point_out[1] = 0.0f;
	point_out[2] = 0.0f;
			
	//��ֵ������������
	for(i = 0;i < ANCHOR_LIST_COUNT;i++)
	{
		Anchor_t *a = &anc_list[i];
		if((Calculate_FLAG>>i)&0x01)
		{
			sort_bs[BS_EN_num][0] = a->dist;
			sort_bs[BS_EN_num][1] = i;
			BS_EN_num++;
		}					
	  }
			
			
//	for(i = 0;i < ANCHOR_LIST_COUNT;i++)
//	{
//		Anchor *a = &anc_list[i];
//		if((Calculate_FLAG>>i) & 0x01)
//		{
//			BS_buf_EN[BS_EN_num][0] = a->x;
//			BS_buf_EN[BS_EN_num][1] = a->y;
//			BS_buf_EN[BS_EN_num][2] = a->z;
//			BS_buf_EN[BS_EN_num][3] = a->dist;
//			BS_EN_num++;
//		}					
//	}
				
	if (BS_EN_num < 4)    //����4����վ���޷���λ                          
		return 0;
	else if(BS_EN_num > 10) //�������ά�ȹ��߷��ֻ���㲻�� ����10����վ����ʹ���������� ����������
	{
		Quick_Sort_withdata(sort_bs,0,BS_EN_num - 1);  //���ٴ�С��������
		BS_EN_num = 10;  //ֻҪǰʮ��
		for(i=0;i<BS_EN_num;i++)
		{
			uint16_t anc_idx = sort_bs[i][1];
			BS_buf_EN[i][0] = anc_list[anc_idx].x;
			BS_buf_EN[i][1] = anc_list[anc_idx].y;
			BS_buf_EN[i][2] = anc_list[anc_idx].z;
			BS_buf_EN[i][3] = anc_list[anc_idx].dist;
		}
	}
	else  //С��10��ֱ�Ӹ�ֵ
	{
		for(i=0;i<BS_EN_num;i++)
		{
			uint16_t anc_idx = sort_bs[i][1];
			BS_buf_EN[i][0] = anc_list[anc_idx].x;
			BS_buf_EN[i][1] = anc_list[anc_idx].y;
			BS_buf_EN[i][2] = anc_list[anc_idx].z;
			BS_buf_EN[i][3] = anc_list[anc_idx].dist;
		}
	}
				
	/* ɸѡ�쳣���� */
	dist_3D_anc_max = Cal_Max_3D_Dist(BS_buf_EN,BS_EN_num);
	BS_EN_temp = BS_EN_num;
	for(i=BS_EN_num-1;i>=0;i--)
	{
		if(BS_buf_EN[i][3] > dist_3D_anc_max * 2) //��Ϊ�쳣���� ����������ά����
		{
			for(q = i;q < BS_EN_num - 1; q++)
			{
				BS_buf_EN[q][0] = BS_buf_EN[q+1][0];
				BS_buf_EN[q][1] = BS_buf_EN[q+1][1];
				BS_buf_EN[q][2] = BS_buf_EN[q+1][2];				
			}
			BS_EN_temp--;
		}
	}
	
		
	if(Cal_3D_LeastSquare(BS_buf_EN,BS_EN_num,point_temp) == 1)
	{
		x = point_temp[0];
		y = point_temp[1];
		z = point_temp[2];
	}
	 
	point_out[0] = x;
	point_out[1] = y;
	point_out[2] = z;

	//Taylor����
	taylor_time = 5;
	do
	{
		if(Cal_Taylor_3D(BS_buf_EN,BS_EN_num,x,y,z,taylor_result) != -1)
		{
			if(fabs(taylor_result[0]) + fabs(taylor_result[1] + fabs(taylor_result[2])) < TAYLOR_3D_THRESH)
			{
//				point_out[0] = x + taylor_result[0];
//				point_out[1] = y + taylor_result[1];
//				point_out[2] = z + taylor_result[2];
				taylor_ok = 1;
				break;
			}
			else
			{
				x += taylor_result[0];
				y += taylor_result[1];
				z += taylor_result[2];
			}						 
		}
		else
		{
			break;
		}
	}
	while(taylor_time-- > 0);
			 
	if(taylor_ok == 1)  //�����ɹ� ���ʧ�� ���������ǰ��ֵ
	{
		point_out[0] = x + taylor_result[0];
		point_out[1] = y + taylor_result[1];
		point_out[2] = z + taylor_result[2];
	}
	return 1;
			 
//			 if(taylor_time <= 0)
//			 {
//				 point_out[0] = x + taylor_result[0];
//			   point_out[1] = y + taylor_result[1];
//				 point_out[2] = z + taylor_result[1];
//				 return 1;
//			 }
//			 else
//			 {
//				 //taylor����ʧ�� �����һ�μ��������ֵ
//				 return 1;
//			 }
//				return 0;
}


 
/*! ------------------------------------------------------------------------------------------------------------------
 * @brief ��ά�������� ֱ�Ӿ������ �ɣ�5.0���º���ʹ�ã�
 *  ��ʽΪAX=B => X = A^-1 * B
 *     -                       -      -   -      -                                          -
 *     | 2(x0 - x1) 2(y0 - y1) |      | x |      | x0^2 + y0^2 - d0^2 - x1^2 - y1^2 + d1^2  |
 * A = |                       |  X = |   |  B = |                                          |
 *     | 2(x0 - x2) 2(y0 - y2) |      | y |      | x0^2 + y0^2 - d0^2 - x2^2 - y2^2 + d2^2  |
 *     -                       -      -   -      -                                          -
 * input parameters
 * @param x1 y1  ��վ1��ά���� r1 ��վ1��þ���
 * @param x2 y2  ��վ2��ά���� r2 ��վ2��þ���
 * @param x3 y3  ��վ3��ά���� r3 ��վ3��þ���
 * @param *PP_point_out  ����������
 * output parameters 
   1��������ɹ� 0ʧ��
 */					
uint8_t Get_three_BS_Out_XY(double x1, double y1, double r1,
												 double x2, double y2, double r2,
												 double x3, double y3, double r3,double *PP_point_out)
{
	double A[2][2];
	double B[2][2];
	double C[2];
	double det = 0;    //determinant
	A[0][0] = 2 * (x1 - x2); 
	A[0][1] = 2 * (y1 - y2); 
	A[1][0] = 2 * (x1 - x3);
	A[1][1] = 2 * (y1 - y3); 
	 
	det =A[0][0] * A[1][1] - A[1][0] * A[0][1];

	if (det != 0)
	{
		B[0][0] = A[1][1] / det;
		B[0][1] = -A[0][1] / det;


		B[1][0] = -A[1][0] / det;
		B[1][1] = A[0][0] / det;

		C[0] = r2 * r2 - r1 * r1 - x2 * x2 + x1 * x1 - y2 * y2 + y1 * y1;
		C[1] = r3 * r3 - r1 * r1 - x3 * x3 + x1 * x1 - y3 * y3 + y1 * y1;

		PP_point_out[0] = B[0][0] * C[0] + B[0][1] * C[1] ;
		PP_point_out[1] = B[1][0] * C[0] + B[1][1] * C[1] ;	
		return 1;
	}
	else
	{
		PP_point_out[0] = 0;
		PP_point_out[1] = 0;
		return 0;
	}
				 
}



/*! ------------------------------------------------------------------------------------------------------------------
 * @brief ��ά�������� ֱ�Ӿ������ �ɣ�5.0���º���ʹ�ã�
 *  ��ʽΪAX=B => X = A^-1 * B
 *     -                                  -      -   -      -                                                        -
 *     | 2(x0 - x1) 2(y0 - y1) 2(z0 - z1) |      | x |      | x0^2 + y0^2 + z0^2 - d0^2 - x1^2 - y1^2 - z1^2 + d1^2  |
 * A = | 2(x0 - x2) 2(y0 - y2) 2(z0 - y2) |  X = | y |  B = | x0^2 + y0^2 + z0^2 - d0^2 - x2^2 - y2^2 - z2^2 + d2^2  |
 *     | 2(x0 - x3) 2(y0 - y3) 2(z0 - y3) |      | z |      | x0^2 + y0^2 + z0^2 - d0^2 - x3^2 - y2^2 - z3^2 + d3^2  |
 *     -                                  -      -   -      -                                                        -
 * input parameters
 * @param x1 y1 z1  ��վ1��ά���� r1 ��վ1��þ���
 * @param x2 y2 z2  ��վ2��ά���� r2 ��վ2��þ���
 * @param x3 y3 z3  ��վ3��ά���� r3 ��վ3��þ���
 * @param x4 y4 z4  ��վ4��ά���� r4 ��վ4��þ���
 * @param *PP_point_out  ����������
 * output parameters 
   1��������ɹ� 0ʧ��
 */			
uint8_t Get_three_BS_Out_XYZ(double x1, double y1, double z1, double r1,
                           double x2, double y2, double z2, double r2,
                           double x3, double y3, double z3, double r3,
                           double x4, double y4, double z4, double r4,double *Point_xyz)//��ά�������
{
	double A[3][3];
	double B[3][3];
	double C[3];
	double det = 0;    //����A������ʽ
	//��3*3�Ķ�ά����A�洢����A������
	A[0][0] = 2 * (x1 - x2); A[0][1] = 2 * (y1 - y2); A[0][2] = 2 * (z1 - z2);
	A[1][0] = 2 * (x1 - x3); A[1][1] = 2 * (y1 - y3); A[1][2] = 2 * (z1 - z3);
	A[2][0] = 2 * (x1 - x4); A[2][1] = 2 * (y1 - y4); A[2][2] = 2 * (z1 - z4);  

	//�����A������ʽ��ֵ
	det = A[0][0]*A[1][1]*A[2][2]+A[0][1]*A[1][2]*A[2][0]+A[0][2]*A[1][0]*A[2][1]
	-A[2][0]*A[1][1]*A[0][2]-A[1][0]*A[0][1]*A[2][2]-A[0][0]*A[2][1]*A[1][2];

	if (det != 0)  //ֻ���ھ���A������ʽ��Ϊ0ʱ������A�Ŵ��������3*3�Ķ�ά����B��ΪA�������
	{
		B[0][0] = (A[1][1] * A[2][2] - A[1][2] * A[2][1]) / det;
		B[0][1] = -(A[0][1] * A[2][2] - A[0][2] * A[2][1]) / det;
		B[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) / det;

		B[1][0] = -(A[1][0] * A[2][2] - A[1][2] * A[2][0]) / det;
		B[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) / det;
		B[1][2] = -(A[0][0] * A[1][2] - A[0][2] * A[1][0]) / det;

		B[2][0] = (A[1][0] * A[2][1] - A[1][1] * A[2][0]) / det;
		B[2][1] = -(A[0][0] * A[2][1] - A[0][1] * A[2][0]) / det;
		B[2][2] = (A[0][0] * A[1][1] - A[0][1] * A[1][0]) / det;

		//����CΪ��ʽA*X=C�еľ���C
		C[0] = r2 * r2 - r1 * r1 - x2 * x2 + x1 * x1 - y2 * y2 + y1 * y1 - z2 * z2 + z1 * z1;
		C[1] = r3 * r3 - r1 * r1 - x3 * x3 + x1 * x1 - y3 * y3 + y1 * y1 - z3 * z3 + z1 * z1;
		C[2] = r4 * r4 - r1 * r1 - x4 * x4 + x1 * x1 - y4 * y4 + y1 * y1 - z4 * z4 + z1 * z1;

		//������A���������˾���C�õ���ǩx,y,z��ֵ
		Point_xyz[0] = B[0][0] * C[0] + B[0][1] * C[1] + B[0][2] * C[2];
		Point_xyz[1] = B[1][0] * C[0] + B[1][1] * C[1] + B[1][2] * C[2];
		Point_xyz[2] = B[2][0] * C[0] + B[2][1] * C[1] + B[2][2] * C[2];
		return 1;
	}
	else
	{
		Point_xyz[0] = 0;
		Point_xyz[1] = 0;
		Point_xyz[2] = 0;

	}
	return 0;
}


/*! ------------------------------------------------------------------------------------------------------------------
 * @brief ��ά�������� ��С���˷����� �ɣ�5.0���º���ʹ�ã�
 *  ��ʽΪAX=B => X = (AT * A)^-1 * AT * B
 *     -                                  -      -   -      -                                                        -
 *     | 2(x0 - x1) 2(y0 - y1) 2(z0 - z1) |      | x |      | x0^2 + y0^2 + z0^2 - d0^2 - x1^2 - y1^2 - z1^2 + d1^2  |
 * A = | 2(x0 - x2) 2(y0 - y2) 2(z0 - y2) |  X = | y |  B = | x0^2 + y0^2 + z0^2 - d0^2 - x2^2 - y2^2 - z2^2 + d2^2  |
 *     | 2(x0 - x3) 2(y0 - y3) 2(z0 - y3) |      | z |      | x0^2 + y0^2 + z0^2 - d0^2 - x3^2 - y2^2 - z3^2 + d3^2  |
 *     -                                  -      -   -      -                                                        -
 * input parameters
 * @param x1 y1 z1  ��վ1��ά���� r1 ��վ1��þ���
 * @param x2 y2 z2  ��վ2��ά���� r2 ��վ2��þ���
 * @param x3 y3 z3  ��վ3��ά���� r3 ��վ3��þ���
 * @param x4 y4 z4  ��վ4��ά���� r4 ��վ4��þ���
 * @param *PP_point_out  ����������
 * output parameters 
   1��������ɹ� 0ʧ��
 */							 
uint8_t Get_three_BS_Out_XYZ_New(double x1, double y1, double z1, double r1,
								 double x2, double y2, double z2, double r2,
								 double x3, double y3, double z3, double r3,
								 double x4, double y4, double z4, double r4,float *Point_xyz)//��ά�������
{
	uint8_t i,j;
	double A[3][3];
	double AT[3][3];
	double ATA[3][3];
	double H[3][3];
	double B[3][3];
	double C[3];
	double det = 0;    //����A������ʽ
	//��3*3�Ķ�ά����A�洢����A������
	A[0][0] = 2 * (x1 - x2); A[0][1] = 2 * (y1 - y2); A[0][2] = 2 * (z1 - z2);
	A[1][0] = 2 * (x1 - x3); A[1][1] = 2 * (y1 - y3); A[1][2] = 2 * (z1 - z3);
	A[2][0] = 2 * (x1 - x4); A[2][1] = 2 * (y1 - y4); A[2][2] = 2 * (z1 - z4);  

	//��A��ת�þ���
	for(i=0;i<3;i++)
	{
		for(j=0;j<3;j++)
		{
			AT[i][j] = A[j][i];
		}
	}
 
 //��AT*A
	for(i=0;i<3;i++)
	{
		for(j=0;j<3;j++)
		{
			ATA[i][j] = AT[i][0]*A[0][j]+AT[i][1]*A[1][j]+AT[i][2]*A[2][j];
		}
	}

 
 //�����ATA������ʽ��ֵ
	det = ATA[0][0]*ATA[1][1]*ATA[2][2]+ATA[0][1]*ATA[1][2]*ATA[2][0]+ATA[0][2]*ATA[1][0]*ATA[2][1]
			-ATA[2][0]*ATA[1][1]*ATA[0][2]-ATA[1][0]*ATA[0][1]*ATA[2][2]-ATA[0][0]*ATA[2][1]*ATA[1][2];

	 if (det != 0)  //ֻ���ھ���A������ʽ��Ϊ0ʱ������A�Ŵ��������3*3�Ķ�ά����B��ΪATA�������
	 {
		B[0][0] = (ATA[1][1] * ATA[2][2] - ATA[1][2] * ATA[2][1]) / det;
		B[0][1] = -(ATA[0][1] * ATA[2][2] - ATA[0][2] * ATA[2][1]) / det;
		B[0][2] = (ATA[0][1] * ATA[1][2] - ATA[0][2] * ATA[1][1]) / det;

		B[1][0] = -(ATA[1][0] * ATA[2][2] - ATA[1][2] * ATA[2][0]) / det;
		B[1][1] = (ATA[0][0] * ATA[2][2] - ATA[0][2] * ATA[2][0]) / det;
		B[1][2] = -(ATA[0][0] * ATA[1][2] - ATA[0][2] * ATA[1][0]) / det;

		B[2][0] = (ATA[1][0] * ATA[2][1] - ATA[1][1] * ATA[2][0]) / det;
		B[2][1] = -(ATA[0][0] * ATA[2][1] - ATA[0][1] * ATA[2][0]) / det;
		B[2][2] = (ATA[0][0] * ATA[1][1] - ATA[0][1] * ATA[1][0]) / det;
			 
		 
		//��B*AT
		for(i=0;i<3;i++)
		{
			for(j=0;j<3;j++)
			{
				H[i][j] = B[i][0]*AT[0][j]+B[i][1]*AT[1][j]+B[i][2]*AT[2][j];
			}
		}
		//����CΪ��ʽH*X=C�еľ���C
		C[0] = r2 * r2 - r1 * r1 - x2 * x2 + x1 * x1 - y2 * y2 + y1 * y1 - z2 * z2 + z1 * z1;
		C[1] = r3 * r3 - r1 * r1 - x3 * x3 + x1 * x1 - y3 * y3 + y1 * y1 - z3 * z3 + z1 * z1;
		C[2] = r4 * r4 - r1 * r1 - x4 * x4 + x1 * x1 - y4 * y4 + y1 * y1 - z4 * z4 + z1 * z1;

		//������A���������˾���C�õ���ǩx,y,z��ֵ
		Point_xyz[0] = H[0][0] * C[0] + H[0][1] * C[1] + H[0][2] * C[2];
		Point_xyz[1] = H[1][0] * C[0] + H[1][1] * C[1] + H[1][2] * C[2];
		Point_xyz[2] = H[2][0] * C[0] + H[2][1] * C[1] + H[2][2] * C[2];
		return 1;
	 }
	 else
	 {
		Point_xyz[0] = 0;
		Point_xyz[1] = 0;
		Point_xyz[2] = 0;
	 }
	 return 0;
}
				 
				 
				
