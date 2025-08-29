#include "array.h"
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief ��������
 *
 * input parameters
 * @param init_size    ���鳤�� 
 * output parameters
 * ���ش���������
 */
Array_t Array_create(uint16_t init_size)
{
	Array_t a;
	a.array = (float*)malloc(sizeof(float) * init_size);
	a.size = init_size;
	return a;
}

/*! ------------------------------------------------------------------------------------------------------------------
* @brief ��������
 *
 * input parameters
 * @param a    ����ָ��
 * output parameters
 * none
 */
void Array_free(Array_t* a)
{
	free(a->array);
	a->array = NULL;
	a->size = 0;
}

/*! ------------------------------------------------------------------------------------------------------------------
* @brief ��ȡ�����Ӧ������Ԫ��ֵ
 *
 * input parameters
 * @param a    ����ָ��
 * @param idx  ��������
 * output parameters
 * ��������Ӧֵ
 */
float* Array_get(Array_t *a, uint16_t idx)
{
	return &(a->array[idx]);
}

/*! ------------------------------------------------------------------------------------------------------------------
* @brief ���������Ӧ������Ԫ��ֵ
 *
 * input parameters
 * @param a    ����ָ��
 * @param idx  ��������
 * output parameters
 * none
 */
void Array_set(Array_t *a, uint16_t idx, float value)
{
	(a->array)[idx] = value;
}

/*! ------------------------------------------------------------------------------------------------------------------
* @brief ��ȡ���鳤��
 *
 * input parameters
 * @param a    ����ָ��
 * output parameters
 * �����鳤��
 */
uint16_t Array_size(const Array_t *a)
{
	return a->size;
}


