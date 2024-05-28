#ifndef LINEAR_THROTTLE_H
#define LINEAR_THROTTLE_H

#include "struct_typedef.h"


#pragma pack(1) // ѹ���ṹ��,ȡ���ֽڶ���

typedef struct
{
	fp32 abs_step;        // �������� ÿ��ϣ��������ֵ - ���Ų���
	fp32 abs_init;				// �޷���Сֵ, ���ͼ��ٳ��� - ��ʼ����ֵ
  fp32 abs_target;      // �޷����ֵ, �����ٶ� - ȫ�����ٶ�
	
  fp32 frame_period; //ʱ����
	
	fp32 out;
	
} linear_throttle_t;

//б��������ʼ��
extern void ramp_init(ramp_function_source_t *ramp_source_type, fp32 frame_period, fp32 max, fp32 min);

//б����������
extern void ramp_calc(ramp_function_source_t *ramp_source_type, fp32 input);

#endif
