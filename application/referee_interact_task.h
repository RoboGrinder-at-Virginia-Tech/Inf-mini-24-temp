#ifndef __REFEREE_INTERACT_TASK__
#define __REFEREE_INTERACT_TASK__

#include "stm32f4xx.h"
#include "user_lib.h"
#include "struct_typedef.h"
#include "gimbal_task.h"
#include "chassis_task.h"
#include "detect_task.h"
#include "shoot.h"

/*
���������, UI��Referee_Interactive_infoģ����ʹ��Ȩ�Ϳ���Ȩ������ cmd task, UI����ָ����cmd task������Referee_Interactive_info_t,
������֤�˶Ա仯��Ӧ��ʵʱ. ����cmd task�л����(����)����chassis_flag��ֱ��ˢ�¸�UI, �򷢲�chassis_mode, Ȼ����UIģ�����ж�Mode_Change_Check
*/

#pragma pack(1)

typedef struct
{
	uint32_t chassis_mode_flag : 1;
	uint32_t chassis_energy_mode_flag : 1;
	uint32_t shoot_mode_flag : 1;
	uint32_t autoAim_mode_flag : 1;
	uint32_t cv_gimbal_sts_flag : 1;
} Referee_Interactive_Flag_t;

// �˽ṹ�����UI����������˳���ͨ�ŵ���Ҫ�������ǲ���ϵͳ����
typedef struct
{
	Referee_Interactive_Flag_t Referee_Interactive_Flag;
	// ΪUI�����Լ�������������
	chassis_mode_e chassis_mode;			 // ����ģʽ
	
	user_fire_ctrl_e user_fire_ctrl_mode;				 // ����ģʽ����

	// ��һ�ε�ģʽ������flag�ж� (2024 RMULû��)
	chassis_mode_e chassis_last_mode;
	user_fire_ctrl_e shoot_last_mode;
	user_fire_ctrl_e friction_last_mode;
} Referee_Interactive_info_t;

#pragma pack()

#endif
