
#include "robot_cmd.h"

#pragma pack(1)

typedef struct
{
	uint32_t chassis_flag : 1;
	uint32_t gimbal_flag : 1;
	uint32_t shoot_flag : 1;
	uint32_t lid_flag : 1;
	uint32_t friction_flag : 1;
	uint32_t Power_flag : 1;
} Referee_Interactive_Flag_t;

// �˽ṹ�����UI����������˳���ͨ�ŵ���Ҫ�������ǲ���ϵͳ����
typedef struct
{
	Referee_Interactive_Flag_t Referee_Interactive_Flag;
	// ΪUI�����Լ�������������
	chassis_mode_e chassis_mode;			 // ����ģʽ
	
	user_fire_ctrl_e gun_mode;				 // ����ģʽ����

	// ������������, ģʽ
	Chassis_Power_Data_s Chassis_Power_Data; // ���ʿ���

	// ��һ�ε�ģʽ������flag�ж�
	chassis_mode_e chassis_last_mode;
	gimbal_mode_e gimbal_last_mode;
	shoot_mode_e shoot_last_mode;
	friction_mode_e friction_last_mode;
	lid_mode_e lid_last_mode;
	Chassis_Power_Data_s Chassis_last_Power_Data;

} Referee_Interactive_info_t;

#pragma pack()
