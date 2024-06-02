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
理想情况下, UI和Referee_Interactive_info模块在使用权和控制权上属于 cmd task, UI中由指向由cmd task操作的Referee_Interactive_info_t,
这样保证了对变化响应的实时. 比如cmd task中会控制(发布)比如chassis_flag来直接刷新该UI, 或发布chassis_mode, 然后由UI模块来判断Mode_Change_Check
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

// 此结构体包含UI绘制与机器人车间通信的需要的其他非裁判系统数据
typedef struct
{
	Referee_Interactive_Flag_t Referee_Interactive_Flag;
	// 为UI绘制以及交互数据所用
	chassis_mode_e chassis_mode;			 // 底盘模式
	
	user_fire_ctrl_e user_fire_ctrl_mode;				 // 发射模式设置

	// 上一次的模式，用于flag判断 (2024 RMUL没用)
	chassis_mode_e chassis_last_mode;
	user_fire_ctrl_e shoot_last_mode;
	user_fire_ctrl_e friction_last_mode;
} Referee_Interactive_info_t;

#pragma pack()

#endif
