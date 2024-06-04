/*************************************************************

此任务负责向裁判系统发送信息

弗吉尼亚理工 Virginia Tech; RoboGrinder

**************************************************************/

#include "referee_interact_task.h"
#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>
#include "SuperCap_comm.h"
#include "shoot.h"
#include "referee.h"
#include "referee_usart_task.h"
#include "detect_task.h"
#include "chassis_task.h"
#include "chassis_behaviour.h"
#include "miniPC_msg.h"
#include "user_lib.h"
#include "remote_control.h"
#include <string.h>
#include "referee_ui.h"

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t referee_interact_task_high_water;
#endif

// 暂时在这里存 interactive info
Referee_Interactive_info_t Referee_Interactive_info;

// 更新每个UI的刷新请求flag
void set_interactive_flag_chassis_mode_flag(uint8_t set_val)
{
	Referee_Interactive_info.Referee_Interactive_Flag.chassis_mode_flag = set_val;
}
void set_interactive_flag_chassis_energy_mode_flag(uint8_t set_val)
{
	Referee_Interactive_info.Referee_Interactive_Flag.chassis_energy_mode_flag = set_val;
}
void set_interactive_flag_shoot_mode_flag(uint8_t set_val)
{
	Referee_Interactive_info.Referee_Interactive_Flag.shoot_mode_flag = set_val;
}
void set_interactive_flag_auto_aim_mode_flag(uint8_t set_val)
{
	Referee_Interactive_info.Referee_Interactive_Flag.auto_aim_mode_flag = set_val;
}
void set_interactive_flag_cv_gimbal_sts_flag(uint8_t set_val)
{
	Referee_Interactive_info.Referee_Interactive_Flag.cv_gimbal_sts_flag = set_val;
}

// 更新UI所见的状态机
void set_interactive_info_chassis_mode(chassis_mode_e chassis_mode)
{
	Referee_Interactive_info.chassis_mode = chassis_mode;
}
void set_interactive_info_chassis_energy_mode(chassis_energy_mode_e chassis_energy_mode)
{
	Referee_Interactive_info.chassis_energy_mode = chassis_energy_mode;
}
void set_interactive_info_user_fire_ctrl_mode(user_fire_ctrl_e user_fire_ctrl_mode)
{
	Referee_Interactive_info.user_fire_ctrl_mode = user_fire_ctrl_mode;
}
void set_interactive_info_auto_aim_mode(auto_aim_mode_e auto_aim_mode)
{
	Referee_Interactive_info.auto_aim_mode = auto_aim_mode;
}
void set_interactive_info_cv_gimbal_sts(auto_aim_mode_e cv_gimbal_sts)
{
	Referee_Interactive_info.cv_gimbal_sts = cv_gimbal_sts;
}


//uint32_t temp_time_check_RTOS = 0;
//uint32_t temp_time_check_HAL = 0;

void referee_interact_task(void const *pvParameters)
{
	//等待referee_usart_task中完成对MCU UART6的初始化
	vTaskDelay(200);
	
	Referee_Interactive_info.rc_ctrl_ptr = get_remote_control_point();

	while (1)
  {
		//temp_time_check_RTOS = xTaskGetTickCount();
		//temp_time_check_HAL = HAL_GetTick();
		
		switch (Referee_Interactive_info.rc_ctrl_ptr[TEMP].key[KEY_PRESS].r)
		{
			case 0:
				// does nothings
			break;

			// 按下
			case 1:
				static_UI_func();
			break;

			default:
				// does nothings
			break;
		}
		
		vTaskDelay(100); //100

#if INCLUDE_uxTaskGetStackHighWaterMark
        referee_interact_task_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif	
		}
}
