/*************************************************************

RM�Զ���UIЭ��

���������� Virginia Tech; RoboGrinder

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
#include <string.h>

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t client_ui_task_high_water;
#endif
