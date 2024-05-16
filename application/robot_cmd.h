#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>
#include "SuperCap_comm.h"
#include "shoot.h"
#include "referee.h"
#include "detect_task.h"
#include "chassis_task.h"
#include "miniPC_msg.h"
#include "user_lib.h"

extern supercap_can_msg_id_e current_superCap;
extern wulieCap_info_t wulie_Cap_info;
extern sCap23_info_t sCap23_info; //新的超级电容控制板

