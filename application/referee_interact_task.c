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
#include <string.h>

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t client_ui_task_high_water;
#endif

//extern uint8_t turboMode;
//extern uint8_t swing_flag;

//extern shoot_control_t shoot_control; 
//extern miniPC_info_t miniPC_info;
extern supercap_can_msg_id_e current_superCap;
extern wulieCap_info_t wulieCap_info;
extern gen2Cap_info_t gen2Cap_info; //新的超级电容控制板

//uint32_t temp_time_check_RTOS = 0;
//uint32_t temp_time_check_HAL = 0;



uint32_t client_ui_count_ref = 0;
uint8_t client_ui_test_flag = 1;





static void ui_error_flag_clear()
{
	ui_info.chassis_error_flag = devOK; //初始化为删除
	ui_info.gimbal_error_flag = devOK; //初始化为删除
	ui_info.shoot_error_flag = devOK; //初始化为删除
	ui_info.superCap_error_flag = devOK; //初始化为删除
}

static void ui_error_code_str_clear()
{	
	strcpy(ui_info.chassis_error_code, "\0");
	strcpy(ui_info.gimbal_error_code, "\0");
	strcpy(ui_info.shoot_error_code, "\0");
	strcpy(ui_info.superCap_error_code, "\0");
	strcpy(ui_info.referee_error_code, "\0");	
}

//机器人各个设备的error code
static void ui_error_code_update()
{
	ui_error_code_str_clear();
	
	//check chassis 不显示数字
	ui_info.chassis_error_flag = devOK;
	if(toe_is_error(CHASSIS_MOTOR1_TOE))
	{
		strcat(ui_info.chassis_error_code, "1\0");
		ui_info.chassis_error_flag = devError;
	}
	if(toe_is_error(CHASSIS_MOTOR2_TOE))
	{
		strcat(ui_info.chassis_error_code, "2\0");
		ui_info.chassis_error_flag = devError;
	}
	if(toe_is_error(CHASSIS_MOTOR3_TOE))
	{
		strcat(ui_info.chassis_error_code, "3\0");
		ui_info.chassis_error_flag = devError;
	}
	if(toe_is_error(CHASSIS_MOTOR4_TOE))
	{
		strcat(ui_info.chassis_error_code, "4\0");
		ui_info.chassis_error_flag = devError;
	}
	if(ui_info.chassis_error_flag == devOK)
	{
		strcpy(ui_info.chassis_error_code, "OK  \0");
	}
	else
	{
		strcpy(ui_info.chassis_error_code, "ERR!\0");
	}
	
	//check gimbal 要单独显示
	ui_info.gimbal_error_flag = devOK;
	if(toe_is_error(YAW_GIMBAL_MOTOR_TOE))
	{
		strcpy(ui_info.gimbal_error_code, "Y     \0"); //strcat
		ui_info.gimbal_error_flag = devError;
	}
	if(toe_is_error(PITCH_GIMBAL_MOTOR_L_TOE) || toe_is_error(PITCH_GIMBAL_MOTOR_R_TOE)) // 当任意一个出问题时
	{
		strcpy(ui_info.gimbal_error_code, "P     \0");
		if(toe_is_error(YAW_GIMBAL_MOTOR_TOE))
		{strcpy(ui_info.gimbal_error_code, "YP-ERR\0");}
		
		ui_info.gimbal_error_flag = devError;
	}
	if(ui_info.gimbal_error_flag == devOK)
	{
		strcpy(ui_info.gimbal_error_code, "OK    \0");
	}
	
	//check feed or shoot 不详细显示
	ui_info.shoot_error_flag = devOK;
//	if(toe_is_error(SHOOT_FRIC_L_TOE))
//	{ //MD步兵没有此设备
//		strcat(ui_info.shoot_error_code, "L\0");
//		ui_info.shoot_error_flag = devError;
//	}
//	if(toe_is_error(SHOOT_FRIC_R_TOE))
//	{ //MD步兵没有此设备
//		strcat(ui_info.shoot_error_code, "R\0");
//		ui_info.shoot_error_flag = devError;
//	}
	if(toe_is_error(TRIGGER_MOTOR17mm_L_TOE))// || toe_is_error(TRIGGER_MOTOR17mm_R_TOE)) // 当任意一个出问题时
	{
		strcat(ui_info.shoot_error_code, "T\0");
		ui_info.shoot_error_flag = devError;
	}
	if(ui_info.shoot_error_flag == devOK)
	{
		strcat(ui_info.shoot_error_code, "OK  \0");
	}
	else
	{
		strcpy(ui_info.shoot_error_code, "ERR!\0");
	}
	
	//check for superCap
	if(current_superCap_is_offline())
	{
		strcpy(ui_info.superCap_error_code, "ERR!\0"); //strcat
		ui_info.superCap_error_flag = devError;
	}
	else
	{
		strcpy(ui_info.superCap_error_code, "OK  \0");
		ui_info.superCap_error_flag = devOK;
	}
	
	//check for referee
	if(toe_is_error(REFEREE_TOE))
	{
		strcpy(ui_info.referee_error_code, "ERR!\0"); //strcat
//		Char_Draw(&strRef, "978", UI_Graph_ADD, 5, UI_Color_Purplish_red, Robot_Warning_Msg_Font_Size, strlen("REF!"), 3, Robot_Warning_REF_X, Robot_Warning_REF_Y, "REF!");
		ui_info.referee_error_flag = devError;
	}
	else
	{
		strcpy(ui_info.referee_error_code, "OK  \0");
//		Char_Draw(&strRef, "978", UI_Graph_Del, 5, UI_Color_Purplish_red, Robot_Warning_Msg_Font_Size, strlen("REF!"), 3, Robot_Warning_REF_X, Robot_Warning_REF_Y, "REF!");
		ui_info.referee_error_flag = devOK;
	}
	
	
}

void client_ui_task(void const *pvParameters)
{
		//等待referee_usart_task中完成对MCU UART6的初始化
		vTaskDelay(200);
		ui_info.error_list_UI_local = get_error_list_point(); //list pointer init

//		memset(&G1,0,sizeof(G1));
//		memset(&G2,0,sizeof(G2));
//		memset(&G3,0,sizeof(G3));
//		memset(&G4,0,sizeof(G4));
//		memset(&G5,0,sizeof(G5));

		//初速绘制"创建"一次动态UI
		//动态图层 占用图层 4,5,6,7
	
		ui_error_code_str_clear(); 
	  ui_error_flag_clear();
	
		/*右上角 各种设备的error code*/
//		// 5-26-2023 不显示那么详细
//		Char_Draw(&strVarChassis, "985", ui_info.chassis_error_flag, 5, UI_Color_Purplish_red, 20, strlen(ui_info.chassis_error_code), 3, CHASSIS_ERROR_CODE_X+ERROR_CODE_STR_X_OFFSET_FROM_FIX_STR, CHASSIS_ERROR_CODE_Y, ui_info.chassis_error_code);
//		Char_Draw(&strVarGimbal, "984", ui_info.gimbal_error_flag, 5, UI_Color_Purplish_red, 20, strlen(ui_info.gimbal_error_code), 3, GIMBAL_ERROR_CODE_X+ERROR_CODE_STR_X_OFFSET_FROM_FIX_STR, GIMBAL_ERROR_CODE_Y, ui_info.gimbal_error_code);
//		Char_Draw(&strVarShoot, "983", ui_info.shoot_error_flag, 5, UI_Color_Purplish_red, 20, strlen(ui_info.shoot_error_code), 3, SHOOT_ERROR_CODE_X+ERROR_CODE_STR_X_OFFSET_FROM_FIX_STR, SHOOT_ERROR_CODE_Y, ui_info.shoot_error_code);
//		Char_Draw(&strVarSuperCap, "982", ui_info.superCap_error_flag, 5, UI_Color_Purplish_red, 20, strlen(ui_info.superCap_error_code), 3, SUPERCAP_ERROR_CODE_X+ERROR_CODE_STR_X_OFFSET_FROM_FIX_STR, SUPERCAP_ERROR_CODE_Y, ui_info.superCap_error_code);
//		Char_Draw(&strVarReferee, "981", ui_info.referee_error_flag, 5, UI_Color_Purplish_red, 20, strlen(ui_info.referee_error_code), 3, REFEREE_ERROR_CODE_X+ERROR_CODE_STR_X_OFFSET_FROM_FIX_STR, REFEREE_ERROR_CODE_Y, ui_info.referee_error_code);
		
		//中间靠上 各种字符 Warning 需要时才显示所以不需要只有 add 和 delete 没有change
		Char_Draw(&strSpin, "980", UI_Graph_ADD, 5, UI_Color_Purplish_red, Robot_Warning_Msg_Font_Size, strlen("SPIN!"), 3, Robot_Warning_Spin_X, Robot_Warning_Spin_Y, "SPIN!");
		Char_Draw(&strFric, "979", UI_Graph_ADD, 5, UI_Color_Purplish_red, Robot_Warning_Msg_Font_Size, strlen("FRIC!"), 3, Robot_Warning_Fric_X, Robot_Warning_Fric_Y, "FRIC!");
//		Char_Draw(&strRef, "978", UI_Graph_ADD, 5, UI_Color_Purplish_red, Robot_Warning_Msg_Font_Size, strlen("REF!"), 3, Robot_Warning_REF_X, Robot_Warning_REF_Y, "REF!");
		
		//中间 NOT右上角 超级电容电压
		//离线时superCap_info.VBKelvin_fromCap修改为 0
		//Float_Draw(&fCapVolt, "999", UI_Graph_ADD, 4, UI_Color_Yellow, 20, 2, 3, 1620, 810, 00.00);//superCap_info.VBKelvin_fromCap);
	//new position of voltage of sup_cap
		Float_Draw(&fCapVolt, "999", UI_Graph_ADD, 4, UI_Color_Yellow, Center_Bottom_SuperCap_VOLT_Font_Size, 2, 3, Center_Bottom_SuperCap_VOLT_NUM_X_COORD, Center_Bottom_SuperCap_VOLT_NUM_Y_COORD, 00.00);//superCap_info.VBKelvin_fromCap);
		//中间 NOT右上角 超级电容百分比
		//离线时superCap_info.EBPct_fromCap修改为 0
		//Float_Draw(&fCapPct, "998", UI_Graph_ADD, 4, UI_Color_Yellow, 20, 2, 3, 1620, 840, 00.00);//superCap_info.EBPct_fromCap);
		//new display position with number size = 40
		Float_Draw(&fCapPct, "998", UI_Graph_ADD, 4, UI_Color_Yellow, Center_Bottom_SuperCap_PCT_Font_Size, 2, 3, Center_Bottom_SuperCap_PCT_NUM_X_COORD, Center_Bottom_SuperCap_PCT_NUM_Y_COORD, 00.00);//superCap_info.EBPct_fromCap);
		//new position of rectangle 右上角方框
		Rectangle_Draw(&gChassisSts_box, "997", UI_Graph_ADD, 4, UI_Color_Cyan, 3, TopRight_REC_on_NORM_START_X-80, TopRight_REC_on_NORM_START_Y-95, TopRight_REC_on_NORM_END_X-80, TopRight_REC_on_NORM_END_Y-95);
		Rectangle_Draw(&gSPINSts_box, "996", UI_Graph_ADD, 4, UI_Color_Cyan, 3, TopRight_REC_on_FOLL_START_X-80, TopRight_REC_on_FOLL_START_Y-95, TopRight_REC_on_FOLL_END_X-80, TopRight_REC_on_FOLL_END_Y-95);
		
		//左上角---还未改
		Rectangle_Draw(&gCVSts_box, "995", UI_Graph_ADD, 4, UI_Color_Cyan, 3, 240, 850, 330, 815);
		Rectangle_Draw(&gGunSts_box, "994", UI_Graph_ADD, 4, UI_Color_Cyan, 3, 240, 810, 330, 775);
//		Rectangle_Draw(&gABoxSts_box, "993", UI_Graph_ADD, 4, UI_Color_Cyan, 3, 240, 770, 330, 735);
		
		//左上角 CV反馈状态
		Rectangle_Draw(&gCVfb_sts_box, "989", UI_Graph_ADD, 4, UI_Color_White, 3, TopLeft_CV_FEEDBACK_STATUS_on_OFF_START_X, TopLeft_CV_FEEDBACK_STATUS_on_OFF_START_Y, TopLeft_CV_FEEDBACK_STATUS_on_OFF_END_X, TopLeft_CV_FEEDBACK_STATUS_on_OFF_END_Y);
		
		//CV自瞄 是否瞄准到目标 圈
		Circle_Draw(&gEnemyDetected_circle, "990", UI_Graph_ADD, 4, UI_Color_Cyan, ui_cv_circle_size_debug, TopLeft_Cir_on_cv_DET_START_X, TopLeft_Cir_on_cv_DET_START_Y, TopLeft_Cir_on_cv_DET_radius);
		
    Float_Draw(&fProjSLim, "992", UI_Graph_ADD, 4, UI_Color_Main, 20, 2, 3, 240, 720, 15.00);
		Float_Draw(&fDis, "991", UI_Graph_ADD, 4, UI_Color_Main, 20, 2, 3, 240, 680, 1.00);
		
		//5-18-23 超级电容 移动能量条 初始化为满能量状态
		Line_Draw(&superCapLine, "988", UI_Graph_ADD, 4, UI_Color_Main, Center_Bottom_SuperCap_Line_Width, Center_Bottom_SuperCap_Line_Start_X, Center_Bottom_SuperCap_Line_Start_Y, Center_Bottom_SuperCap_Line_Start_X, Center_Bottom_SuperCap_Line_Start_Y);
		
		chassis_frame_UI_sensor_and_graph_init();
		chassis_frame_UI_sensor_update();
		chassis_frame_UI_arm_init(ui_info.yaw_relative_angle);
		
		//初始创建 底盘角度指示框
		Line_Draw(&chassisLine, "987", UI_Graph_ADD, 0, UI_Color_Main, Chassis_Frame_Height_Pen, ui_info.frame_chassis_coord_final[0], ui_info.frame_chassis_coord_final[1], ui_info.frame_chassis_coord_final[2], ui_info.frame_chassis_coord_final[3]);
		Line_Draw(&chassisLightBar, "986", UI_Graph_ADD, 7, UI_Color_Yellow, Chassis_Frame_Light_Bar_Height_Pen, ui_info.bar_chassis_coord_final[0], ui_info.bar_chassis_coord_final[1], ui_info.bar_chassis_coord_final[2], ui_info.bar_chassis_coord_final[3]);
		//炮塔 球 和 枪 线 捆绑动态图像
		Circle_Draw(&turretCir, "026", UI_Graph_ADD, 8, UI_Color_White, Turret_Cir_Pen, Turret_Cir_Start_X, Turret_Cir_Start_Y, Turret_Cir_Radius);
		Line_Draw(&gunLine, "027", UI_Graph_ADD, 8, UI_Color_Black, Gun_Line_Pen, Gun_Line_Start_X, Gun_Line_Start_Y, Gun_Line_End_X, Gun_Line_End_Y);
				
		UI_ReFresh(5, chassisLine, turretCir, gunLine, fCapVolt, chassisLightBar); //chassisLine, turretCir, gunLine需捆绑发送
		UI_ReFresh(2, fCapPct, superCapLine);
//		UI_ReFresh(1, superCapLine);
//		UI_ReFresh(2, superCapLine, chassisLine);
//  	UI_ReFresh(2, fCapVolt, fCapPct);
//		UI_ReFresh(1, chassisLightBar);
//		UI_ReFresh(5, chassisLightBar, superCapLine, chassisLine, fCapVolt, fCapPct);
//		UI_ReFresh(2, fProjSLim, fDis);
		UI_ReFresh(1, fDis); // 7-4去掉弹舱
//		UI_ReFresh(2, gEnemyDetected_circle, gCVfb_sts_box);
//		UI_ReFresh(5, gChassisSts_box, gSPINSts_box, gCVSts_box, gGunSts_box, gABoxSts_box);
		UI_ReFresh(7, gEnemyDetected_circle, gCVfb_sts_box, gChassisSts_box, gSPINSts_box, gCVSts_box, gGunSts_box, fProjSLim); // 7-4去掉弹舱
//		Char_ReFresh(strVarChassis); // 5-26-2023 不显示那么详细
//		Char_ReFresh(strVarGimbal); 
//		Char_ReFresh(strVarShoot); 
//		Char_ReFresh(strVarSuperCap); 
//		Char_ReFresh(strVarReferee);
		
		Char_ReFresh(strSpin);
		Char_ReFresh(strFric);
//		Char_ReFresh(strRef);
	  //UI 初始创建 + 发送结束
		
		//底盘 对位线计算 初始化 左
		ui_info.chassis_drive_pos_line_left_slope_var = Chassis_Drive_Pos_Line_Left_Slope;
		ui_info.chassis_drive_pos_line_left_var_startX = Chassis_Drive_Pos_Line_Left_Start_X;
		ui_info.chassis_drive_pos_line_left_var_startY = Chassis_Drive_Pos_Line_Left_Start_Y;
		ui_info.chassis_drive_pos_line_left_var_endX = Chassis_Drive_Pos_Line_Left_End_X;
		
		//底盘 对位线计算 初始化 右
		ui_info.chassis_drive_pos_line_right_slope_var = Chassis_Drive_Pos_Line_Right_Slope;
		ui_info.chassis_drive_pos_line_right_var_startX = Chassis_Drive_Pos_Line_Right_Start_X;
		ui_info.chassis_drive_pos_line_right_var_startY = Chassis_Drive_Pos_Line_Right_Start_Y;
		ui_info.chassis_drive_pos_line_right_var_endX = Chassis_Drive_Pos_Line_Right_End_X;
		
	
		/*	大装甲板宽 230mm 
		*/
		while (1)
    {
				//
//			 temp_time_check_RTOS = xTaskGetTickCount();
//			 temp_time_check_HAL = HAL_GetTick();
//			 //int _temp_a = 0;
//			 _temp_a++;
			
				//先画静态图像; 占用图层 0, 1, 2, 3
				//右上角
				ui_error_code_str_clear(); 
				/*右上角 各种设备的error code*/
			  // 5-26-2023 不显示那么详细 错误时才显示
				Char_Draw(&strChassis, "030", ui_info.chassis_error_flag, 2, UI_Color_Yellow, 20, strlen("CH:"), 3, CHASSIS_ERROR_CODE_X, CHASSIS_ERROR_CODE_Y, "CH:");
				Char_Draw(&strGimbal, "031", ui_info.gimbal_error_flag, 2, UI_Color_Yellow, 20, strlen("GY:"), 3, GIMBAL_ERROR_CODE_X, GIMBAL_ERROR_CODE_Y, "GY:");
				Char_Draw(&strShoot, "032", ui_info.shoot_error_flag, 2, UI_Color_Yellow, 20, strlen("FD:"), 3, SHOOT_ERROR_CODE_X, SHOOT_ERROR_CODE_Y, "FD:");
				Char_Draw(&strSuperCap, "033", ui_info.superCap_error_flag, 2, UI_Color_Yellow, 20, strlen("SC:"), 3, SUPERCAP_ERROR_CODE_X, SUPERCAP_ERROR_CODE_Y, "SC:");
//				Char_Draw(&strReferee, "034", ui_info.referee_error_flag, 2, UI_Color_Yellow, 20, strlen("RF:"), 3, REFEREE_ERROR_CODE_X, REFEREE_ERROR_CODE_Y, "RF:");
				
				//Cap Pct:字样 封装字号20 和 图形线宽 3
				//Char_Draw(&strCapPct, "008", UI_Graph_ADD, 2, UI_Color_Yellow, 20, 17, 3, CAP_PCT_X, CAP_PCT_Y,   "Cap Pct:        %");
			  //Cap Volt:字样 封装字号20 和 图形线宽 3
				//Char_Draw(&strCapVolt, "009", UI_Graph_ADD, 2, UI_Color_Yellow, 20, 9, 3, CAP_VOLT_X, CAP_VOLT_Y, "Cap Volt:");  

				//底盘状态		
				//Char_Draw(&strChassisSts, "010", UI_Graph_ADD, 2, UI_Color_Yellow, 20, 10, 3, CHASSIS_STS_X, CHASSIS_STS_Y, "NORM BOOST");
				//Char_Draw(&strSPINSts, "011", UI_Graph_ADD, 2, UI_Color_Yellow, 20, 9, 3, SPIN_STS_X, SPIN_STS_Y,           "FOLL SPIN");
				//New position 底盘状态
				Char_Draw(&strChassisSts, "010", UI_Graph_ADD, 2, UI_Color_Yellow, 20, 10, 3, CHASSIS_STS_X, CHASSIS_STS_Y, "NORM BOOST");
				Char_Draw(&strSPINSts, "011", UI_Graph_ADD, 2, UI_Color_Yellow, 20, 9, 3, SPIN_STS_X, SPIN_STS_Y,           "FOLL SPIN");
			
				//中间
				//Aim 竖线
				Line_Draw(&gAimVertL, "001", UI_Graph_ADD, 2, UI_Color_Orange, 1, 960-9+5, 540+45, 960-9+5, 0);
					
				//2m
				//Line_Draw(&gAimHorizL2m, "003", UI_Graph_ADD, 2, UI_Color_Yellow, 1, (960-50), (540-24), (960+50), (540-24));//big armor plate
				Line_Draw(&gAimHorizL2m, "003", UI_Graph_ADD, 2, UI_Color_Yellow, 1, (960-30-9), (540-24), (960+30-9), (540-24)); // small armor plate
				//4m
				//Line_Draw(&gAimHorizL4m, "004", UI_Graph_ADD, 2, UI_Color_Yellow, 1, (960-25), (540-45), (960+25), (540-45));
				Line_Draw(&gAimHorizL4m, "004", UI_Graph_ADD, 2, UI_Color_Yellow, 1, (960-15-9), (540-45), (960+15-9), (540-45));		
				//5 (540-150) "010"
			//	Line_Draw(&gAimHorizL5m, "005", UI_Graph_ADD, 2, UI_Color_Yellow, 1, (960-21), (540-66), (960+21), (540-66));
				Line_Draw(&gAimHorizL5m, "005", UI_Graph_ADD, 2, UI_Color_Yellow, 1, (960-12-9), (540-66), (960+12-9), (540-66));
				//7 (540-170) "011"
				//Line_Draw(&gAimHorizL7m, "006", UI_Graph_ADD, 2, UI_Color_Yellow, 1, (960-15), (540-95), (960+15), (540-95));
				Line_Draw(&gAimHorizL7m, "006", UI_Graph_ADD, 2, UI_Color_Yellow, 1, (960-9-9), (540-95), (960+9-9), (540-95));		
				//8 (540-190) "012"
				//Line_Draw(&gAimHorizL8m, "007", UI_Graph_ADD, 2, UI_Color_Yellow, 1, (960-12), (540-113), (960+12), (540-113));
				Line_Draw(&gAimHorizL8m, "007", UI_Graph_ADD, 2, UI_Color_Yellow, 1, (960-7-9+5), (540+41), (960+7-9+5), (540+41));
				// left side line drawing
				Line_Draw(&left8to7, "017", UI_Graph_ADD, 2, UI_Color_Yellow, 1, (960-12-9), (540-113), (960-15-9),(540-95) );
				Line_Draw(&left7to5, "018", UI_Graph_ADD, 2, UI_Color_Yellow, 1, (960-15-9), (540-95), (960-21-9), (540-66));
				Line_Draw(&left5to4, "019", UI_Graph_ADD, 2, UI_Color_Yellow, 1, (960-21-9), (540-66), (960-25-9), (540-45));
				Line_Draw(&left4to2, "020", UI_Graph_ADD, 2, UI_Color_Yellow, 1, (960-25-9), (540-45), (960-50-9), (540-24));
				// right side line drawing
				Line_Draw(&right8to7, "021", UI_Graph_ADD, 2, UI_Color_Yellow, 1, (960+12-9), (540-113), (960+15-9),(540-95) );
				Line_Draw(&right7to5, "022", UI_Graph_ADD, 2, UI_Color_Yellow, 1, (960+15-9), (540-95), (960+21-9), (540-66));
				Line_Draw(&right5to4, "023", UI_Graph_ADD, 2, UI_Color_Yellow, 1, (960+21-9), (540-66), (960+25-9), (540-45));
				Line_Draw(&right4to2, "024", UI_Graph_ADD, 2, UI_Color_Yellow, 1, (960+25-9), (540-45), (960+50-9), (540-24));


				//左边火控相关信息
				Char_Draw(&strCVSts, "012", UI_Graph_ADD, 2, UI_Color_Yellow, 20, 19, 3, CV_STS_X, CV_STS_Y,                              			   "CV:  OFF  AID  LOCK");  
				Char_Draw(&strGunSts, "013", UI_Graph_ADD, 2, UI_Color_Yellow, 20, 19, 3, GUN_STS_X, GUN_STS_Y,                           			   "GUN: OFF  SEMI AUTO");  
//				Char_Draw(&strABoxSts, "014", UI_Graph_ADD, 2, UI_Color_Yellow, 20, 14, 3, AmmoBox_cover_STS_X, AmmoBox_cover_STS_Y,      			   "ABC: OFF  OPEN");
				Char_Draw(&strProjSLimSts, "015", UI_Graph_ADD, 2, UI_Color_Yellow, 20, 8, 3, Enemy_dis_STS_X, Enemy_dis_STS_Y, 									 "DS:    m");
				Char_Draw(&strDisSts, "016", UI_Graph_ADD, 2, UI_Color_Yellow, 20, 10, 3, Projectile_speed_lim_STS_X, Projectile_speed_lim_STS_Y,  "PL:    m/s");
				
				//超级电容 容量静态外框
				Rectangle_Draw(&superCapFrame, "025", UI_Graph_ADD, 3, UI_Color_Main, 3, Center_Bottom_SuperCap_Frame_Start_X, Center_Bottom_SuperCap_Frame_Start_Y, Center_Bottom_SuperCap_Frame_End_X, Center_Bottom_SuperCap_Frame_End_Y);
				//026 027
				//底盘对位线
				ui_info.chassis_drive_pos_line_left_var_endY = ((fp32) (ui_info.chassis_drive_pos_line_left_var_endX - ui_info.chassis_drive_pos_line_left_var_startX) ) * ui_info.chassis_drive_pos_line_left_slope_var + ui_info.chassis_drive_pos_line_left_var_startY;
				ui_info.chassis_drive_pos_line_right_var_endY = ((fp32) (ui_info.chassis_drive_pos_line_right_var_endX - ui_info.chassis_drive_pos_line_right_var_startX) ) * ui_info.chassis_drive_pos_line_right_slope_var + ui_info.chassis_drive_pos_line_right_var_startY;
//				Line_Draw(&chassisPosAimLeftLine, "028", UI_Graph_ADD, 3, UI_Color_Main, 5, Chassis_Drive_Pos_Line_Left_Start_X, Chassis_Drive_Pos_Line_Left_Start_Y, Chassis_Drive_Pos_Line_Left_End_X, Chassis_Drive_Pos_Line_Left_End_Y);
				Line_Draw(&chassisPosAimLeftLine, "028", UI_Graph_ADD, 3, UI_Color_Main, 5, ui_info.chassis_drive_pos_line_left_var_startX, ui_info.chassis_drive_pos_line_left_var_startY, ui_info.chassis_drive_pos_line_left_var_endX, ui_info.chassis_drive_pos_line_left_var_endY);
				Line_Draw(&chassisPosAimRightLine, "029", UI_Graph_ADD, 3, UI_Color_Main, 5, ui_info.chassis_drive_pos_line_right_var_startX, ui_info.chassis_drive_pos_line_right_var_startY, ui_info.chassis_drive_pos_line_right_var_endX, ui_info.chassis_drive_pos_line_right_var_endY);
				
				//更新 同态图标的坐标数据------------
				ui_coord_update();
				//动态图层 占用图层 4,5,6,7 ****在这里加/改了东西之后记得要在ui_dynamic_crt_send_fuc() 里也加/改****
				
				/*右上角 各种设备的error code*/
//				// 5-26-2023 不显示那么详细
//				Char_Draw(&strVarChassis, "985", UI_Graph_Change, 5, UI_Color_Purplish_red, 20, strlen(ui_info.chassis_error_code), 3, CHASSIS_ERROR_CODE_X+ERROR_CODE_STR_X_OFFSET_FROM_FIX_STR, CHASSIS_ERROR_CODE_Y, ui_info.chassis_error_code);
//				Char_Draw(&strVarGimbal, "984", UI_Graph_Change, 5, UI_Color_Purplish_red, 20, strlen(ui_info.gimbal_error_code), 3, GIMBAL_ERROR_CODE_X+ERROR_CODE_STR_X_OFFSET_FROM_FIX_STR, GIMBAL_ERROR_CODE_Y, ui_info.gimbal_error_code);
//				Char_Draw(&strVarShoot, "983", UI_Graph_Change, 5, UI_Color_Purplish_red, 20, strlen(ui_info.shoot_error_code), 3, SHOOT_ERROR_CODE_X+ERROR_CODE_STR_X_OFFSET_FROM_FIX_STR, SHOOT_ERROR_CODE_Y, ui_info.shoot_error_code);
//				Char_Draw(&strVarSuperCap, "982", UI_Graph_Change, 5, UI_Color_Purplish_red, 20, strlen(ui_info.superCap_error_code), 3, SUPERCAP_ERROR_CODE_X+ERROR_CODE_STR_X_OFFSET_FROM_FIX_STR, SUPERCAP_ERROR_CODE_Y, ui_info.superCap_error_code);
//				Char_Draw(&strVarReferee, "981", UI_Graph_Change, 5, UI_Color_Purplish_red, 20, strlen(ui_info.referee_error_code), 3, REFEREE_ERROR_CODE_X+ERROR_CODE_STR_X_OFFSET_FROM_FIX_STR, REFEREE_ERROR_CODE_Y, ui_info.referee_error_code);
				
				//右上角 超级电容电压
				//离线时superCap_info.VBKelvin_fromCap修改为 0
			  //Float_Draw(&fCapVolt, "999", UI_Graph_Change, 4, UI_Color_Main, 20, 2, 3, 1620, 810, ui_info.cap_volt);
				//new position
				Float_Draw(&fCapVolt, "999", UI_Graph_Change, 4, UI_Color_Main, Center_Bottom_SuperCap_VOLT_Font_Size, 2, 3, Center_Bottom_SuperCap_VOLT_NUM_X_COORD, Center_Bottom_SuperCap_VOLT_NUM_Y_COORD, ui_info.cap_volt);
				//右上角 超级电容百分比
				//离线时superCap_info.EBPct_fromCap修改为 0
				//Float_Draw(&fCapPct, "998", UI_Graph_Change, 4, UI_Color_Main, 20, 2, 3, 1620, 840, ui_info.cap_pct);
				//new position with number size = 40
				Float_Draw(&fCapPct, "998", UI_Graph_Change, 4, UI_Color_Main, Center_Bottom_SuperCap_PCT_Font_Size, 2, 3, Center_Bottom_SuperCap_PCT_NUM_X_COORD, Center_Bottom_SuperCap_PCT_NUM_Y_COORD, ui_info.cap_pct);
				//new position of rectangle 右上角方框
				Rectangle_Draw(&gChassisSts_box, "997", UI_Graph_Change, 4, UI_Color_Cyan, 3, ui_info.box_chassis_sts_coord[0], ui_info.box_chassis_sts_coord[1], ui_info.box_chassis_sts_coord[2], ui_info.box_chassis_sts_coord[3]);
				Rectangle_Draw(&gSPINSts_box, "996", UI_Graph_Change, 4, UI_Color_Cyan, 3, ui_info.box_spin_sts_coord[0], ui_info.box_spin_sts_coord[1], ui_info.box_spin_sts_coord[2], ui_info.box_spin_sts_coord[3]);
				
				//左上角---还未改
				Rectangle_Draw(&gCVSts_box, "995", UI_Graph_Change, 4, UI_Color_Cyan, 3, ui_info.box_cv_sts_coord[0], ui_info.box_cv_sts_coord[1], ui_info.box_cv_sts_coord[2], ui_info.box_cv_sts_coord[3]);
				Rectangle_Draw(&gGunSts_box, "994", UI_Graph_Change, 4, UI_Color_Cyan, 3, ui_info.box_gun_sts_coord[0], ui_info.box_gun_sts_coord[1], ui_info.box_gun_sts_coord[2], ui_info.box_gun_sts_coord[3]);
//				Rectangle_Draw(&gABoxSts_box, "993", UI_Graph_Change, 4, UI_Color_Cyan, 3, ui_info.box_ammoBox_sts_coord[0], ui_info.box_ammoBox_sts_coord[1], ui_info.box_ammoBox_sts_coord[2], ui_info.box_ammoBox_sts_coord[3]);
				
				
				Float_Draw(&fProjSLim, "992", UI_Graph_Change, 4, UI_Color_Main, 20, 2, 3, 240, 720, ui_info.enemy_dis);
				Float_Draw(&fDis, "991", UI_Graph_Change, 4, UI_Color_Main, 20, 2, 3, 240, 680, ui_info.proj_speed_limit);
				
				//5-18-23 超级电容 移动能量条
				Line_Draw(&superCapLine, "988", UI_Graph_Change, 4, UI_Color_Main, Center_Bottom_SuperCap_Line_Width, Center_Bottom_SuperCap_Line_Start_X, Center_Bottom_SuperCap_Line_Start_Y, Center_Bottom_SuperCap_Line_Start_X + ui_info.superCap_line_var_length, Center_Bottom_SuperCap_Line_End_Y);
				
				chassis_frame_UI_arm_cal(ui_info.yaw_relative_angle);
				//底盘角度指示框
				Line_Draw(&chassisLine, "987", UI_Graph_Change, 0, UI_Color_Main, Chassis_Frame_Height_Pen, ui_info.frame_chassis_coord_final[0], ui_info.frame_chassis_coord_final[1], ui_info.frame_chassis_coord_final[2], ui_info.frame_chassis_coord_final[3]);
				Line_Draw(&chassisLightBar, "986", UI_Graph_Change, 7, UI_Color_Yellow, Chassis_Frame_Light_Bar_Height_Pen, ui_info.bar_chassis_coord_final[0], ui_info.bar_chassis_coord_final[1], ui_info.bar_chassis_coord_final[2], ui_info.bar_chassis_coord_final[3]);
				//炮塔 球 和 枪 线 捆绑动态图像
				Circle_Draw(&turretCir, "026", UI_Graph_Change, 8, UI_Color_White, Turret_Cir_Pen, Turret_Cir_Start_X, Turret_Cir_Start_Y, Turret_Cir_Radius);
				Line_Draw(&gunLine, "027", UI_Graph_Change, 8, UI_Color_Black, Gun_Line_Pen, Gun_Line_Start_X, Gun_Line_Start_Y, Gun_Line_End_X, Gun_Line_End_Y);
				
				//CV是否识别到目标
				if(is_enemy_detected_with_pc_toe()) //(miniPC_info.enemy_detected == 1)
				{
					Circle_Draw(&gEnemyDetected_circle, "990", UI_Graph_Change, 4, UI_Color_Green, ui_cv_circle_size_debug, TopLeft_Cir_on_cv_DET_START_X, TopLeft_Cir_on_cv_DET_START_Y, TopLeft_Cir_on_cv_DET_radius);
				}
				else
				{
					Circle_Draw(&gEnemyDetected_circle, "990", UI_Graph_Change, 4, UI_Color_Cyan, ui_cv_circle_size_debug, TopLeft_Cir_on_cv_DET_START_X, TopLeft_Cir_on_cv_DET_START_Y, TopLeft_Cir_on_cv_DET_radius);
				}
				
				Rectangle_Draw(&gCVfb_sts_box, "989", UI_Graph_Change, 4, UI_Color_White, 3, ui_info.box_cv_feedback_sts[0], ui_info.box_cv_feedback_sts[1], ui_info.box_cv_feedback_sts[2], ui_info.box_cv_feedback_sts[3]);
				
				//****在这里 上面 加/改了东西之后记得要在ui_dynamic_crt_send_fuc() 里也加/改****
				
				//完成绘制 开始发送 先发静态
				//refresh UI and String(Char)
				if(xTaskGetTickCount() - ui_static_crt_send_TimeStamp >= ui_static_crt_sendPerd) 
				{
					ui_static_crt_send_TimeStamp = xTaskGetTickCount();
					Char_ReFresh(strChassis);
					Char_ReFresh(strGimbal); 
					Char_ReFresh(strShoot); 
					Char_ReFresh(strSuperCap); 
	//				Char_ReFresh(strReferee);
					
					UI_ReFresh(2, chassisPosAimLeftLine, chassisPosAimRightLine);
					UI_ReFresh(2, gAimVertL, superCapFrame);
					UI_ReFresh(5, gAimHorizL2m, gAimHorizL4m, gAimHorizL5m, gAimHorizL7m, gAimHorizL8m);
					UI_ReFresh(5, left8to7, left7to5,left5to4,left4to2, right8to7);
					UI_ReFresh(2,right5to4,right4to2 ); //before-5-wrong #
					UI_ReFresh(1, right7to5); //before-5-wrong #
					//Right
					//Char_ReFresh(strCapVolt);
					//Char_ReFresh(strCapPct);	
					Char_ReFresh(strChassisSts);
					Char_ReFresh(strSPINSts);
					//Left
					Char_ReFresh(strCVSts);
					Char_ReFresh(strGunSts);
	//				Char_ReFresh(strABoxSts);
					Char_ReFresh(strProjSLimSts);
					Char_ReFresh(strDisSts);
				}
				
				//动态的修改 发送
				if(xTaskGetTickCount() - ui_dynamic_chg_send_TimeStamp >= ui_dynamic_chg_sendPerd) 
				{
					ui_dynamic_chg_send_TimeStamp = xTaskGetTickCount();
					UI_ReFresh(5, chassisLine, turretCir, gunLine, fCapVolt, chassisLightBar); //chassisLine, turretCir, gunLine需捆绑发送
					UI_ReFresh(2, fCapPct, superCapLine);
	//				UI_ReFresh(1, superCapLine);
	//				UI_ReFresh(2, superCapLine, chassisLine);
	//				UI_ReFresh(2, fCapVolt, fCapPct);
	//				UI_ReFresh(1, chassisLightBar);
	//				UI_ReFresh(2, fProjSLim, fDis);
					UI_ReFresh(1, fDis); // 7-4去掉弹舱
	//				UI_ReFresh(2, gEnemyDetected_circle, gCVfb_sts_box);
	//		UI_ReFresh(5, gChassisSts_box, gSPINSts_box, gCVSts_box, gGunSts_box, gABoxSts_box);
					UI_ReFresh(7, gEnemyDetected_circle, gCVfb_sts_box, gChassisSts_box, gSPINSts_box, gCVSts_box, gGunSts_box, fProjSLim); // 7-4去掉弹舱
	//				// 5-26-2023 不显示那么详细
	//				Char_ReFresh(strVarChassis);
	//				Char_ReFresh(strVarGimbal); 
	//				Char_ReFresh(strVarShoot); 
	//				Char_ReFresh(strVarSuperCap); 
	//				Char_ReFresh(strVarReferee);
					
					Char_ReFresh(strSpin); //中间靠上 各种字符 Warning 需要时才显示所以不需要只有 add 和 delete 没有change
					Char_ReFresh(strFric);
	//				Char_ReFresh(strRef);
				}
				
				//定时创建一次动态的------之前是错误的: xTaskGetTickCount() - ui_dynamic_crt_sendFreq > ui_dynamic_crt_send_TimeStamp
				if(xTaskGetTickCount() - ui_dynamic_crt_send_TimeStamp >= ui_dynamic_crt_sendPerd)
				{
						ui_dynamic_crt_send_TimeStamp = xTaskGetTickCount(); //更新时间戳 
						ui_dynamic_crt_send_fuc(); //到时间了, 在客户端创建一次动态的图像
				}
//				temp_time_check_RTOS = xTaskGetTickCount();
//			 temp_time_check_HAL = HAL_GetTick();
//			 _temp_a++;
				
//				delLayer.Delete_Operate = UI_Data_Del_Layer;
//				delLayer.Layer = 6;
//				Delete_ReFresh(delLayer);
//				//测试 2023 服务器问题
//				//炮塔 球 和 枪 线
//				Circle_Draw(&turretCir, "026", UI_Graph_ADD, 8, UI_Color_White, Turret_Cir_Pen, Turret_Cir_Start_X, Turret_Cir_Start_Y, Turret_Cir_Radius);
//				Line_Draw(&gunLine, "027", UI_Graph_ADD, 8, UI_Color_Black, Gun_Line_Pen, Gun_Line_Start_X, Gun_Line_Start_Y, Gun_Line_End_X, Gun_Line_End_Y);
//				UI_ReFresh(2, turretCir, gunLine);
				
//				vTaskDelay(100); //100
				vTaskDelay(100); //100
				//
				client_ui_count_ref++;
				
				if(client_ui_count_ref % 10 == 0)
					client_ui_test_flag++; // use ++ to debug instead of -1 * client_ui_test_flag
				
			
#if INCLUDE_uxTaskGetStackHighWaterMark
        client_ui_task_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif	
		}
}

/*通过判断相关 状态机 -> ui坐标 */
void ui_coord_update()
{
	 //底盘 turbo 模式的判断
	 if(get_turboMode() == 0)
	 {
		 ui_info.ui_chassis_sts = NORM;
		 ui_info.box_chassis_sts_coord[0] = TopRight_REC_on_NORM_START_X;
		 ui_info.box_chassis_sts_coord[1] = TopRight_REC_on_NORM_START_Y;
		 ui_info.box_chassis_sts_coord[2] = TopRight_REC_on_NORM_END_X;
		 ui_info.box_chassis_sts_coord[3] = TopRight_REC_on_NORM_END_Y;
	 }
	 else
	 {
		 ui_info.ui_chassis_sts = BOOST;
		 ui_info.box_chassis_sts_coord[0] = TopRight_REC_on_BOOST_START_X;
		 ui_info.box_chassis_sts_coord[1] = TopRight_REC_on_BOOST_START_Y;
		 ui_info.box_chassis_sts_coord[2] = TopRight_REC_on_BOOST_END_X;
		 ui_info.box_chassis_sts_coord[3] = TopRight_REC_on_BOOST_END_Y;
	 }
	 
	 /*
		swing_flag是小陀螺状态机
		swing_flag = 0 无小陀螺
		swing_flag = 1 有小陀螺
		*/
	 if(get_swing_flag() == 0)
	 {
		 ui_info.ui_spin_sts = spinFOLL;
		 ui_info.box_spin_sts_coord[0] = TopRight_REC_on_FOLL_START_X;
		 ui_info.box_spin_sts_coord[1] = TopRight_REC_on_FOLL_START_Y;
		 ui_info.box_spin_sts_coord[2] = TopRight_REC_on_FOLL_END_X;
		 ui_info.box_spin_sts_coord[3] = TopRight_REC_on_FOLL_END_Y;
		 
		Char_Draw(&strSpin, "980", UI_Graph_ADD, 5, UI_Color_Purplish_red, Robot_Warning_Msg_Font_Size, strlen("SPIN!"), 3, Robot_Warning_Spin_X, Robot_Warning_Spin_Y, "SPIN!");
	 }
	 else
	 {
		 ui_info.ui_spin_sts = spinSPIN;
		 ui_info.box_spin_sts_coord[0] = TopRight_REC_on_SPIN_START_X;
		 ui_info.box_spin_sts_coord[1] = TopRight_REC_on_SPIN_START_Y;
		 ui_info.box_spin_sts_coord[2] = TopRight_REC_on_SPIN_END_X;
		 ui_info.box_spin_sts_coord[3] = TopRight_REC_on_SPIN_END_Y;
		 
		 Char_Draw(&strSpin, "980", UI_Graph_Del, 5, UI_Color_Purplish_red, Robot_Warning_Msg_Font_Size, strlen("SPIN!"), 3, Robot_Warning_Spin_X, Robot_Warning_Spin_Y, "SPIN!");
	 }
	 	 
	 //CV 状态机 状态 //自动瞄准开关状态 0关 1自动瞄准
	 if(get_autoAimFlag() == 0) //(miniPC_info.autoAimFlag == 0)
	 {
		 ui_info.ui_cv_sts = cvOFF;
		 ui_info.box_cv_sts_coord[0] = TopLeft_REC_on_cv_OFF_START_X;
		 ui_info.box_cv_sts_coord[1] = TopLeft_REC_on_cv_OFF_START_Y;
		 ui_info.box_cv_sts_coord[2] = TopLeft_REC_on_cv_OFF_END_X;
		 ui_info.box_cv_sts_coord[3] = TopLeft_REC_on_cv_OFF_END_Y;
	 }
	 else if(get_autoAimFlag() == 1) //(miniPC_info.autoAimFlag == 1)
	 {
		 ui_info.ui_cv_sts = cvAID;
		 ui_info.box_cv_sts_coord[0] = TopLeft_REC_on_cv_AID_START_X;
		 ui_info.box_cv_sts_coord[1] = TopLeft_REC_on_cv_AID_START_Y;
		 ui_info.box_cv_sts_coord[2] = TopLeft_REC_on_cv_AID_END_X;
		 ui_info.box_cv_sts_coord[3] = TopLeft_REC_on_cv_AID_END_Y;
	 }
	 else if(get_autoAimFlag() == 2) //(miniPC_info.autoAimFlag == 2)
	 {
		 ui_info.ui_cv_sts = cvLOCK;
		 ui_info.box_cv_sts_coord[0] = TopLeft_REC_on_cv_LOCK_START_X;
		 ui_info.box_cv_sts_coord[1] = TopLeft_REC_on_cv_LOCK_START_Y;
		 ui_info.box_cv_sts_coord[2] = TopLeft_REC_on_cv_LOCK_END_X;
		 ui_info.box_cv_sts_coord[3] = TopLeft_REC_on_cv_LOCK_END_Y;
	 }
	 
	 //CV feedback 状态机 - 7-4-2023修改为: cv_gimbal_st只有0-cvOFF或其它-cvNORMAL
	 if(get_cv_gimbal_sts() == 0) //(miniPC_info.cv_status == 0)
	 {
		 ui_info.ui_cv_feedback_sts = cvOFF;
		 ui_info.box_cv_feedback_sts[0] = TopLeft_CV_FEEDBACK_STATUS_on_OFF_START_X;
		 ui_info.box_cv_feedback_sts[1] = TopLeft_CV_FEEDBACK_STATUS_on_OFF_START_Y;
		 ui_info.box_cv_feedback_sts[2] = TopLeft_CV_FEEDBACK_STATUS_on_OFF_END_X;
		 ui_info.box_cv_feedback_sts[3] = TopLeft_CV_FEEDBACK_STATUS_on_OFF_END_Y;
	 }
	 else
	 {
		 ui_info.ui_cv_feedback_sts = cvNORMAL; //对应的框也将会是一个大框
		 ui_info.box_cv_feedback_sts[0] = TopLeft_CV_FEEDBACK_STATUS_on_AID_START_X;
		 ui_info.box_cv_feedback_sts[1] = TopLeft_CV_FEEDBACK_STATUS_on_AID_START_Y;
		 ui_info.box_cv_feedback_sts[2] = TopLeft_CV_FEEDBACK_STATUS_on_LOCK_END_X;
		 ui_info.box_cv_feedback_sts[3] = TopLeft_CV_FEEDBACK_STATUS_on_LOCK_END_Y;
	 }
//	 else if(get_cv_gimbal_sts() == 1) //(miniPC_info.cv_status == 1)
//	 {
//		 ui_info.ui_cv_feedback_sts = cvAID;
//		 ui_info.box_cv_feedback_sts[0] = TopLeft_CV_FEEDBACK_STATUS_on_AID_START_X;
//		 ui_info.box_cv_feedback_sts[1] = TopLeft_CV_FEEDBACK_STATUS_on_AID_START_Y;
//		 ui_info.box_cv_feedback_sts[2] = TopLeft_CV_FEEDBACK_STATUS_on_AID_END_X;
//		 ui_info.box_cv_feedback_sts[3] = TopLeft_CV_FEEDBACK_STATUS_on_AID_END_Y;
//	 }
//	 else if(get_cv_gimbal_sts() == 2) //(miniPC_info.cv_status == 2)
//	 {
//		 ui_info.ui_cv_feedback_sts = cvLOCK;
//		 ui_info.box_cv_feedback_sts[0] = TopLeft_CV_FEEDBACK_STATUS_on_LOCK_START_X;
//		 ui_info.box_cv_feedback_sts[1] = TopLeft_CV_FEEDBACK_STATUS_on_LOCK_START_Y;
//		 ui_info.box_cv_feedback_sts[2] = TopLeft_CV_FEEDBACK_STATUS_on_LOCK_END_X;
//		 ui_info.box_cv_feedback_sts[3] = TopLeft_CV_FEEDBACK_STATUS_on_LOCK_END_Y;
//	 }
	 
	 //CV状态机掉线提示
	 if(toe_is_error(PC_TOE))
	 {
		 ui_info.ui_cv_feedback_sts = cvOFF;
		 ui_info.box_cv_feedback_sts[0] = TopLeft_CV_FEEDBACK_STATUS_on_OFF_START_X;
		 ui_info.box_cv_feedback_sts[1] = TopLeft_CV_FEEDBACK_STATUS_on_OFF_START_Y;
		 ui_info.box_cv_feedback_sts[2] = TopLeft_CV_FEEDBACK_STATUS_on_OFF_END_X;
		 ui_info.box_cv_feedback_sts[3] = TopLeft_CV_FEEDBACK_STATUS_on_OFF_END_Y;
	 }
	 
	 //GUN 状态机 状态
	 if(get_shoot_mode() == SHOOT_STOP)
	 {
		 ui_info.ui_gun_sts = gunOFF;
		 ui_info.box_gun_sts_coord[0] = TopLeft_REC_on_gun_OFF_START_X;
		 ui_info.box_gun_sts_coord[1] = TopLeft_REC_on_gun_OFF_START_Y;
		 ui_info.box_gun_sts_coord[2] = TopLeft_REC_on_gun_OFF_END_X;
		 ui_info.box_gun_sts_coord[3] = TopLeft_REC_on_gun_OFF_END_Y;
		 
		 Char_Draw(&strFric, "979", UI_Graph_ADD, 5, UI_Color_Purplish_red, Robot_Warning_Msg_Font_Size, strlen("FRIC!"), 3, Robot_Warning_Fric_X, Robot_Warning_Fric_Y, "FRIC!");
	 }
	 else if(get_user_fire_ctrl() == user_SHOOT_AUTO) // 指交替发射
	 {
		 ui_info.ui_gun_sts = gunAUTO;
		 ui_info.box_gun_sts_coord[0] = TopLeft_REC_on_gun_AUTO_START_X;
		 ui_info.box_gun_sts_coord[1] = TopLeft_REC_on_gun_AUTO_START_Y;
		 ui_info.box_gun_sts_coord[2] = TopLeft_REC_on_gun_AUTO_END_X;
		 ui_info.box_gun_sts_coord[3] = TopLeft_REC_on_gun_AUTO_END_Y;
		 
		 Char_Draw(&strFric, "979", UI_Graph_Del, 5, UI_Color_Purplish_red, Robot_Warning_Msg_Font_Size, strlen("FRIC!"), 3, Robot_Warning_Fric_X, Robot_Warning_Fric_Y, "FRIC!");
	 }
	 else if(get_user_fire_ctrl() == user_SHOOT_SEMI) //同时发射就是指semi user_SHOOT_SEMI
	 {
		 ui_info.ui_gun_sts = gunSEMI;
		 ui_info.box_gun_sts_coord[0] = TopLeft_REC_on_gun_SEMI_START_X;
		 ui_info.box_gun_sts_coord[1] = TopLeft_REC_on_gun_SEMI_START_Y;
		 ui_info.box_gun_sts_coord[2] = TopLeft_REC_on_gun_SEMI_END_X;
		 ui_info.box_gun_sts_coord[3] = TopLeft_REC_on_gun_SEMI_END_Y;
		 
		 Char_Draw(&strFric, "979", UI_Graph_Del, 5, UI_Color_Purplish_red, Robot_Warning_Msg_Font_Size, strlen("FRIC!"), 3, Robot_Warning_Fric_X, Robot_Warning_Fric_Y, "FRIC!");
	 }
	 
//	 //Ammo Box 状态机 状态
//	 if(get_ammoBox_sts() == ammoOFF)
//	 {
//		 ui_info.ui_ammoBox_sts = ammoOFF;
//		 //ui_info.box_ammoBox_sts_coord
//		 ui_info.box_ammoBox_sts_coord[0] = TopLeft_REC_on_ammo_OFF_START_X;
//		 ui_info.box_ammoBox_sts_coord[1] = TopLeft_REC_on_ammo_OFF_START_Y;
//		 ui_info.box_ammoBox_sts_coord[2] = TopLeft_REC_on_ammo_OFF_END_X;
//		 ui_info.box_ammoBox_sts_coord[3] = TopLeft_REC_on_ammo_OFF_END_Y;
//	 }
//	 else if(get_ammoBox_sts() == ammoOPEN)
//	 {
//		 ui_info.ui_ammoBox_sts = ammoOPEN;
//		 //ui_info.box_ammoBox_sts_coord
//		 ui_info.box_ammoBox_sts_coord[0] = TopLeft_REC_on_ammo_OPEN_START_X;
//		 ui_info.box_ammoBox_sts_coord[1] = TopLeft_REC_on_ammo_OPEN_START_Y;
//		 ui_info.box_ammoBox_sts_coord[2] = TopLeft_REC_on_ammo_OPEN_END_X;
//		 ui_info.box_ammoBox_sts_coord[3] = TopLeft_REC_on_ammo_OPEN_END_Y;
//	 }

	 //开始整数字相关的东西 即插即用的超级电容控制板 判断
//	 ui_info.cap_pct = ui_get_current_cap_voltage();
	 ui_info.cap_volt = ui_get_current_cap_voltage();
	 ui_info.cap_relative_pct = ui_get_current_cap_relative_pct();
	 ui_info.cap_pct = ui_info.cap_relative_pct;
	 
//	 if(current_superCap == SuperCap_ID)
//	 {
//		 if(toe_is_error(ZIDACAP_TOE))
//		 {
//			 ui_info.cap_pct = 0.0f;
//			 ui_info.cap_volt = 0.0f;
//		 }
//		 else
//		 {
//			 ui_info.cap_pct = superCap_info.EBPct_fromCap;
//			 ui_info.cap_volt = superCap_info.VBKelvin_fromCap;
//		 }
//	 }
//	 else if(current_superCap == gen2Cap_ID)
//	 {
//		 if(toe_is_error(GEN2CAP_TOE))
//		 {
//			 ui_info.cap_pct = 0.0f;
//			 ui_info.cap_volt = 0.0f;
//		 }
//		 else
//		 {
//			 ui_info.cap_pct = gen2Cap_info.EBPct;
//		   ui_info.cap_volt = gen2Cap_info.Vbank_f;
//		 }
//	 }
//	 else
//	 {
//		 if(toe_is_error(WULIECAP_TOE))
//		 {
//			 ui_info.cap_pct = 0.0f;
//			 ui_info.cap_volt = 0.0f;
//		 }
//		 else
//		 {
//			 ui_info.cap_pct = wulieCap_info.EBPct;
//		   ui_info.cap_volt = wulieCap_info.cap_voltage;
//		 }
//	 }
	 
	 //ui_info.enemy_dis = miniPC_info.dis; //和张哥商量 3-26: ui包的发送于必要性
	 ui_info.enemy_dis = get_aim_pos_dis(); 
	 ui_info.proj_speed_limit = get_shooter_id1_17mm_speed_limit();
	 
	 //超级电容相关
	 ui_info.superCap_line_var_length = (uint16_t) Center_Bottom_SuperCap_Line_Length_Max * fp32_constrain( ui_get_current_cap_relative_pct(), 0.0f, 1.0f);
	 
	 chassis_frame_UI_sensor_update(); //update gimbal yaw angle
	 ui_error_code_update();
}

void ui_dynamic_crt_send_fuc()
{
		/*右上角 各种设备的error code*/
	  // 5-26-2023 不显示那么详细	
//		Char_Draw(&strVarChassis, "985", ui_info.chassis_error_flag, 5, UI_Color_Purplish_red, 20, strlen(ui_info.chassis_error_code), 3, CHASSIS_ERROR_CODE_X+ERROR_CODE_STR_X_OFFSET_FROM_FIX_STR, CHASSIS_ERROR_CODE_Y, ui_info.chassis_error_code);
//		Char_Draw(&strVarGimbal, "984", ui_info.gimbal_error_flag, 5, UI_Color_Purplish_red, 20, strlen(ui_info.gimbal_error_code), 3, GIMBAL_ERROR_CODE_X+ERROR_CODE_STR_X_OFFSET_FROM_FIX_STR, GIMBAL_ERROR_CODE_Y, ui_info.gimbal_error_code);
//		Char_Draw(&strVarShoot, "983", ui_info.shoot_error_flag, 5, UI_Color_Purplish_red, 20, strlen(ui_info.shoot_error_code), 3, SHOOT_ERROR_CODE_X+ERROR_CODE_STR_X_OFFSET_FROM_FIX_STR, SHOOT_ERROR_CODE_Y, ui_info.shoot_error_code);
//		Char_Draw(&strVarSuperCap, "982", ui_info.superCap_error_flag, 5, UI_Color_Purplish_red, 20, strlen(ui_info.superCap_error_code), 3, SUPERCAP_ERROR_CODE_X+ERROR_CODE_STR_X_OFFSET_FROM_FIX_STR, SUPERCAP_ERROR_CODE_Y, ui_info.superCap_error_code);
//		Char_Draw(&strVarReferee, "981", ui_info.referee_error_flag, 5, UI_Color_Purplish_red, 20, strlen(ui_info.referee_error_code), 3, REFEREE_ERROR_CODE_X+ERROR_CODE_STR_X_OFFSET_FROM_FIX_STR, REFEREE_ERROR_CODE_Y, ui_info.referee_error_code);
	
		//右上角 超级电容电压
		//离线时superCap_info.VBKelvin_fromCap修改为 0
		//Float_Draw(&fCapVolt, "999", UI_Graph_ADD, 4, UI_Color_Main, 20, 2, 3, 1620, 810, ui_info.cap_volt);
	//new postiion of voltage of super-cap
			Float_Draw(&fCapVolt, "999", UI_Graph_ADD, 4, UI_Color_Main, Center_Bottom_SuperCap_VOLT_Font_Size, 2, 3, Center_Bottom_SuperCap_VOLT_NUM_X_COORD, Center_Bottom_SuperCap_VOLT_NUM_Y_COORD, ui_info.cap_volt);
		//右上角 超级电容百分比
		//离线时superCap_info.EBPct_fromCap修改为 0
		//Float_Draw(&fCapPct, "998", UI_Graph_ADD, 4, UI_Color_Main, 20, 2, 3, 1620, 840, ui_info.cap_pct);
	//new position with larger size of number size = 40
		Float_Draw(&fCapPct, "998", UI_Graph_ADD, 4, UI_Color_Main, Center_Bottom_SuperCap_PCT_Font_Size, 2, 3, Center_Bottom_SuperCap_PCT_NUM_X_COORD, Center_Bottom_SuperCap_PCT_NUM_Y_COORD, ui_info.cap_pct);
		//右上角方框
		Rectangle_Draw(&gChassisSts_box, "997", UI_Graph_ADD, 4, UI_Color_Cyan, 3, ui_info.box_chassis_sts_coord[0], ui_info.box_chassis_sts_coord[1], ui_info.box_chassis_sts_coord[2], ui_info.box_chassis_sts_coord[3]);
		Rectangle_Draw(&gSPINSts_box, "996", UI_Graph_ADD, 4, UI_Color_Cyan, 3, ui_info.box_spin_sts_coord[0], ui_info.box_spin_sts_coord[1], ui_info.box_spin_sts_coord[2], ui_info.box_spin_sts_coord[3]);
				
		//左上角---还未改
		Rectangle_Draw(&gCVSts_box, "995", UI_Graph_ADD, 4, UI_Color_Cyan, 3, ui_info.box_cv_sts_coord[0], ui_info.box_cv_sts_coord[1], ui_info.box_cv_sts_coord[2], ui_info.box_cv_sts_coord[3]);
		Rectangle_Draw(&gGunSts_box, "994", UI_Graph_ADD, 4, UI_Color_Cyan, 3, ui_info.box_gun_sts_coord[0], ui_info.box_gun_sts_coord[1], ui_info.box_gun_sts_coord[2], ui_info.box_gun_sts_coord[3]);
//		Rectangle_Draw(&gABoxSts_box, "993", UI_Graph_ADD, 4, UI_Color_Cyan, 3, ui_info.box_ammoBox_sts_coord[0], ui_info.box_ammoBox_sts_coord[1], ui_info.box_ammoBox_sts_coord[2], ui_info.box_ammoBox_sts_coord[3]);
		
		Float_Draw(&fProjSLim, "992", UI_Graph_ADD, 4, UI_Color_Main, 20, 2, 3, 240, 720, ui_info.proj_speed_limit);
		Float_Draw(&fDis, "991", UI_Graph_ADD, 4, UI_Color_Main, 20, 2, 3, 240, 680, ui_info.enemy_dis);
		
		//5-18-23 超级电容 移动能量条
		Line_Draw(&superCapLine, "988", UI_Graph_ADD, 4, UI_Color_Main, 10, Center_Bottom_SuperCap_Line_Start_X, Center_Bottom_SuperCap_Line_Start_Y, Center_Bottom_SuperCap_Line_Start_X + ui_info.superCap_line_var_length, Center_Bottom_SuperCap_Line_End_Y);
		
		chassis_frame_UI_arm_cal(ui_info.yaw_relative_angle);
		//底盘角度指示框
		Line_Draw(&chassisLine, "987", UI_Graph_ADD, 0, UI_Color_Main, Chassis_Frame_Height_Pen, ui_info.frame_chassis_coord_final[0], ui_info.frame_chassis_coord_final[1], ui_info.frame_chassis_coord_final[2], ui_info.frame_chassis_coord_final[3]);
		Line_Draw(&chassisLightBar, "986", UI_Graph_ADD, 7, UI_Color_Yellow, Chassis_Frame_Light_Bar_Height_Pen, ui_info.bar_chassis_coord_final[0], ui_info.bar_chassis_coord_final[1], ui_info.bar_chassis_coord_final[2], ui_info.bar_chassis_coord_final[3]);
		//炮塔 球 和 枪 线 捆绑动态图像
		Circle_Draw(&turretCir, "026", UI_Graph_ADD, 8, UI_Color_White, Turret_Cir_Pen, Turret_Cir_Start_X, Turret_Cir_Start_Y, Turret_Cir_Radius);
		Line_Draw(&gunLine, "027", UI_Graph_ADD, 8, UI_Color_Black, Gun_Line_Pen, Gun_Line_Start_X, Gun_Line_Start_Y, Gun_Line_End_X, Gun_Line_End_Y);
				
		//CV是否识别到目标
		if(is_enemy_detected_with_pc_toe()) //(miniPC_info.enemy_detected == 1)
		{
			Circle_Draw(&gEnemyDetected_circle, "990", UI_Graph_ADD, 4, UI_Color_Green, ui_cv_circle_size_debug, TopLeft_Cir_on_cv_DET_START_X, TopLeft_Cir_on_cv_DET_START_Y, TopLeft_Cir_on_cv_DET_radius);
		}
		else
		{
			Circle_Draw(&gEnemyDetected_circle, "990", UI_Graph_ADD, 4, UI_Color_Cyan, ui_cv_circle_size_debug, TopLeft_Cir_on_cv_DET_START_X, TopLeft_Cir_on_cv_DET_START_Y, TopLeft_Cir_on_cv_DET_radius);
		}
		
		Rectangle_Draw(&gCVfb_sts_box, "989", UI_Graph_ADD, 4, UI_Color_White, 3, ui_info.box_cv_feedback_sts[0], ui_info.box_cv_feedback_sts[1], ui_info.box_cv_feedback_sts[2], ui_info.box_cv_feedback_sts[3]);
				
		//新增绘制结束		
				
		//动态的修改 发送
		UI_ReFresh(5, chassisLine, turretCir, gunLine, fCapVolt, chassisLightBar); //chassisLine, turretCir, gunLine需捆绑发送
		UI_ReFresh(2, fCapPct, superCapLine);
//		UI_ReFresh(1, superCapLine);
//		UI_ReFresh(2, superCapLine, chassisLine);
//		UI_ReFresh(2, fCapVolt, fCapPct);
//		UI_ReFresh(2, fProjSLim, fDis);
		UI_ReFresh(1, fDis); // 7-4去掉弹舱
		UI_ReFresh(2, gEnemyDetected_circle, gCVfb_sts_box);
//		UI_ReFresh(5, gChassisSts_box, gSPINSts_box, gCVSts_box, gGunSts_box, gABoxSts_box);
		UI_ReFresh(7, gEnemyDetected_circle, gCVfb_sts_box, gChassisSts_box, gSPINSts_box, gCVSts_box, gGunSts_box, fProjSLim); // 7-4去掉弹舱
//		// 5-26-2023 不显示那么详细
//		Char_ReFresh(strVarChassis);
//	  Char_ReFresh(strVarGimbal); 
//		Char_ReFresh(strVarShoot); 
//		Char_ReFresh(strVarSuperCap); 
//		Char_ReFresh(strVarReferee);
}

////先开始删除 图层4 5 6 7
//				delLayer.Delete_Operate = UI_Data_Del_Layer;
//				delLayer.Layer = 4;
//				Delete_ReFresh(delLayer);
//			
//			  delLayer.Layer = 5;
//				Delete_ReFresh(delLayer);
//			
//			  delLayer.Layer = 6;
//				Delete_ReFresh(delLayer);
//			
//			  delLayer.Layer = 7;
//				Delete_ReFresh(delLayer);


