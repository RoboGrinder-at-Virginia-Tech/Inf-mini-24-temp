/*************************************************************

RM自定义UI协议

弗吉尼亚理工 Virginia Tech; RoboGrinder

此文件包含驱动层: UI绘制驱动, (CRC校验驱动, 机器人裁判系统部分信息均重复包含在此文件内),
和 绘制静态UI, 判断是否发生变化并绘制修改动态UI

**************************************************************/

#include "referee_ui.h"
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

// 动态UI的刷新频率
const uint16_t ui_dynamic_crt_send_period = 250; //500; //5000

/** ---- 以下为 静态UI ---- **/
// 瞄准线, infantry only keep gAimVertL and gAimHorizL8m
Graph_Data gAimVertL, gAimHorizL8m;
// heor still needs other lines
Graph_Data gAimHorizL1m, gAimHorizL2m, gAimHorizL4m, gAimHorizL5m, gAimHorizL7m, left8to7, left7to5,left5to4,left4to2, right8to7, right7to5,right5to4,right4to2;

String_Data strChassisSts, strSPINSts; // 底盘状态
String_Data strCVSts, strGunSts, strProjSLimSts; // 自瞄火控状态

//底盘对位线
Graph_Data chassisPosAimLeftLine, chassisPosAimRightLine;
/*****************************/


/** ---- 以下为 弹出式UI ---- **/
String_Data strChassis, strGimbal, strShoot, strSuperCap; // 模块offline消息
String_Data strSpin, strFric; // 正中间弹出来的消息
/*****************************/

/** ---- 以下为 动态UI ---- **/
Float_Data fCapVolt, fCapPct; // 超级电容电压和能量相对百分比
Float_Data fProjSLim; // 当前弹速上限

//方框
Graph_Data gChassisSts_box, gSPINSts_box;// 底盘状态方框
Graph_Data gCVSts_box, gGunSts_box; // CV用户控制 枪火控状态 已去掉弹舱盖 gABoxSts_box
Graph_Data gEnemyDetected_circle, gCVfb_sts_box; // 敌人是否识别到 CV反馈状态

//超级电容能量框
Graph_Data superCapFrame; // 外框 长方形
Graph_Data superCapLine; // 里面填充

//底盘角度指示器
Graph_Data chassisLine, chassisLightBar; // 表示底盘的线和枪管
Graph_Data turretCir, gunLine; // 炮塔和枪口
/*****************************/

//删除图层结构体
UI_Data_Delete delLayer;

//UI信息封装结构体
interactive_ui_info_t ui_info;

uint16_t ui_cv_circle_size_debug = TopLeft_Cir_on_cv_DET_Pen_Size;

/**********************************变量定义结束*********************************/

/* ---------------------------- 底盘角度指示器helper function 开始 ---------------------------- */
//初始化 准备使用arm矩阵库去计算底盘指示器角度
static void chassis_frame_UI_sensor_and_graph_init()
{
	//初始化数据 底盘框
	ui_info.gimbal_control_ptr = get_gimbal_pointer();
	ui_info.frame_chassis_coord_start_raw[0] = Chassis_Frame_Start_X;
	ui_info.frame_chassis_coord_start_raw[1] = Chassis_Frame_Start_Y;
	ui_info.frame_chassis_coord_end_raw[0] = Chassis_Frame_End_X;
	ui_info.frame_chassis_coord_end_raw[1] = Chassis_Frame_End_Y;
	
	//初始化数据 灯条
	ui_info.bar_chassis_coord_start_raw[0] = Chassis_Frame_Light_Bar_Start_X;
	ui_info.bar_chassis_coord_start_raw[1] = Chassis_Frame_Light_Bar_Start_Y;
	ui_info.bar_chassis_coord_end_raw[0] = Chassis_Frame_Light_Bar_End_X;
	ui_info.bar_chassis_coord_end_raw[1] = Chassis_Frame_Light_Bar_End_Y;
	
	//坐标轴变换 底盘框
	ui_info.frame_chassis_coord_final[0] = ui_info.frame_chassis_coord_start_raw[0] + Chassis_Frame_Coord_Center_X;
	ui_info.frame_chassis_coord_final[1] = ui_info.frame_chassis_coord_start_raw[1] + Chassis_Frame_Coord_Center_Y;
	ui_info.frame_chassis_coord_final[2] = ui_info.frame_chassis_coord_end_raw[0] + Chassis_Frame_Coord_Center_X;
	ui_info.frame_chassis_coord_final[3] = ui_info.frame_chassis_coord_end_raw[1] + Chassis_Frame_Coord_Center_Y;
	
	//坐标轴变换 灯条
	ui_info.bar_chassis_coord_final[0] = ui_info.bar_chassis_coord_start_raw[0] + Chassis_Frame_Coord_Center_X;
	ui_info.bar_chassis_coord_final[1] = ui_info.bar_chassis_coord_start_raw[1] + Chassis_Frame_Coord_Center_Y;
	ui_info.bar_chassis_coord_final[2] = ui_info.bar_chassis_coord_end_raw[0] + Chassis_Frame_Coord_Center_X;
	ui_info.bar_chassis_coord_final[3] = ui_info.bar_chassis_coord_end_raw[1] + Chassis_Frame_Coord_Center_Y;
}
static void chassis_frame_UI_sensor_update()
{
	//由于是转的地盘 云台夹需要取-
	ui_info.yaw_relative_angle = -rad_format(ui_info.gimbal_control_ptr->gimbal_yaw_motor.relative_angle);
}
static void chassis_frame_UI_arm_init(fp32 angle)
{
	//底盘框
	mat_init(&ui_info.frame_chassis_coord_start_vec, 2, 1, (fp32 *) ui_info.frame_chassis_coord_start_raw);
	mat_init(&ui_info.frame_chassis_coord_end_vec, 2, 1, (fp32 *) ui_info.frame_chassis_coord_end_raw);
	
	mat_init(&ui_info.new_frame_chassis_coord_start_vec, 2, 1, (fp32 *) ui_info.new_frame_chassis_coord_start_raw);
	mat_init(&ui_info.new_frame_chassis_coord_end_vec, 2, 1, (fp32 *) ui_info.new_frame_chassis_coord_end_raw);
	
	//灯条
	mat_init(&ui_info.bar_chassis_coord_start_vec, 2, 1, (fp32 *) ui_info.bar_chassis_coord_start_raw);
	mat_init(&ui_info.bar_chassis_coord_end_vec, 2, 1, (fp32 *) ui_info.bar_chassis_coord_end_raw);
	
	mat_init(&ui_info.new_bar_chassis_coord_start_vec, 2, 1, (fp32 *) ui_info.new_bar_chassis_coord_start_raw);
	mat_init(&ui_info.new_bar_chassis_coord_end_vec, 2, 1, (fp32 *) ui_info.new_bar_chassis_coord_end_raw);
	
	/* matrix element (i, j) is stored at: pData[i*numCols + j] */
	ui_info.frame_chassis_rotation_matrix_raw[0] = arm_cos_f32(angle);
	ui_info.frame_chassis_rotation_matrix_raw[1] = -arm_sin_f32(angle);
	ui_info.frame_chassis_rotation_matrix_raw[2] = arm_sin_f32(angle);
	ui_info.frame_chassis_rotation_matrix_raw[3] = arm_cos_f32(angle);
	
	mat_init(&ui_info.chassis_rotation_matrix, 2, 2, (fp32 *) ui_info.frame_chassis_rotation_matrix_raw);
}
//计算底盘指示器角度
static void chassis_frame_UI_arm_cal(fp32 angle)
{
	ui_info.chassis_rotation_matrix.pData[0] = arm_cos_f32(angle);
	ui_info.chassis_rotation_matrix.pData[1] = -arm_sin_f32(angle);
	ui_info.chassis_rotation_matrix.pData[2] = arm_sin_f32(angle);
	ui_info.chassis_rotation_matrix.pData[3] = arm_cos_f32(angle);
	
	//计算旋转后坐标 底盘框
	mat_mult(&ui_info.chassis_rotation_matrix, &ui_info.frame_chassis_coord_start_vec, &ui_info.new_frame_chassis_coord_start_vec);
	mat_mult(&ui_info.chassis_rotation_matrix, &ui_info.frame_chassis_coord_end_vec, &ui_info.new_frame_chassis_coord_end_vec);
	
	//计算旋转后坐标 灯条
	mat_mult(&ui_info.chassis_rotation_matrix, &ui_info.bar_chassis_coord_start_vec, &ui_info.new_bar_chassis_coord_start_vec);
	mat_mult(&ui_info.chassis_rotation_matrix, &ui_info.bar_chassis_coord_end_vec, &ui_info.new_bar_chassis_coord_end_vec);
	
	//坐标轴变换 底盘框
	ui_info.frame_chassis_coord_final[0] = ui_info.new_frame_chassis_coord_start_vec.pData[0] + Chassis_Frame_Coord_Center_X;
	ui_info.frame_chassis_coord_final[1] = ui_info.new_frame_chassis_coord_start_vec.pData[1] + Chassis_Frame_Coord_Center_Y;
	ui_info.frame_chassis_coord_final[2] = ui_info.new_frame_chassis_coord_end_vec.pData[0] + Chassis_Frame_Coord_Center_X;
	ui_info.frame_chassis_coord_final[3] = ui_info.new_frame_chassis_coord_end_vec.pData[1] + Chassis_Frame_Coord_Center_Y;
	
	//坐标轴变换 灯条
	ui_info.bar_chassis_coord_final[0] = ui_info.new_bar_chassis_coord_start_vec.pData[0] + Chassis_Frame_Coord_Center_X;
	ui_info.bar_chassis_coord_final[1] = ui_info.new_bar_chassis_coord_start_vec.pData[1] + Chassis_Frame_Coord_Center_Y;
	ui_info.bar_chassis_coord_final[2] = ui_info.new_bar_chassis_coord_end_vec.pData[0] + Chassis_Frame_Coord_Center_X;
	ui_info.bar_chassis_coord_final[3] = ui_info.new_bar_chassis_coord_end_vec.pData[1] + Chassis_Frame_Coord_Center_Y;
}
/* ---------------------------- 底盘角度指示器helper function结束 ---------------------------- */

// This function draws the static UI
void static_UI_func()
{
}

// This function draws the dynamic UI
void dynamic_UI_func()
{
}

/**************************************** 以下为驱动层 *****************************************/

unsigned char UI_Seq;     //包序号

/****************************************串口驱动映射************************************/
void UI_SendByte(unsigned char ch)
{
//   USART_SendData(USART3,ch);
//   while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);	
	HAL_UART_Transmit(&huart6, (uint8_t*)&ch, 1,99999); //-----
//	while(HAL_UART_GetState(&huart6) == HAL_UART_STATE_BUSY_TX)
//	{
//		vTaskDelay(1);
//	}
	
//	// ---------------------------------------------------
//	HAL_UART_Transmit_IT(&huart6, (uint8_t*)&ch, 1);
//	//while(!(huart6.Instance->SR))
//	while(!(__HAL_UART_GET_FLAG(&huart6, UART_FLAG_TC) == 1))
//	{
//		vTaskDelay(1);
//	}
//	// ---------------------------------------------------
}

/********************************************删除操作*************************************
**参数：Del_Operate  对应头文件删除操作
        Del_Layer    要删除的层 取值0-9
*****************************************************************************************/

void UI_Delete(u8 Del_Operate,u8 Del_Layer)
{

   unsigned char *framepoint;                      //读写指针
   u16 frametail=0xFFFF;                        //CRC16校验值
   int loop_control;                       //For函数循环控制
   
   UI_Packhead framehead;
   UI_Data_Operate datahead;
   UI_Data_Delete del;
   
   framepoint=(unsigned char *)&framehead;
   
   framehead.SOF=UI_SOF;
   framehead.Data_Length=8;
   framehead.Seq=UI_Seq;
   framehead.CRC8=Get_CRC8_Check_Sum_UI(framepoint,4,0xFF);
   framehead.CMD_ID=UI_CMD_Robo_Exchange;                   //填充包头数据
   
   datahead.Data_ID=UI_Data_ID_Del;
	 /*SZL 6-16-2022 动态的收取 裁判系统ID*/
   datahead.Sender_ID=get_robot_id(); //Robot_ID;
	 switch(get_robot_id()){
		 case UI_Data_RobotID_RHero:
				datahead.Receiver_ID = UI_Data_CilentID_RHero;//为英雄操作手客户端(红)
				break;
			case UI_Data_RobotID_REngineer:
				datahead.Receiver_ID = UI_Data_CilentID_REngineer;
				break;
			case UI_Data_RobotID_RStandard1:
				datahead.Receiver_ID = UI_Data_CilentID_RStandard1;
				break;
			case UI_Data_RobotID_RStandard2:
				datahead.Receiver_ID = UI_Data_CilentID_RStandard2;
				break;
			case UI_Data_RobotID_RStandard3:
				datahead.Receiver_ID = UI_Data_CilentID_RStandard3;
				break;
			case UI_Data_RobotID_RAerial:
				datahead.Receiver_ID = UI_Data_CilentID_RAerial;
				break;
			
			case UI_Data_RobotID_BHero:
				datahead.Receiver_ID = UI_Data_CilentID_BHero;
				break;
			case UI_Data_RobotID_BEngineer:
				datahead.Receiver_ID = UI_Data_CilentID_BEngineer;
				break;
			case UI_Data_RobotID_BStandard1:
				datahead.Receiver_ID = UI_Data_CilentID_BStandard1;
				break;
			case UI_Data_RobotID_BStandard2:
				datahead.Receiver_ID = UI_Data_CilentID_BStandard2;
				break;
			case UI_Data_RobotID_BStandard3:
				datahead.Receiver_ID = UI_Data_CilentID_BStandard3;
				break;
			case UI_Data_RobotID_BAerial:
				datahead.Receiver_ID = UI_Data_CilentID_BAerial;
				break;
			default :
				datahead.Receiver_ID = Cilent_ID; //默认 无论怎样发一个出去
				datahead.Sender_ID = Robot_ID;
				break;
	 }//填充操作数据
   
   del.Delete_Operate=Del_Operate;
   del.Layer=Del_Layer;                                     //控制信息
   
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(framehead),frametail);
   framepoint=(unsigned char *)&datahead;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(datahead),frametail);
   framepoint=(unsigned char *)&del;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(del),frametail);  //CRC16校验值计算
   
   framepoint=(unsigned char *)&framehead;
   for(loop_control=0;loop_control<sizeof(framehead);loop_control++)
   {
      UI_SendByte(*framepoint);
      framepoint++;
   }
   framepoint=(unsigned char *)&datahead;
   for(loop_control=0;loop_control<sizeof(datahead);loop_control++)
   {
      UI_SendByte(*framepoint);
      framepoint++;
   }
   framepoint=(unsigned char *)&del;
   for(loop_control=0;loop_control<sizeof(del);loop_control++)
   {
      UI_SendByte(*framepoint);
      framepoint++;
   }                                                                 //发送所有帧
   framepoint=(unsigned char *)&frametail;
   for(loop_control=0;loop_control<sizeof(frametail);loop_control++)
   {
      UI_SendByte(*framepoint);
      framepoint++;                                                  //发送CRC16校验值
   }
   
   UI_Seq++;                                                         //包序号+1
}
/************************************************绘制直线*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Start_x、Start_x    开始坐标
        End_x、End_y   结束坐标
**********************************************************************************************************/
        
void Line_Draw(Graph_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Width,u32 Start_x,u32 Start_y,u32 End_x,u32 End_y)
{
   int i;
//   for(i=0;i<3&&imagename[i]!='\0';i++)
//      image->graphic_name[2-i]=imagename[i];
	
	 //SZL 6-8-2022修改 图形名按照写入数组的顺序存
	 for(i=0;i<3&&imagename[i]!='\0';i++)
      image->graphic_name[i]=imagename[i];
	
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->end_x = End_x;
   image->end_y = End_y;
}

/************************************************绘制矩形*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Start_x、Start_x    开始坐标
        End_x、End_y   结束坐标（对顶角坐标）
**********************************************************************************************************/
        
void Rectangle_Draw(Graph_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Width,u32 Start_x,u32 Start_y,u32 End_x,u32 End_y)
{
   int i;
//   for(i=0;i<3&&imagename[i]!=\0;i++)
//      image->graphic_name[2-i]=imagename[i];
	
	  //SZL 6-8-2022修改 图形名按照写入数组的顺序存
	 for(i=0;i<3&&imagename[i]!='\0';i++)
      image->graphic_name[i]=imagename[i];
	
   image->graphic_tpye = UI_Graph_Rectangle;
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->end_x = End_x;
   image->end_y = End_y;
}

/************************************************绘制整圆*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Start_x、Start_x    圆心坐标
        Graph_Radius  图形半径
**********************************************************************************************************/
        
void Circle_Draw(Graph_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Width,u32 Start_x,u32 Start_y,u32 Graph_Radius)
{
   int i;
//   for(i=0;i<3&&imagename[i]!=\0;i++)
//      image->graphic_name[2-i]=imagename[i];
	
	  //SZL 6-8-2022修改 图形名按照写入数组的顺序存
	 for(i=0;i<3&&imagename[i]!='\0';i++)
      image->graphic_name[i]=imagename[i];
	
   image->graphic_tpye = UI_Graph_Circle;
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->radius = Graph_Radius;
}

/************************************************绘制圆弧*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Graph_StartAngle,Graph_EndAngle    开始，终止角度
        Start_y,Start_y    圆心坐标
        x_Length,y_Length   x,y方向上轴长，参考椭圆
**********************************************************************************************************/
        
void Arc_Draw(Graph_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_StartAngle,u32 Graph_EndAngle,u32 Graph_Width,u32 Start_x,u32 Start_y,u32 x_Length,u32 y_Length)
{
   int i;
   
//   for(i=0;i<3&&imagename[i]!=\0;i++)
//      image->graphic_name[2-i]=imagename[i];
	
	  //SZL 6-8-2022修改 图形名按照写入数组的顺序存
	 for(i=0;i<3&&imagename[i]!='\0';i++)
      image->graphic_name[i]=imagename[i];
	
   image->graphic_tpye = UI_Graph_Arc;
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->start_angle = Graph_StartAngle;
   image->end_angle = Graph_EndAngle;
   image->end_x = x_Length;
   image->end_y = y_Length;
}



/************************************************绘制浮点型数据*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Graph_Size     字号
        Graph_Digit    小数位数
        Start_x、Start_x    开始坐标
        Graph_Float   要显示的变量
**********************************************************************************************************/
        
void Float_Draw(Float_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Size,u32 Graph_Digit,u32 Graph_Width,u32 Start_x,u32 Start_y,float Graph_Float)
{
   int i;
   
//   for(i=0;i<3&&imagename[i]!=\0;i++)
//      image->graphic_name[2-i]=imagename[i];
	
	  //SZL 6-8-2022修改 图形名按照写入数组的顺序存
	 for(i=0;i<3&&imagename[i]!='\0';i++)
      image->graphic_name[i]=imagename[i];
	
   image->graphic_tpye = UI_Graph_Float;
   image->operate_tpye = Graph_Operate;
   image->layer = Graph_Layer;
   image->color = Graph_Color;
   image->width = Graph_Width;
   image->start_x = Start_x;
   image->start_y = Start_y;
   image->start_angle = Graph_Size;
   image->end_angle = Graph_Digit;
   image->graph_Float = (int32_t)(1000*Graph_Float);
	//SZL 6-8-2022 修改float类型协议
}



/************************************************绘制字符型数据*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
        imagename[3]   图片名称，用于标识更改
        Graph_Operate   图片操作，见头文件
        Graph_Layer    图层0-9
        Graph_Color    图形颜色
        Graph_Width    图形线宽
        Graph_Size     字号
        Graph_Digit    字符个数
        Start_x、Start_x    开始坐标
        *Char_Data          待发送字符串开始地址
**********************************************************************************************************/
        
void Char_Draw(String_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Size,u32 Graph_Digit,u32 Graph_Width,u32 Start_x,u32 Start_y,char *Char_Data)
{
   int i;
   
//   for(i=0;i<3&&imagename[i]!=\0;i++)
//      image->Graph_Control.graphic_name[2-i]=imagename[i];
	
	  //SZL 6-8-2022修改 图形名按照写入数组的顺序存
	 for(i=0;i<3&&imagename[i]!='\0';i++)
      image->Graph_Control.graphic_name[i]=imagename[i];
	
   image->Graph_Control.graphic_tpye = UI_Graph_Char;
   image->Graph_Control.operate_tpye = Graph_Operate;
   image->Graph_Control.layer = Graph_Layer;
   image->Graph_Control.color = Graph_Color;
   image->Graph_Control.width = Graph_Width;
   image->Graph_Control.start_x = Start_x;
   image->Graph_Control.start_y = Start_y;
   image->Graph_Control.start_angle = Graph_Size;
   image->Graph_Control.end_angle = Graph_Digit;
   
   for(i=0;i<Graph_Digit;i++)
   {
      image->show_Data[i]=*Char_Data;
      Char_Data++;
   }
}

/************************************************UI推送函数（使更改生效）*********************************
**参数： cnt   图形个数
         ...   图形变量参数


Tips：：该函数只能推送1，2，5，7个图形，其他数目协议未涉及
**********************************************************************************************************/
int UI_ReFresh(int cnt,...)
{
   int i,n;
   Graph_Data imageData;
   unsigned char *framepoint;                      //读写指针
   u16 frametail=0xFFFF;                        //CRC16校验值
   
   UI_Packhead framehead;//frame_header(5-byte)+cmd_id(2-byte)
   UI_Data_Operate datahead;//交互数据包括了一个统一的数据段头结构: 内容ID 发送者 接收者的ID和内容数据段
   
   va_list ap;
   va_start(ap,cnt);
   
   framepoint=(unsigned char *)&framehead;
   framehead.SOF=UI_SOF;
   framehead.Data_Length=6+cnt*15;
   framehead.Seq=UI_Seq;
   framehead.CRC8=Get_CRC8_Check_Sum_UI(framepoint,4,0xFF);
   framehead.CMD_ID=UI_CMD_Robo_Exchange;                   //填充包头数据
   
   switch(cnt)
   {
      case 1:
         datahead.Data_ID=UI_Data_ID_Draw1;
         break;
      case 2:
         datahead.Data_ID=UI_Data_ID_Draw2;
         break;
      case 5:
         datahead.Data_ID=UI_Data_ID_Draw5;
         break;
      case 7:
         datahead.Data_ID=UI_Data_ID_Draw7;
         break;
      default:
         return (-1);
   }
	 
   /*SZL 6-16-2022 动态的收取 裁判系统ID*/
   datahead.Sender_ID=get_robot_id(); //Robot_ID;
	 switch(get_robot_id()){
		 case UI_Data_RobotID_RHero:
				datahead.Receiver_ID = UI_Data_CilentID_RHero;//为英雄操作手客户端(红)
				break;
			case UI_Data_RobotID_REngineer:
				datahead.Receiver_ID = UI_Data_CilentID_REngineer;
				break;
			case UI_Data_RobotID_RStandard1:
				datahead.Receiver_ID = UI_Data_CilentID_RStandard1;
				break;
			case UI_Data_RobotID_RStandard2:
				datahead.Receiver_ID = UI_Data_CilentID_RStandard2;
				break;
			case UI_Data_RobotID_RStandard3:
				datahead.Receiver_ID = UI_Data_CilentID_RStandard3;
				break;
			case UI_Data_RobotID_RAerial:
				datahead.Receiver_ID = UI_Data_CilentID_RAerial;
				break;
			
			case UI_Data_RobotID_BHero:
				datahead.Receiver_ID = UI_Data_CilentID_BHero;
				break;
			case UI_Data_RobotID_BEngineer:
				datahead.Receiver_ID = UI_Data_CilentID_BEngineer;
				break;
			case UI_Data_RobotID_BStandard1:
				datahead.Receiver_ID = UI_Data_CilentID_BStandard1;
				break;
			case UI_Data_RobotID_BStandard2:
				datahead.Receiver_ID = UI_Data_CilentID_BStandard2;
				break;
			case UI_Data_RobotID_BStandard3:
				datahead.Receiver_ID = UI_Data_CilentID_BStandard3;
				break;
			case UI_Data_RobotID_BAerial:
				datahead.Receiver_ID = UI_Data_CilentID_BAerial;
				break;
			default :
				datahead.Receiver_ID = Cilent_ID; //默认 无论怎样发一个出去
				datahead.Sender_ID = Robot_ID;
				break;
	 }//填充操作数据
   
   framepoint=(unsigned char *)&framehead;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(framehead),frametail);
   framepoint=(unsigned char *)&datahead;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(datahead),frametail);          //CRC16校验值计算（部分）
   
   framepoint=(unsigned char *)&framehead;
   for(i=0;i<sizeof(framehead);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;
   }
   framepoint=(unsigned char *)&datahead;
   for(i=0;i<sizeof(datahead);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;
   }
   
   for(i=0;i<cnt;i++)
   {
      imageData=va_arg(ap,Graph_Data);
      
      framepoint=(unsigned char *)&imageData;
      frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(imageData),frametail);             //CRC16校验
      
      for(n=0;n<sizeof(imageData);n++)
      {
         UI_SendByte(*framepoint);
         framepoint++;             
      } //发送图片帧
   }
   framepoint=(unsigned char *)&frametail;
   for(i=0;i<sizeof(frametail);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;                                                  //发送CRC16校验值
   }
   
   va_end(ap);
   
   UI_Seq++;                                                         //包序号+1
   return 0;
}


/************************************************UI推送字符（使更改生效）*********************************
**参数： cnt   图形个数
         ...   图形变量参数


Tips：：该函数只能推送1，2，5，7个图形，其他数目协议未涉及
**********************************************************************************************************/
int Char_ReFresh(String_Data string_Data)
{
   int i;
   String_Data imageData;
   unsigned char *framepoint;                      //读写指针
   u16 frametail=0xFFFF;                        //CRC16校验值
   
   UI_Packhead framehead;//frame_header(5-byte)+cmd_id(2-byte)
   UI_Data_Operate datahead;//交互数据包括了一个统一的数据段头结构: 内容ID 发送者 接收者的ID和内容数据段
   imageData=string_Data;
   
   
   framepoint=(unsigned char *)&framehead;
   framehead.SOF=UI_SOF;
   framehead.Data_Length=6+45;
   framehead.Seq=UI_Seq;
   framehead.CRC8=Get_CRC8_Check_Sum_UI(framepoint,4,0xFF);
   framehead.CMD_ID=UI_CMD_Robo_Exchange;                   //填充包头数据
   

   datahead.Data_ID=UI_Data_ID_DrawChar;//SZL 6-8-2022 修改

   /*SZL 6-16-2022 动态的收取 裁判系统ID*/
   datahead.Sender_ID=get_robot_id(); //Robot_ID;
	 switch(get_robot_id()){
		 case UI_Data_RobotID_RHero:
				datahead.Receiver_ID = UI_Data_CilentID_RHero;//为英雄操作手客户端(红)
				break;
			case UI_Data_RobotID_REngineer:
				datahead.Receiver_ID = UI_Data_CilentID_REngineer;
				break;
			case UI_Data_RobotID_RStandard1:
				datahead.Receiver_ID = UI_Data_CilentID_RStandard1;
				break;
			case UI_Data_RobotID_RStandard2:
				datahead.Receiver_ID = UI_Data_CilentID_RStandard2;
				break;
			case UI_Data_RobotID_RStandard3:
				datahead.Receiver_ID = UI_Data_CilentID_RStandard3;
				break;
			case UI_Data_RobotID_RAerial:
				datahead.Receiver_ID = UI_Data_CilentID_RAerial;
				break;
			
			case UI_Data_RobotID_BHero:
				datahead.Receiver_ID = UI_Data_CilentID_BHero;
				break;
			case UI_Data_RobotID_BEngineer:
				datahead.Receiver_ID = UI_Data_CilentID_BEngineer;
				break;
			case UI_Data_RobotID_BStandard1:
				datahead.Receiver_ID = UI_Data_CilentID_BStandard1;
				break;
			case UI_Data_RobotID_BStandard2:
				datahead.Receiver_ID = UI_Data_CilentID_BStandard2;
				break;
			case UI_Data_RobotID_BStandard3:
				datahead.Receiver_ID = UI_Data_CilentID_BStandard3;
				break;
			case UI_Data_RobotID_BAerial:
				datahead.Receiver_ID = UI_Data_CilentID_BAerial;
				break;
			default :
				datahead.Receiver_ID = Cilent_ID; //默认 无论怎样发一个出去
				datahead.Sender_ID = Robot_ID;
				break;
	 }//填充操作数据
   
   framepoint=(unsigned char *)&framehead;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(framehead),frametail);
   framepoint=(unsigned char *)&datahead;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(datahead),frametail);
   framepoint=(unsigned char *)&imageData;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(imageData),frametail);             //CRC16校验   //CRC16校验值计算（部分）
   
   framepoint=(unsigned char *)&framehead;
   for(i=0;i<sizeof(framehead);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;
   }
   framepoint=(unsigned char *)&datahead;
   for(i=0;i<sizeof(datahead);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;
   }                                                   //发送操作数据  
   framepoint=(unsigned char *)&imageData;
   for(i=0;i<sizeof(imageData);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;             
   }                                               //发送图片帧
   
   
   
   framepoint=(unsigned char *)&frametail;
   for(i=0;i<sizeof(frametail);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;                                                  //发送CRC16校验值
   }
   
   
   UI_Seq++;                                                         //包序号+1
   return 0;
}

/*

*/
int Delete_ReFresh(UI_Data_Delete delete_Data)
{
	 int i;
   UI_Data_Delete imageData;
   unsigned char *framepoint;                      //读写指针
   u16 frametail=0xFFFF;                        //CRC16校验值
   
   UI_Packhead framehead;//frame_header(5-byte)+cmd_id(2-byte)
   UI_Data_Operate datahead;//交互数据包括了一个统一的数据段头结构: 内容ID 发送者 接收者的ID和内容数据段
   imageData=delete_Data;
   
   
   framepoint=(unsigned char *)&framehead;
   framehead.SOF=UI_SOF;
   framehead.Data_Length=6+2;
   framehead.Seq=UI_Seq;
   framehead.CRC8=Get_CRC8_Check_Sum_UI(framepoint,4,0xFF);
   framehead.CMD_ID=UI_CMD_Robo_Exchange;                   //填充包头数据
   

   datahead.Data_ID=UI_Data_ID_Del;

   /*SZL 6-16-2022 动态的收取 裁判系统ID*/
   datahead.Sender_ID=get_robot_id(); //Robot_ID;
	 switch(get_robot_id()){
		 case UI_Data_RobotID_RHero:
				datahead.Receiver_ID = UI_Data_CilentID_RHero;//为英雄操作手客户端(红)
				break;
			case UI_Data_RobotID_REngineer:
				datahead.Receiver_ID = UI_Data_CilentID_REngineer;
				break;
			case UI_Data_RobotID_RStandard1:
				datahead.Receiver_ID = UI_Data_CilentID_RStandard1;
				break;
			case UI_Data_RobotID_RStandard2:
				datahead.Receiver_ID = UI_Data_CilentID_RStandard2;
				break;
			case UI_Data_RobotID_RStandard3:
				datahead.Receiver_ID = UI_Data_CilentID_RStandard3;
				break;
			case UI_Data_RobotID_RAerial:
				datahead.Receiver_ID = UI_Data_CilentID_RAerial;
				break;
			
			case UI_Data_RobotID_BHero:
				datahead.Receiver_ID = UI_Data_CilentID_BHero;
				break;
			case UI_Data_RobotID_BEngineer:
				datahead.Receiver_ID = UI_Data_CilentID_BEngineer;
				break;
			case UI_Data_RobotID_BStandard1:
				datahead.Receiver_ID = UI_Data_CilentID_BStandard1;
				break;
			case UI_Data_RobotID_BStandard2:
				datahead.Receiver_ID = UI_Data_CilentID_BStandard2;
				break;
			case UI_Data_RobotID_BStandard3:
				datahead.Receiver_ID = UI_Data_CilentID_BStandard3;
				break;
			case UI_Data_RobotID_BAerial:
				datahead.Receiver_ID = UI_Data_CilentID_BAerial;
				break;
			default :
				datahead.Receiver_ID = Cilent_ID; //默认 无论怎样发一个出去
				datahead.Sender_ID = Robot_ID;
				break;
	 }//填充操作数据
   
   framepoint=(unsigned char *)&framehead;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(framehead),frametail);
   framepoint=(unsigned char *)&datahead;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(datahead),frametail);
   framepoint=(unsigned char *)&imageData;
   frametail=Get_CRC16_Check_Sum_UI(framepoint,sizeof(imageData),frametail);             //CRC16校验   //CRC16校验值计算（部分）
   
   framepoint=(unsigned char *)&framehead;
   for(i=0;i<sizeof(framehead);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;
   }
   framepoint=(unsigned char *)&datahead;
   for(i=0;i<sizeof(datahead);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;
   }                                                   //发送操作数据  
   framepoint=(unsigned char *)&imageData;
   for(i=0;i<sizeof(imageData);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;             
   }                                               //发送图片帧
   

   framepoint=(unsigned char *)&frametail;
   for(i=0;i<sizeof(frametail);i++)
   {
      UI_SendByte(*framepoint);
      framepoint++;                                                  //发送CRC16校验值
   }
   
   
   UI_Seq++;                                                         //包序号+1
   return 0;
}

/*****************************************************CRC8校验值计算**********************************************/
const unsigned char CRC8_INIT_UI = 0xff; 
const unsigned char CRC8_TAB_UI[256] = 
{ 
	0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41, 
	0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc, 
	0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62, 
	0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff, 
	0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07, 
	0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a, 
	0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24, 
	0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9, 
	0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd, 
	0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50, 
	0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee, 
	0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73, 
	0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b, 
	0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16, 
	0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8, 
	0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35, 
};
unsigned char Get_CRC8_Check_Sum_UI(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8) 
{ 
	unsigned char ucIndex; 
	while (dwLength--) 
	{ 
		ucIndex = ucCRC8^(*pchMessage++); 
		ucCRC8 = CRC8_TAB_UI[ucIndex]; 
	} 
	return(ucCRC8); 
}

uint16_t CRC_INIT_UI = 0xffff; 
const uint16_t wCRC_Table_UI[256] = 
{ 
	0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 
	0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7, 
	0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e, 
	0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876, 
	0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd, 
	0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5, 
	0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 
	0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974, 
	0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb, 
	0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, 
	0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
	0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72, 
	0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 
	0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1, 
	0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738, 
	0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
	0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 
	0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff, 
	0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 
	0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 
	0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5, 
	0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 
	0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134, 
	0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c, 
	0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 
	0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb, 
	0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232, 
	0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, 
	0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1, 
	0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9, 
	0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 
	0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};
/* 
** Descriptions: CRC16 checksum function 
** Input: Data to check,Stream length, initialized checksum 
** Output: CRC checksum 
*/ 
uint16_t Get_CRC16_Check_Sum_UI(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC) 
{ 
	Uint8_t chData; 
	if (pchMessage == NULL) 
	{ 
		return 0xFFFF; 
	} 
	while(dwLength--) 
	{ 
		chData = *pchMessage++;
		(wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table_UI[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 
		0x00ff]; 
	} 
	return wCRC; 
}
