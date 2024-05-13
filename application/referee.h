#ifndef REFEREE_H
#define REFEREE_H

#include "main.h"

#include "protocol.h"

// 串口协议附录 https://www.robomaster.com/zh-CN/products/components/referee

typedef enum
{
    RED_HERO        = 1,
    RED_ENGINEER    = 2,
    RED_STANDARD_1  = 3,
    RED_STANDARD_2  = 4,
    RED_STANDARD_3  = 5,
    RED_AERIAL      = 6,
    RED_SENTRY      = 7,
		RED_DART				= 8,
		RED_RADAR 			= 9,
    BLUE_HERO       = 101,
    BLUE_ENGINEER   = 102,
    BLUE_STANDARD_1 = 103,
    BLUE_STANDARD_2 = 104,
    BLUE_STANDARD_3 = 105,
    BLUE_AERIAL     = 106,
    BLUE_SENTRY     = 107,
		BLUE_DART				= 108,
		BLUE_RADAR 			= 109,
} robot_id_t;

typedef enum
{
    OPERATOR_RED_HERO        = 0x0101,
    OPERATOR_RED_ENGINEER    = 0x0102,
    OPERATOR_RED_STANDARD_1  = 0x0103,
    OPERATOR_RED_STANDARD_2  = 0x0104,
    OPERATOR_RED_STANDARD_3  = 0x0105,
    OPERATOR_RED_AERIAL      = 0x0106,
    
		//
    OPERATOR_BLUE_HERO       = 0x0165,
    OPERATOR_BLUE_ENGINEER   = 0x0166,
    OPERATOR_BLUE_STANDARD_1 = 0x0167,
    OPERATOR_BLUE_STANDARD_2 = 0x0168,
    OPERATOR_BLUE_STANDARD_3 = 0x0169,
    OPERATOR_BLUE_AERIAL     = 0x016A,
    
} operator_robot_id_t;

typedef enum
{
    PROGRESS_UNSTART        = 0,
    PROGRESS_PREPARE        = 1,
    PROGRESS_SELFCHECK      = 2,
    PROGRESS_5sCOUNTDOWN    = 3,
    PROGRESS_BATTLE         = 4,
    PROGRESS_CALCULATING    = 5,
} game_progress_t;

typedef __packed struct // 0x0001
{
	uint8_t game_type : 4;
	uint8_t game_progress : 4;
	uint16_t stage_remain_time;
	uint64_t SyncTimeStamp;
}game_status_t;

typedef __packed struct // 0x0002
{
 uint8_t winner;
}game_result_t;

typedef __packed struct // 0x0003
{
	uint16_t red_1_robot_HP;
	uint16_t red_2_robot_HP;
	uint16_t red_3_robot_HP;
	uint16_t red_4_robot_HP;
	uint16_t red_5_robot_HP;
	uint16_t red_7_robot_HP; // sentry
	uint16_t red_outpost_HP;
	uint16_t red_base_HP;
	uint16_t blue_1_robot_HP;
	uint16_t blue_2_robot_HP;
	uint16_t blue_3_robot_HP; 
	uint16_t blue_4_robot_HP;
	uint16_t blue_5_robot_HP;
	uint16_t blue_7_robot_HP;
	uint16_t blue_outpost_HP;
	uint16_t blue_base_HP;
}game_robot_HP_t; 

/* 0x0005 deleted */

typedef __packed struct // 0x0101
{
	uint32_t event_data;
}event_data_t; 


typedef __packed struct // 0x0102
{
	uint8_t reserved;
	uint8_t supply_robot_id;
	uint8_t supply_projectile_step;
	uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

/* 0x0103 deleted */

typedef __packed struct // 0x0104
{
	uint8_t level;
	uint8_t offending_robot_id;
	uint8_t count;
}referee_warning_t; 

typedef __packed struct // 0x0105
{
	uint8_t dart_remaining_time;
	uint16_t dart_info;
}dart_info_t; 

typedef __packed struct // 0x0201
{
	uint8_t robot_id;
	uint8_t robot_level;
	uint16_t current_HP;
	uint16_t maximum_HP;
	uint16_t shooter_barrel_cooling_value;
	uint16_t shooter_barrel_heat_limit;
	uint16_t chassis_power_limit;
	uint8_t power_management_gimbal_output : 1;
	uint8_t power_management_chassis_output : 1;
	uint8_t power_management_shooter_output : 1;
}robot_status_t; 

/*
下面这个是判断枪口热量用了的:
原先的:
typedef __packed struct //0x0202
{
    uint16_t chassis_volt;
    uint16_t chassis_current;
    float chassis_power;
    uint16_t chassis_power_buffer;
    uint16_t shooter_heat0;
    uint16_t shooter_heat1;
} ext_power_heat_data_t;
shooter_heat0 对应 shooter_id1_17mm_cooling_heat
shooter_heat1 对应 shooter_id2_17mm_cooling_heat
*/
typedef __packed struct // 0x0202
{
 uint16_t chassis_voltage;
 uint16_t chassis_current;
 float chassis_power;
 uint16_t buffer_energy;
 uint16_t shooter_17mm_1_barrel_heat;
 uint16_t shooter_17mm_2_barrel_heat;
 uint16_t shooter_42mm_barrel_heat;
}power_heat_data_t; 


typedef __packed struct // 0x0203
{
 float x;
 float y;
 float angle;
}robot_pos_t; 

typedef __packed struct // 0x0204
{
 uint8_t recovery_buff;
 uint8_t cooling_buff;
 uint8_t defence_buff;
 uint8_t vulnerability_buff;
 uint16_t attack_buff;
}buff_t;

typedef __packed struct // 0x0205
{
 uint8_t airforce_status;
 uint8_t time_remain;
}air_support_data_t;

typedef __packed struct // 0x0206
{
 uint8_t armor_id : 4;
 uint8_t HP_deduction_reason : 4;
}hurt_data_t;

typedef __packed struct // 0x0207
{
 uint8_t bullet_type;
 uint8_t shooter_number;
 uint8_t launching_frequency;
 float initial_speed;
}shoot_data_t;

typedef __packed struct // 0x0208
{
 uint16_t projectile_allowance_17mm;
 uint16_t projectile_allowance_42mm;
 uint16_t remaining_gold_coin;
}projectile_allowance_t;

typedef __packed struct // 0x0209
{
 uint32_t rfid_status;
}rfid_status_t; 

typedef __packed struct // 0x020A
{
 uint8_t dart_launch_opening_status;
 uint8_t reserved;
 uint16_t target_change_time;
 uint16_t latest_launch_cmd_time;
}dart_client_cmd_t; 

////SZL改为串口协议附录V1.3l 新的交互数据接收信息
//typedef __packed struct //0x0301 old
//{
//    uint16_t data_cmd_id;
//		uint16_t sender_ID;
//		uint16_t receiver_ID;
//} ext_student_interactive_data_t;
typedef __packed struct //0x0301 新 V1.4
{
		uint16_t data_cmd_id;
		uint16_t sender_ID;
		uint16_t receiver_ID;
}ext_student_interactive_header_data_t;

////裁判系统串口协议附录V1.4 P26
//typedef __packed struct //自定义控制器交互数据 0x0302 30Hz  最大30字节
//{
//		uint8_t data[];
//} robot_interactive_data_t;

//typedef __packed struct //表 5-8 客户端绘制字符 机器人间通信： 0x0301 裁判系统串口协议附录V1.4 P25
//{
//		graphic_data_struct_t grapic_data_struct;
//		uint8_t data[30];
//} ext_client_custom_character_t;

// UI绘制 及发送的数据包 见UI相关.c .h文件

/*
很老的协议内容
typedef __packed struct
{
    float data1;
    float data2;
    float data3;
    uint8_t data4;
} custom_data_t;


typedef __packed struct
{
    uint8_t data[64];
} ext_up_stream_data_t;

typedef __packed struct
{
    uint8_t data[32];
} ext_download_stream_data_t;
*/

// 7. 小地图交互信息
/*客户端下发信息
小地图下发信息标识： 0x0303。发送频率：触发时发送*/
typedef __packed struct //0x0303
{
		float target_position_x;
		float target_position_y;
		float target_position_z;
		uint8_t commd_keyboard;
		uint16_t target_robot_ID;
} ext_robot_command_t;

/*客户端接收信息
小地图接收信息标识： 0x0305。 最大接收频率： 10Hz*/
typedef __packed struct
{
		uint16_t target_robot_ID;
		float target_position_x;
		float target_position_y;
} ext_client_map_command_t;

//8. 图传遥控信息 图传遥控信息，是通过图传模块下发
//图传遥控信息标识： 0x0304 30Hz P28
typedef __packed struct
{
		int16_t mouse_x;
		int16_t mouse_y;
		int16_t mouse_z;
		uint8_t left_button_down; //可能是int8_t
		uint8_t right_button_down; //可能是int8_t
		uint16_t keyboard_value;
		uint16_t reserved;
} ext_camRC_robot_command_t; //ext_robot_command_t同名

/*
上面的 7. 和 8. 暂时不使用 RMUL没有定位和小地图 图传遥控链路不使用
*/

extern void init_referee_struct_data(void);
extern void referee_data_solve(uint8_t *frame);

extern void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer);

extern uint8_t get_robot_id(void);
extern uint8_t get_robot_level(void);

extern void get_shooter_id1_17mm_heat_limit_and_heat(uint16_t *heat1_limit, uint16_t *heat1);
extern void get_shooter_id2_17mm_heat_limit_and_heat(uint16_t *heat1_limit, uint16_t *heat1);
extern uint16_t get_chassis_power_limit(void);
extern uint16_t get_shooter_id1_17mm_speed_limit(void);
extern uint16_t get_shooter_id2_17mm_speed_limit(void);

extern uint16_t get_shooter_id1_17mm_cd_rate(void);

extern uint32_t get_last_robot_state_rx_timestamp(void);

extern uint8_t get_chassis_power_output_status(void);

#endif
