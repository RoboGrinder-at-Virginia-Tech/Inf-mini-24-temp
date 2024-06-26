/**
  ****************************(C) COPYRIGHT 2022 RoboGrinder at Virginia Tech****************************
  * @file       chassis_power_control.c/h
  * @brief      chassis power control.底盘功率控制
  * @note       Based on competition rules: chassis power control.
  *             Two ways to limit chassis power 
	*							1) The overall program structure is from DJI 2019 example code and file "chassis_power_control"
	*									based on remaining buffer # to regulate and to control the raw esc control current(message send to can bus)
	*             2) Speed adaptive chassis power control; the program will regulate and control the PID speed of each wheel first.
  *           
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. add chassis power control
	*  V2.0.0     July-20-2022    Zelin Shen      2. re-write basic chassis power control function; support dynamic charging power
	*																								 add speed adaptive chassis power control;
	*  V3.0.0     August-4-2022   Zelin Shen      3. add chassis power & energy control using superCap; It is a speed adaptive chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2022 RoboGrinder at Virginia Tech****************************
  */
#include "chassis_power_control.h"
#include "referee.h"
#include "arm_math.h"
#include "detect_task.h"
#include "SuperCap_comm.h"

#define CPC_MAX(a,b) ( ((a)>(b)) ? (a):(b) )
#define CPC_MIN(a,b) ( ((a)>(b)) ? (b):(a) )

extern  supercap_can_msg_id_e current_superCap;

void update_energy_buffer_debuff_total_current_limit(uint16_t chassis_p_lim, fp32* total_i_lim);
static void map_superCap_charge_pwr_to_debuff_total_current_limit(uint16_t charge_pwr, fp32* total_i_lim);

//调试用
uint8_t SZL_debug_place=4;
fp32 SZL_debug_chassis_power_buffer = 0;
//static uint8_t robot_id = 0 ; //test
//调试用END----

cpc_cap_energy_t cpc_cap_energy;
cpc_buffer_energy_t cpc_buffer_energy; //chassis_energy_control_direct_connect;

//regular power control global var - for debug
fp32 current_scale;

//speed adaptive power control global var - for debug
fp32 speed_adp_scale;

// 指用第二代超级电容架构的
void gen2_superCap_speed_adaptive_chassis_power_control(chassis_move_t *chassis_power_control)
{
		cpc_cap_energy.robot_id = get_robot_id();
	
		/*---更新一下需要用到的 动态变动的数据---*/
	  cpc_cap_energy.chassis_power_limit = get_chassis_power_limit();
		if(cpc_cap_energy.chassis_power_limit>MAX_REASONABLE_CHARGE_PWR) //( (chassis_e_ctrl.chassis_power_limit>MAX_REASONABLE_CHARGE_PWR) || (chassis_e_ctrl.chassis_power_limit <0) )
		{//识别 并处理 不合理数值
			cpc_cap_energy.chassis_power_limit = MAX_REASONABLE_CHARGE_PWR;
		}
		
		//从裁判系统获取当前缓冲能量
		cpc_get_chassis_power_and_buffer(&cpc_cap_energy.chassis_power, &cpc_cap_energy.chassis_power_buffer);//不重要
		//识别 并处理 chassis_power 和 chassis_power_buffer 不合理数值；--- SZL: 暂时不处理 --- 且这下数据对有超级电容来说 不是很重要
		
		//从 超级电容 获取当前剩余能量 获取当前使用的超级电容的剩余能量
	  cpc_get_superCap_vol_and_energy(&cpc_cap_energy.superCap_vol, &cpc_cap_energy.superCap_e_buffer);
		cpc_cap_energy.superCap_charge_pwr = (uint16_t)cpc_get_superCap_charge_pwr();
		
		//judge output cut-off point based on remaining energy and set the buffer ene critical val point
		/* VOL_OUTPUT_CUTOFF_POINT = 14.72f; MINIMUM_VOL=15.81f*/
		if(cpc_cap_energy.superCap_vol <= gen2Cap_VOL_OUTPUT_CUTOFF_POINT) //superCap_ENERGY_BUFF_OUTPUT_CUTOFF_POINT)
		{//一定产生cut off条件
			cpc_cap_energy.critical_val = gen2Cap_MINIMUM_VOL; //gen2Cap_MINIMUM_ENERGY_BUFF;
		}
		else if(cpc_cap_energy.superCap_vol >= gen2Cap_MINIMUM_VOL)
		{//一定关闭cut off条件
			cpc_cap_energy.critical_val = gen2Cap_VOL_OUTPUT_CUTOFF_POINT; //gen2Cap_ENERGY_BUFF_OUTPUT_CUTOFF_POINT;
		}
		else
		{// default sts
			cpc_cap_energy.critical_val = gen2Cap_VOL_OUTPUT_CUTOFF_POINT;
		}
		
		//分层 计算当前可用功率上限
		if(cpc_cap_energy.superCap_vol >= gen2Cap_WARNING_VOL)
		{//功率限制
//			cpc_buffer_energy.p_max = (fp32)(cpc_buffer_energy.chassis_power_buffer - MINIMUM_ENERGY_BUFF) / CHASSIS_REFEREE_COMM_TIME;
			cpc_cap_energy.p_max = gen2Cap_MAX_POWER_VALUE;
			
			cpc_cap_energy.p_max = fp32_constrain(cpc_cap_energy.p_max, INITIAL_STATE_CHASSIS_POWER_LIM, gen2Cap_MAX_POWER_VALUE);//最大功率的 限制
			//convert p_max to total_current_limit for esc raw values
		  cpc_cap_energy.total_current_limit = (fp32)cpc_cap_energy.p_max / 24.0f * 1000.0f;//* 1000.0f is to convert metric unit var to esc control raw value
		}
		else if(cpc_cap_energy.superCap_vol > gen2Cap_MINIMUM_VOL && cpc_cap_energy.superCap_vol < gen2Cap_WARNING_VOL)
		{//直接电流限制; 这样比较方便; 减缓
			/* 为了有更加平滑的能量控制 通过当前超级电容的充电功率 来映射出 当前debuff的最大功率
			*/
//			chassis_e_ctrl.p_max = 200.0f; //(fp32)(0.5f*6.0f*chassis_e_ctrl.superCap_vol*chassis_e_ctrl.superCap_vol - 0.5f*6.0f*superCap_WARNING_VOL*gen2Cap_WARNING_VOL) / CHASSIS_REFEREE_COMM_TIME;
//			chassis_e_ctrl.p_max = fp32_constrain(chassis_e_ctrl.p_max, INITIAL_STATE_CHASSIS_POWER_LIM, gen2Cap_MAX_POWER_VALUE);
//			
////			fp32 power_scale = chassis_e_ctrl.superCap_vol / gen2Cap_WARNING_VOL;
//			
//			chassis_e_ctrl.total_current_limit = (fp32)chassis_e_ctrl.p_max / 24.0f * 1000.0f;// * power_scale;
//			//反着更新 p_max
//			chassis_e_ctrl.p_max = chassis_e_ctrl.total_current_limit / 1000.0f * 24.0f;
			//-------------------------------------------
			
//			fp32 power_scale = chassis_e_ctrl.superCap_vol / gen2Cap_WARNING_VOL;
////			update_energy_buffer_debuff_total_current_limit(cpc_buffer_energy.chassis_power_limit, &cpc_buffer_energy.buffer_debuff_total_current_limit);
////			cpc_buffer_energy.total_current_limit = cpc_buffer_energy.buffer_debuff_total_current_limit * power_scale;
//			chassis_e_ctrl.total_current_limit = 16000.0f * power_scale; //16000.0f * power_scale;
//			
//			//反着更新 p_max
//			chassis_e_ctrl.p_max = chassis_e_ctrl.total_current_limit / 1000.0f * 24.0f;
			//8-4-2022 新方法---------------------------------------------------------------------------------------------------
			fp32 power_scale = cpc_cap_energy.superCap_vol / gen2Cap_WARNING_VOL;
			map_superCap_charge_pwr_to_debuff_total_current_limit(cpc_cap_energy.superCap_charge_pwr, &(cpc_cap_energy.buffer_debuff_total_current_limit));//fp32 buffer_debuff_total_current_limit;
			cpc_cap_energy.total_current_limit = cpc_cap_energy.buffer_debuff_total_current_limit * power_scale;
			//反着更新 p_max
			cpc_cap_energy.p_max = cpc_cap_energy.total_current_limit / 1000.0f * 24.0f;
		}
		else
		{//功率限制
			//缓冲能量达到或者小于危险值了, 保证当前底盘输出功率 小于等于 裁判系统的功率上限
			cpc_cap_energy.p_max = (fp32)cpc_cap_energy.superCap_charge_pwr - 4.0f; //(fp32)chassis_e_ctrl.chassis_power_limit - 4.0f;
			
			cpc_cap_energy.p_max = fp32_constrain(cpc_cap_energy.p_max, INITIAL_STATE_CHASSIS_POWER_LIM, gen2Cap_MAX_POWER_VALUE);//最大功率的 限制
			//convert p_max to total_current_limit for esc raw values
		  cpc_cap_energy.total_current_limit = (fp32)cpc_cap_energy.p_max / 24.0f * 1000.0f;//* 1000.0f is to convert metric unit var to esc control raw value
		}
		
//		//在这里实现 最大功率的 限制
//		if(fabs(cpc_buffer_energy.p_max) > MAX_POWER_VALUE)
//		{
//			cpc_buffer_energy.p_max = MAX_POWER_VALUE;
//		}
		
		/*---完成 动态变动的数据 的更新---*/
		
		/*先处理 裁判系统离线的情况---就只限制输出功率*/
		if(current_superCap_is_offline())
		{
			//就按找一个功率来限制就行了; 设备离线; 特殊情况下 的数据更新
			cpc_cap_energy.p_max = REFEREE_OFFLINE_POWER_LIM_VAL;
			cpc_cap_energy.p_max = fp32_constrain(cpc_cap_energy.p_max, INITIAL_STATE_CHASSIS_POWER_LIM, REFEREE_OFFLINE_POWER_LIM_VAL);
			cpc_cap_energy.total_current_limit = (fp32)cpc_cap_energy.p_max / 24.0f * 1000.0f;
			
			//calculate pid
			/*调试时发现的一个现象: PID算法; set=0; fdb=0; error=0; 由于I项, Iout=922; out=530.809
			  即total_current=1500~3000时 底盘 极低功率; 
			  测试数据: 裁判系统离线时如果 p_max = 100 -> total_current_limit=2083.3; total_current = 3630.90; 操作界面显示的chassis_power = 3.5w;
			
				(已用 分段常数控制器 解决)把机器人架起来, 摇杆向前推, 轮子向前空转到最大速度后; PID set 接近 fdb; 使得 out不高
			  即低total_current=1500~3000时 较高底盘功率出现;
				测试数据: 摇杆推到最前面; total_current = 1295.1; total_current_limit=20833.3(p_max=499.9); 数据包接收到的 chassis_power = 49.43w; 使用功率计测到的功率也差不多
				
				------ 第一个问题怎么解决呢? ------ 目前暂时把REFEREE_OFFLINE_POWER_LIM_VAL设高
			*/
			for (uint8_t i = 0; i < 4; i++)
			{
					PID_calc(&chassis_power_control->motor_speed_pid[i], chassis_power_control->motor_chassis[i].speed, chassis_power_control->motor_chassis[i].speed_set);
			}
				
			cpc_cap_energy.total_current = 0.0f;
			//calculate the original motor current set
			//计算原本电机电流设定
			for(uint8_t i = 0; i < 4; i++)
			{
					cpc_cap_energy.total_current += fabs(chassis_power_control->motor_speed_pid[i].out);
			}
			
			if(cpc_cap_energy.total_current > cpc_cap_energy.total_current_limit)
			{
				current_scale = cpc_cap_energy.total_current_limit / cpc_cap_energy.total_current;
				chassis_power_control->motor_speed_pid[0].out*=current_scale;
				chassis_power_control->motor_speed_pid[1].out*=current_scale;
				chassis_power_control->motor_speed_pid[2].out*=current_scale;
				chassis_power_control->motor_speed_pid[3].out*=current_scale;
			}
		}/*开始 分段常数控制器 + 速度自适应的功率控制*/
		else if(cpc_cap_energy.superCap_vol < cpc_cap_energy.critical_val)
		{//when below critical pt; just cut-off output
			chassis_power_control->motor_speed_pid[0].out = 0.0f;
			chassis_power_control->motor_speed_pid[1].out = 0.0f;
			chassis_power_control->motor_speed_pid[2].out = 0.0f;
			chassis_power_control->motor_speed_pid[3].out = 0.0f;
			
			cpc_cap_energy.ene_cutoff_sts = below_ENERGY_CRITICAL_POINT;//for debug
		}
		else
		{
			cpc_cap_energy.ene_cutoff_sts = above_ENERGY_CRITICAL_POINT;//for debug
			
			cpc_cap_energy.current_loop_cnt = 0;// init value
			while(1)
			{
				//calculate pid
				for (uint8_t i = 0; i < 4; i++)
				{
						PID_calc(&chassis_power_control->motor_speed_pid[i], chassis_power_control->motor_chassis[i].speed, chassis_power_control->motor_chassis[i].speed_set);
				}
				
				cpc_cap_energy.total_current = 0.0f;
				//calculate the original motor current set
				//计算原本电机电流设定
				for(uint8_t i = 0; i < 4; i++)
				{
						cpc_cap_energy.total_current += fabs(chassis_power_control->motor_speed_pid[i].out);
				}
				cpc_cap_energy.total_current_unit_amp = cpc_cap_energy.total_current / 1000.0f;//convert esc control value to unit amp current
				
				if(cpc_cap_energy.total_current > cpc_cap_energy.total_current_limit)//cpc_buffer_energy.total_current_unit_amp * 24.0f > cpc_buffer_energy.p_max)
				{
	//				  fp32 speed_adp_scale;
						
						cpc_cap_energy.current_loop_cnt++;
						if(cpc_cap_energy.current_loop_cnt >= 8)
						{
							//达到设定循环次数上限 直接削弱目标电流来保证
							current_scale = cpc_cap_energy.total_current_limit / cpc_cap_energy.total_current;
							chassis_power_control->motor_speed_pid[0].out*=current_scale;
							chassis_power_control->motor_speed_pid[1].out*=current_scale;
							chassis_power_control->motor_speed_pid[2].out*=current_scale;
							chassis_power_control->motor_speed_pid[3].out*=current_scale;

							cpc_cap_energy.adp_pwr_ctrl_result_status = adp_cpc_MAX_loop_cnt_reached;
							break;
						}
						else
						{
							//adapt speed
							speed_adp_scale = 0.99f; //cpc_buffer_energy.total_current_limit / cpc_buffer_energy.total_current; //cpc_buffer_energy.p_max / (cpc_buffer_energy.total_current_unit_amp * 24.0f);
							chassis_power_control->motor_chassis[0].speed_set *= speed_adp_scale;
							chassis_power_control->motor_chassis[1].speed_set *= speed_adp_scale;
							chassis_power_control->motor_chassis[2].speed_set *= speed_adp_scale;
							chassis_power_control->motor_chassis[3].speed_set *= speed_adp_scale;
						}
				}
				else
				{
					cpc_cap_energy.adp_pwr_ctrl_result_status = adp_cpc_NORMAL;
					break;
				}
			}
		}
		
		//values and FSM for debug regarding speed-adaptive power ctrl algorithm
		if(cpc_cap_energy.adp_pwr_ctrl_result_status == adp_cpc_MAX_loop_cnt_reached)
		{
			cpc_cap_energy.num_loop_limit_reached++;
		}
		else
		{
			if(cpc_cap_energy.current_loop_cnt != 0)
			{
				cpc_cap_energy.num_of_normal_loop++;
			}
			
			if(cpc_cap_energy.current_loop_cnt > cpc_cap_energy.max_speed_adp_loop_cnt)
			{
				cpc_cap_energy.max_speed_adp_loop_cnt = cpc_cap_energy.current_loop_cnt;
			}
		}
		
		//values for debug
		cpc_cap_energy.motor_final_current[0] = chassis_power_control->motor_speed_pid[0].out;
		cpc_cap_energy.motor_final_current[1] = chassis_power_control->motor_speed_pid[1].out;
		cpc_cap_energy.motor_final_current[2] = chassis_power_control->motor_speed_pid[2].out;
		cpc_cap_energy.motor_final_current[3] = chassis_power_control->motor_speed_pid[3].out;
		
		cpc_cap_energy.motor_final_total_current = 0;
		for(uint8_t i = 0; i < 4; i++)
		{
			cpc_cap_energy.motor_final_total_current += fabs(cpc_buffer_energy.motor_final_current[i]);
		}

}

// 第三代超级电容 仅有 裁判系统离线的情况下工况
void gen3_superCap_ref_sys_error_case_sacpc(chassis_move_t *chassis_power_control)
{
	/* 如果发生了裁判系统离线, 就盯着 第三代超级电容发聩的最大可补足电流来限制
	 为了避免不必要的刹车, 也没有底盘关断功能 */
	
	//fp32 current_scale;

	cpc_buffer_energy.robot_id = get_robot_id();

	/*---更新一下需要用到的 动态变动的数据---*/
	cpc_buffer_energy.chassis_power_limit = get_chassis_power_limit();
	if(cpc_buffer_energy.chassis_power_limit>MAX_REASONABLE_CHARGE_PWR) //( (cpc_buffer_energy.chassis_power_limit>MAX_REASONABLE_CHARGE_PWR) || (cpc_buffer_energy.chassis_power_limit <0) )
	{//识别 并处理 不合理数值
		cpc_buffer_energy.chassis_power_limit = MAX_REASONABLE_CHARGE_PWR;
	}
	
	//从裁判系统获取当前缓冲能量
	cpc_get_chassis_power_and_buffer(&cpc_buffer_energy.chassis_power, &cpc_buffer_energy.chassis_power_buffer);
	
	//识别 并处理 chassis_power 和 chassis_power_buffer 不合理数值；--- SZL: 暂时不处理 ---
	
	//judge output cut-off point based on remaining energy and set the buffer ene critical val point
	/* ENERGY_BUFF_OUTPUT_CUTOFF_POINT = 3, 6; MINIMUM_ENERGY_BUFF=10, 13*/
	if(cpc_buffer_energy.chassis_power_buffer <= ENERGY_BUFF_OUTPUT_CUTOFF_POINT)
	{//一定产生cut off条件
		cpc_buffer_energy.critical_power_buffer = MINIMUM_ENERGY_BUFF;
	}
	else if(cpc_buffer_energy.chassis_power_buffer >= MINIMUM_ENERGY_BUFF)
	{//一定关闭cut off条件
		cpc_buffer_energy.critical_power_buffer = ENERGY_BUFF_OUTPUT_CUTOFF_POINT;
	}
	else
	{// default sts
		cpc_buffer_energy.critical_power_buffer = ENERGY_BUFF_OUTPUT_CUTOFF_POINT;
	}
	
	//超级电容能量 按照gen3 超级电容反馈的当前最大能补足的功率上限
	// gen3cap_Pmax_spt 的一个估计值就是 Vbank * 15
	cpc_cap_energy.gen3cap_Pmax_spt = fp32_constrain(cpc_get_gen3Cap_Pmax(), cpc_buffer_energy.chassis_power_limit, 500.5f);
	cpc_cap_energy.gen3cap_spt_total_current_limit = (fp32)cpc_cap_energy.gen3cap_Pmax_spt / 24.0f * 1000.0f;//* 1000.0f is to convert metric unit var to esc control raw value
	
	//从 超级电容 获取当前剩余能量 获取当前使用的超级电容的剩余能量
	cpc_get_superCap_vol_and_energy(&cpc_cap_energy.superCap_vol, &cpc_cap_energy.superCap_e_buffer);
	cpc_cap_energy.superCap_charge_pwr = (uint16_t)cpc_get_superCap_charge_pwr();
	
	//根据超级电容电压计算 上限电流 并且综合 (gen3 超级电容反馈的当前最大能补足的功率上限 电流) => total_current_limit
	if(cpc_cap_energy.superCap_vol >= gen3Cap_WARNING_VOL)
	{//功率限制
		//cpc_buffer_energy.p_max = (fp32)(cpc_buffer_energy.chassis_power_buffer - MINIMUM_ENERGY_BUFF) / CHASSIS_REFEREE_COMM_TIME;
		cpc_cap_energy.p_max = gen3Cap_LARGE_POWER_VALUE;
		
		cpc_cap_energy.p_max = fp32_constrain(cpc_cap_energy.p_max, INITIAL_STATE_CHASSIS_POWER_LIM, gen3Cap_MAX_POWER_VALUE);//最大功率的 限制
		//convert p_max to total_current_limit for esc raw values
		cpc_cap_energy.cap_vol_cali_total_current_limit = (fp32)cpc_cap_energy.p_max / 24.0f * 1000.0f;//* 1000.0f is to convert metric unit var to esc control raw value
		
		// 此融合用 两者最小值 可修改
		cpc_cap_energy.total_current_limit = CPC_MIN(cpc_cap_energy.cap_vol_cali_total_current_limit, cpc_cap_energy.gen3cap_spt_total_current_limit);
	}
	else if(cpc_cap_energy.superCap_vol > gen3Cap_MINIMUM_VOL && cpc_cap_energy.superCap_vol < gen3Cap_WARNING_VOL)
	{//功率限制
		//cpc_buffer_energy.p_max = (fp32)(cpc_buffer_energy.chassis_power_buffer - MINIMUM_ENERGY_BUFF) / CHASSIS_REFEREE_COMM_TIME;
		cpc_cap_energy.p_max = gen3Cap_MEDIUM_POWER_VALUE;
		
		cpc_cap_energy.p_max = fp32_constrain(cpc_cap_energy.p_max, INITIAL_STATE_CHASSIS_POWER_LIM, gen3Cap_MAX_POWER_VALUE);//最大功率的 限制
		//convert p_max to total_current_limit for esc raw values
		cpc_cap_energy.cap_vol_cali_total_current_limit = (fp32)cpc_cap_energy.p_max / 24.0f * 1000.0f;//* 1000.0f is to convert metric unit var to esc control raw value
		
		// 此融合用 两者最小值 可修改
		cpc_cap_energy.total_current_limit = CPC_MIN(cpc_cap_energy.cap_vol_cali_total_current_limit, cpc_cap_energy.gen3cap_spt_total_current_limit);
	}
	else
	{//功率限制
		// 依然按照一个 功率限制来限幅 目前就按照超级电容下发功率来
		cpc_cap_energy.p_max = gen3Cap_SMALL_POWER_VALUE;
		//cpc_buffer_energy.p_max = (fp32)(cpc_buffer_energy.chassis_power_limit + 10.0f); // uncomment会用上最后一段缓冲能量
		
		cpc_cap_energy.p_max = fp32_constrain(cpc_cap_energy.p_max, INITIAL_STATE_CHASSIS_POWER_LIM, gen3Cap_MAX_POWER_VALUE);//最大功率的 限制
		//convert p_max to total_current_limit for esc raw values
		cpc_cap_energy.cap_vol_cali_total_current_limit = (fp32)cpc_cap_energy.p_max / 24.0f * 1000.0f;//* 1000.0f is to convert metric unit var to esc control raw value
		
		// 此融合用 两者最小值 可修改
		cpc_cap_energy.total_current_limit = CPC_MIN(cpc_cap_energy.cap_vol_cali_total_current_limit, cpc_cap_energy.gen3cap_spt_total_current_limit);
		//cpc_cap_energy.total_current_limit = cpc_cap_energy.cap_vol_cali_total_current_limit; // uncomment会用上最后一段缓冲能量
		
//		// 6-1-2024 经过测试 该debuff限幅过了
//		//缓冲能量达到或者小于危险值了, debuff电流上限
//		fp32 power_scale = cpc_cap_energy.superCap_vol / gen3Cap_WARNING_VOL;
//		map_superCap_charge_pwr_to_debuff_total_current_limit(cpc_cap_energy.superCap_charge_pwr, &(cpc_cap_energy.buffer_debuff_total_current_limit));//fp32 buffer_debuff_total_current_limit;
//		cpc_cap_energy.total_current_limit = cpc_cap_energy.buffer_debuff_total_current_limit * power_scale;
//		//反着更新 p_max
//		cpc_cap_energy.p_max = cpc_cap_energy.total_current_limit / 1000.0f * 24.0f;
		
//		// !!第二代超级电容功能!! 保证当前底盘输出功率 小于等于 裁判系统的功率上限
//		cpc_cap_energy.p_max = (fp32)cpc_cap_energy.superCap_charge_pwr - 4.0f; //(fp32)chassis_e_ctrl.chassis_power_limit - 4.0f;
//		
//		cpc_cap_energy.p_max = fp32_constrain(cpc_cap_energy.p_max, INITIAL_STATE_CHASSIS_POWER_LIM, gen3Cap_MAX_POWER_VALUE);//最大功率的 限制
//		//convert p_max to total_current_limit for esc raw values
//		cpc_cap_energy.total_current_limit = (fp32)cpc_cap_energy.p_max / 24.0f * 1000.0f;//* 1000.0f is to convert metric unit var to esc control raw value
	}
	
	// 开始不带分段常数控制器 底盘电流控制
	cpc_cap_energy.ene_cutoff_sts = above_ENERGY_CRITICAL_POINT;//for debug
	
	cpc_cap_energy.current_loop_cnt = 0;// init value
	while(1)
	{
		//calculate pid
		for (uint8_t i = 0; i < 4; i++)
		{
				PID_calc(&chassis_power_control->motor_speed_pid[i], chassis_power_control->motor_chassis[i].speed, chassis_power_control->motor_chassis[i].speed_set);
		}
		
		cpc_cap_energy.total_current = 0.0f;
		//calculate the original motor current set
		//计算原本电机电流设定
		for(uint8_t i = 0; i < 4; i++)
		{
				cpc_cap_energy.total_current += fabs(chassis_power_control->motor_speed_pid[i].out);
		}
		cpc_cap_energy.total_current_unit_amp = cpc_cap_energy.total_current / 1000.0f;//convert esc control value to unit amp current
		
		if(cpc_cap_energy.total_current > cpc_cap_energy.total_current_limit)//cpc_buffer_energy.total_current_unit_amp * 24.0f > cpc_buffer_energy.p_max)
		{
//				  fp32 speed_adp_scale;
				
				cpc_cap_energy.current_loop_cnt++;
				if(cpc_cap_energy.current_loop_cnt >= 8)
				{
					//达到设定循环次数上限 直接削弱目标电流来保证
					current_scale = cpc_cap_energy.total_current_limit / cpc_cap_energy.total_current;
					chassis_power_control->motor_speed_pid[0].out*=current_scale;
					chassis_power_control->motor_speed_pid[1].out*=current_scale;
					chassis_power_control->motor_speed_pid[2].out*=current_scale;
					chassis_power_control->motor_speed_pid[3].out*=current_scale;

					cpc_cap_energy.adp_pwr_ctrl_result_status = adp_cpc_MAX_loop_cnt_reached;
					break;
				}
				else
				{
					//adapt speed
					speed_adp_scale = 0.99f; //cpc_buffer_energy.total_current_limit / cpc_buffer_energy.total_current; //cpc_buffer_energy.p_max / (cpc_buffer_energy.total_current_unit_amp * 24.0f);
					chassis_power_control->motor_chassis[0].speed_set *= speed_adp_scale;
					chassis_power_control->motor_chassis[1].speed_set *= speed_adp_scale;
					chassis_power_control->motor_chassis[2].speed_set *= speed_adp_scale;
					chassis_power_control->motor_chassis[3].speed_set *= speed_adp_scale;
				}
		}
		else
		{
			cpc_cap_energy.adp_pwr_ctrl_result_status = adp_cpc_NORMAL;
			break;
		}
	}
		
	//values and FSM for debug regarding speed-adaptive power ctrl algorithm
	if(cpc_cap_energy.adp_pwr_ctrl_result_status == adp_cpc_MAX_loop_cnt_reached)
	{
		cpc_cap_energy.num_loop_limit_reached++;
	}
	else
	{
		if(cpc_cap_energy.current_loop_cnt != 0)
		{
			cpc_cap_energy.num_of_normal_loop++;
		}
		
		if(cpc_cap_energy.current_loop_cnt > cpc_cap_energy.max_speed_adp_loop_cnt)
		{
			cpc_cap_energy.max_speed_adp_loop_cnt = cpc_cap_energy.current_loop_cnt;
		}
	}
	
	//values for debug
	cpc_cap_energy.motor_final_current[0] = chassis_power_control->motor_speed_pid[0].out;
	cpc_cap_energy.motor_final_current[1] = chassis_power_control->motor_speed_pid[1].out;
	cpc_cap_energy.motor_final_current[2] = chassis_power_control->motor_speed_pid[2].out;
	cpc_cap_energy.motor_final_current[3] = chassis_power_control->motor_speed_pid[3].out;
	
	cpc_cap_energy.motor_final_total_current = 0;
	for(uint8_t i = 0; i < 4; i++)
	{
		cpc_cap_energy.motor_final_total_current += fabs(cpc_buffer_energy.motor_final_current[i]);
	}

}

// 第三代超级电容 正常工况下 底盘功率控制
void gen3_superCap_speed_adaptive_chassis_power_control(chassis_move_t *chassis_power_control)
{
	//fp32 current_scale;

	cpc_buffer_energy.robot_id = get_robot_id();

	/*---更新一下需要用到的 动态变动的数据---*/
	cpc_buffer_energy.chassis_power_limit = get_chassis_power_limit();
	if(cpc_buffer_energy.chassis_power_limit>MAX_REASONABLE_CHARGE_PWR) //( (cpc_buffer_energy.chassis_power_limit>MAX_REASONABLE_CHARGE_PWR) || (cpc_buffer_energy.chassis_power_limit <0) )
	{//识别 并处理 不合理数值
		cpc_buffer_energy.chassis_power_limit = MAX_REASONABLE_CHARGE_PWR;
	}
	
	//从裁判系统获取当前缓冲能量
	cpc_get_chassis_power_and_buffer(&cpc_buffer_energy.chassis_power, &cpc_buffer_energy.chassis_power_buffer);
	
	//识别 并处理 chassis_power 和 chassis_power_buffer 不合理数值；--- SZL: 暂时不处理 ---
	
	//judge output cut-off point based on remaining energy and set the buffer ene critical val point
	/* ENERGY_BUFF_OUTPUT_CUTOFF_POINT = 3, 6; MINIMUM_ENERGY_BUFF=10, 13*/
	if(cpc_buffer_energy.chassis_power_buffer <= ENERGY_BUFF_OUTPUT_CUTOFF_POINT)
	{//一定产生cut off条件
		cpc_buffer_energy.critical_power_buffer = MINIMUM_ENERGY_BUFF;
	}
	else if(cpc_buffer_energy.chassis_power_buffer >= MINIMUM_ENERGY_BUFF)
	{//一定关闭cut off条件
		cpc_buffer_energy.critical_power_buffer = ENERGY_BUFF_OUTPUT_CUTOFF_POINT;
	}
	else
	{// default sts
		cpc_buffer_energy.critical_power_buffer = ENERGY_BUFF_OUTPUT_CUTOFF_POINT;
	}
	
	//超级电容能量 按照gen3 超级电容反馈的当前最大能补足的功率上限
	// gen3cap_Pmax_spt 的一个估计值就是 Vbank * 15
	cpc_cap_energy.gen3cap_Pmax_spt = fp32_constrain(cpc_get_gen3Cap_Pmax(), cpc_buffer_energy.chassis_power_limit, 500.5f);
	cpc_cap_energy.gen3cap_spt_total_current_limit = (fp32)cpc_cap_energy.gen3cap_Pmax_spt / 24.0f * 1000.0f;//* 1000.0f is to convert metric unit var to esc control raw value
	
	//从 超级电容 获取当前剩余能量 获取当前使用的超级电容的剩余能量
	cpc_get_superCap_vol_and_energy(&cpc_cap_energy.superCap_vol, &cpc_cap_energy.superCap_e_buffer);
	cpc_cap_energy.superCap_charge_pwr = (uint16_t)cpc_get_superCap_charge_pwr();
	
	//根据超级电容电压计算 上限电流 并且综合 (gen3 超级电容反馈的当前最大能补足的功率上限 电流) => total_current_limit
	if(cpc_cap_energy.superCap_vol >= gen3Cap_WARNING_VOL)
	{//功率限制
		//cpc_buffer_energy.p_max = (fp32)(cpc_buffer_energy.chassis_power_buffer - MINIMUM_ENERGY_BUFF) / CHASSIS_REFEREE_COMM_TIME;
		cpc_cap_energy.p_max = gen3Cap_LARGE_POWER_VALUE;
		
		cpc_cap_energy.p_max = fp32_constrain(cpc_cap_energy.p_max, INITIAL_STATE_CHASSIS_POWER_LIM, gen3Cap_MAX_POWER_VALUE);//最大功率的 限制
		//convert p_max to total_current_limit for esc raw values
		cpc_cap_energy.cap_vol_cali_total_current_limit = (fp32)cpc_cap_energy.p_max / 24.0f * 1000.0f;//* 1000.0f is to convert metric unit var to esc control raw value
		
		// 此融合用 两者最大值 可修改
		cpc_cap_energy.total_current_limit = CPC_MAX(cpc_cap_energy.cap_vol_cali_total_current_limit, cpc_cap_energy.gen3cap_spt_total_current_limit);
	}
	else if(cpc_cap_energy.superCap_vol > gen3Cap_MINIMUM_VOL && cpc_cap_energy.superCap_vol < gen3Cap_WARNING_VOL)
	{//功率限制
		//cpc_buffer_energy.p_max = (fp32)(cpc_buffer_energy.chassis_power_buffer - MINIMUM_ENERGY_BUFF) / CHASSIS_REFEREE_COMM_TIME;
		cpc_cap_energy.p_max = gen3Cap_MEDIUM_POWER_VALUE;
		
		cpc_cap_energy.p_max = fp32_constrain(cpc_cap_energy.p_max, INITIAL_STATE_CHASSIS_POWER_LIM, gen3Cap_MAX_POWER_VALUE);//最大功率的 限制
		//convert p_max to total_current_limit for esc raw values
		cpc_cap_energy.cap_vol_cali_total_current_limit = (fp32)cpc_cap_energy.p_max / 24.0f * 1000.0f;//* 1000.0f is to convert metric unit var to esc control raw value
		
		// 此融合用 两者最小值 可修改
		cpc_cap_energy.total_current_limit = CPC_MIN(cpc_cap_energy.cap_vol_cali_total_current_limit, cpc_cap_energy.gen3cap_spt_total_current_limit);
	}
	else
	{//功率限制
		// 依然按照一个 功率限制来限幅 目前就按照超级电容下发功率来
		cpc_cap_energy.p_max = gen3Cap_SMALL_POWER_VALUE;
		//cpc_buffer_energy.p_max = (fp32)(cpc_buffer_energy.chassis_power_limit + 10.0f); // uncomment会用上最后一段缓冲能量
		
		cpc_cap_energy.p_max = fp32_constrain(cpc_cap_energy.p_max, INITIAL_STATE_CHASSIS_POWER_LIM, gen3Cap_MAX_POWER_VALUE);//最大功率的 限制
		//convert p_max to total_current_limit for esc raw values
		cpc_cap_energy.cap_vol_cali_total_current_limit = (fp32)cpc_cap_energy.p_max / 24.0f * 1000.0f;//* 1000.0f is to convert metric unit var to esc control raw value
		
		// 此融合用 两者最小值 可修改
		cpc_cap_energy.total_current_limit = CPC_MIN(cpc_cap_energy.cap_vol_cali_total_current_limit, cpc_cap_energy.gen3cap_spt_total_current_limit);
		//cpc_cap_energy.total_current_limit = cpc_cap_energy.cap_vol_cali_total_current_limit; // uncomment会用上最后一段缓冲能量
		
//		// 6-1-2024 经过测试 该debuff限幅过了
//		//缓冲能量达到或者小于危险值了, debuff电流上限
//		fp32 power_scale = cpc_cap_energy.superCap_vol / gen3Cap_WARNING_VOL;
//		map_superCap_charge_pwr_to_debuff_total_current_limit(cpc_cap_energy.superCap_charge_pwr, &(cpc_cap_energy.buffer_debuff_total_current_limit));//fp32 buffer_debuff_total_current_limit;
//		cpc_cap_energy.total_current_limit = cpc_cap_energy.buffer_debuff_total_current_limit * power_scale;
//		//反着更新 p_max
//		cpc_cap_energy.p_max = cpc_cap_energy.total_current_limit / 1000.0f * 24.0f;
		
//		// !!第二代超级电容功能!! 保证当前底盘输出功率 小于等于 裁判系统的功率上限
//		cpc_cap_energy.p_max = (fp32)cpc_cap_energy.superCap_charge_pwr - 4.0f; //(fp32)chassis_e_ctrl.chassis_power_limit - 4.0f;
//		
//		cpc_cap_energy.p_max = fp32_constrain(cpc_cap_energy.p_max, INITIAL_STATE_CHASSIS_POWER_LIM, gen3Cap_MAX_POWER_VALUE);//最大功率的 限制
//		//convert p_max to total_current_limit for esc raw values
//		cpc_cap_energy.total_current_limit = (fp32)cpc_cap_energy.p_max / 24.0f * 1000.0f;//* 1000.0f is to convert metric unit var to esc control raw value
	}
	
	//裁判系统缓冲能量: 分层 根据裁判系统缓冲能量 计算当前可用功率上限
	if(cpc_buffer_energy.chassis_power_buffer >= WARNING_ENERGY_BUFF)
	{
		//功率限制 老代码 
////			cpc_buffer_energy.p_max = (fp32)(cpc_buffer_energy.chassis_power_buffer - MINIMUM_ENERGY_BUFF) / CHASSIS_REFEREE_COMM_TIME;
//		cpc_buffer_energy.p_max = MAX_POWER_VALUE;
//		
//		cpc_buffer_energy.p_max = fp32_constrain(cpc_buffer_energy.p_max, INITIAL_STATE_CHASSIS_POWER_LIM, MAX_POWER_VALUE);//最大功率的 限制
//		//convert p_max to total_current_limit for esc raw values
//		cpc_buffer_energy.total_current_limit = (fp32)cpc_buffer_energy.p_max / 24.0f * 1000.0f;//* 1000.0f is to convert metric unit var to esc control raw value
		
		// 缓冲能量大时, 使用上述计算的电流限制
		cpc_buffer_energy.total_current_limit = cpc_cap_energy.total_current_limit;
	}
	else if(cpc_buffer_energy.chassis_power_buffer > MINIMUM_ENERGY_BUFF && cpc_buffer_energy.chassis_power_buffer < WARNING_ENERGY_BUFF)
	{//直接电流限制; 这样比较方便; 减缓
		fp32 power_scale = cpc_buffer_energy.chassis_power_buffer / WARNING_ENERGY_BUFF;
//			update_energy_buffer_debuff_total_current_limit(cpc_buffer_energy.chassis_power_limit, &cpc_buffer_energy.buffer_debuff_total_current_limit);
//			cpc_buffer_energy.total_current_limit = cpc_buffer_energy.buffer_debuff_total_current_limit * power_scale;
		cpc_buffer_energy.total_current_limit = 16000.0f * power_scale;
		
		//反着更新 p_max
		cpc_buffer_energy.p_max = cpc_buffer_energy.total_current_limit / 1000.0f * 24.0f;
	}
	else
	{//功率限制
		//缓冲能量达到或者小于危险值了, 保证当前底盘输出功率 小于等于 裁判系统的功率上限
		cpc_buffer_energy.p_max = (fp32)cpc_buffer_energy.chassis_power_limit;//-8.0f;
		
		cpc_buffer_energy.p_max = fp32_constrain(cpc_buffer_energy.p_max, INITIAL_STATE_CHASSIS_POWER_LIM, MAX_POWER_VALUE);//最大功率的 限制
		//convert p_max to total_current_limit for esc raw values
		cpc_buffer_energy.total_current_limit = (fp32)cpc_buffer_energy.p_max / 24.0f * 1000.0f;//* 1000.0f is to convert metric unit var to esc control raw value
	}
	
//		//在这里实现 最大功率的 限制
//		if(fabs(cpc_buffer_energy.p_max) > MAX_POWER_VALUE)
//		{
//			cpc_buffer_energy.p_max = MAX_POWER_VALUE;
//		}
	
	/*---完成 动态变动的数据 的更新---*/
	
	/*这里不处理 裁判系统离线的情况*/
	/*调试时发现的一个现象: PID算法; set=0; fdb=0; error=0; 由于I项, Iout=922; out=530.809
			即total_current=1500~3000时 底盘 极低功率; 
			测试数据: 裁判系统离线时如果 p_max = 100 -> total_current_limit=2083.3; total_current = 3630.90; 操作界面显示的chassis_power = 3.5w;
		
			(已用 分段常数控制器 解决)把机器人架起来, 摇杆向前推, 轮子向前空转到最大速度后; PID set 接近 fdb; 使得 out不高
			即低total_current=1500~3000时 较高底盘功率出现;
			测试数据: 摇杆推到最前面; total_current = 1295.1; total_current_limit=20833.3(p_max=499.9); 数据包接收到的 chassis_power = 49.43w; 使用功率计测到的功率也差不多
			这个问题在Hero上通过 分段常数控制器 已经解决了7-20之前测试都没问题
		
		7-20晚上: 第一次测试步兵的时候, 把步兵架在架子上, 未使用超级电容 轮子空转 向前全速跑 有时会出现超功率扣血, 可能是cut-off不及时; 后来安装了 裁判系统超级电容管理模块
		可是在7-21的相同测试中该问题并未复现
		
		
			------ 第一个问题怎么解决呢? ------ 目前暂时把REFEREE_OFFLINE_POWER_LIM_VAL设高
		*/
	
	/*开始 分段常数控制器 + 速度自适应的功率控制*/
	if(cpc_buffer_energy.chassis_power_buffer < cpc_buffer_energy.critical_power_buffer)
	{//when below critical pt; just cut-off output
		chassis_power_control->motor_speed_pid[0].out = 0.0f;
		chassis_power_control->motor_speed_pid[1].out = 0.0f;
		chassis_power_control->motor_speed_pid[2].out = 0.0f;
		chassis_power_control->motor_speed_pid[3].out = 0.0f;
		
		cpc_buffer_energy.ene_cutoff_sts = below_ENERGY_CRITICAL_POINT;//for debug
	}
	else
	{
		cpc_buffer_energy.ene_cutoff_sts = above_ENERGY_CRITICAL_POINT;//for debug
		
		cpc_buffer_energy.current_loop_cnt = 0;// init value
		while(1)
		{
			//calculate pid
			for (uint8_t i = 0; i < 4; i++)
			{
					PID_calc(&chassis_power_control->motor_speed_pid[i], chassis_power_control->motor_chassis[i].speed, chassis_power_control->motor_chassis[i].speed_set);
			}
			
			cpc_buffer_energy.total_current = 0.0f;
			//calculate the original motor current set
			//计算原本电机电流设定
			for(uint8_t i = 0; i < 4; i++)
			{
					cpc_buffer_energy.total_current += fabs(chassis_power_control->motor_speed_pid[i].out);
			}
			cpc_buffer_energy.total_current_unit_amp = cpc_buffer_energy.total_current / 1000.0f;//convert esc control value to unit amp current
			
			if(cpc_buffer_energy.total_current > cpc_buffer_energy.total_current_limit)//cpc_buffer_energy.total_current_unit_amp * 24.0f > cpc_buffer_energy.p_max)
			{
//				  fp32 speed_adp_scale;
					
					cpc_buffer_energy.current_loop_cnt++;
					if(cpc_buffer_energy.current_loop_cnt >= 8)
					{
						//达到设定循环次数上限 直接削弱目标电流来保证
						current_scale = cpc_buffer_energy.total_current_limit / cpc_buffer_energy.total_current;
						chassis_power_control->motor_speed_pid[0].out*=current_scale;
						chassis_power_control->motor_speed_pid[1].out*=current_scale;
						chassis_power_control->motor_speed_pid[2].out*=current_scale;
						chassis_power_control->motor_speed_pid[3].out*=current_scale;

						cpc_buffer_energy.adp_pwr_ctrl_result_status = adp_cpc_MAX_loop_cnt_reached;
						break;
					}
					else
					{
						//adapt speed
						speed_adp_scale = 0.99f; //cpc_buffer_energy.total_current_limit / cpc_buffer_energy.total_current; //cpc_buffer_energy.p_max / (cpc_buffer_energy.total_current_unit_amp * 24.0f);
						chassis_power_control->motor_chassis[0].speed_set *= speed_adp_scale;
						chassis_power_control->motor_chassis[1].speed_set *= speed_adp_scale;
						chassis_power_control->motor_chassis[2].speed_set *= speed_adp_scale;
						chassis_power_control->motor_chassis[3].speed_set *= speed_adp_scale;
					}
			}
			else
			{
				cpc_buffer_energy.adp_pwr_ctrl_result_status = adp_cpc_NORMAL;
				break;
			}
		}
	}
	
	//values and FSM for debug regarding speed-adaptive power ctrl algorithm
	if(cpc_buffer_energy.adp_pwr_ctrl_result_status == adp_cpc_MAX_loop_cnt_reached)
	{
		cpc_buffer_energy.num_loop_limit_reached++;
	}
	else
	{
		if(cpc_buffer_energy.current_loop_cnt != 0)
		{
			cpc_buffer_energy.num_of_normal_loop++;
		}
		
		if(cpc_buffer_energy.current_loop_cnt > cpc_buffer_energy.max_speed_adp_loop_cnt)
		{
			cpc_buffer_energy.max_speed_adp_loop_cnt = cpc_buffer_energy.current_loop_cnt;
		}
	}
	
	//values for debug
	cpc_buffer_energy.motor_final_current[0] = chassis_power_control->motor_speed_pid[0].out;
	cpc_buffer_energy.motor_final_current[1] = chassis_power_control->motor_speed_pid[1].out;
	cpc_buffer_energy.motor_final_current[2] = chassis_power_control->motor_speed_pid[2].out;
	cpc_buffer_energy.motor_final_current[3] = chassis_power_control->motor_speed_pid[3].out;
	
	cpc_buffer_energy.motor_final_total_current = 0;
	for(uint8_t i = 0; i < 4; i++)
	{
		cpc_buffer_energy.motor_final_total_current += fabs(cpc_buffer_energy.motor_final_current[i]);
	}
		
}

//调控速度; 速度自适应的 功率控制; 结合 分段常数控制器
void speed_adaptive_chassis_power_control(chassis_move_t *chassis_power_control)
{
	  //fp32 current_scale;
	
		cpc_buffer_energy.robot_id = get_robot_id();
	
		/*---更新一下需要用到的 动态变动的数据---*/
	  cpc_buffer_energy.chassis_power_limit = get_chassis_power_limit();
	  if(cpc_buffer_energy.chassis_power_limit>MAX_REASONABLE_CHARGE_PWR) //( (cpc_buffer_energy.chassis_power_limit>MAX_REASONABLE_CHARGE_PWR) || (cpc_buffer_energy.chassis_power_limit <0) )
		{//识别 并处理 不合理数值
			cpc_buffer_energy.chassis_power_limit = MAX_REASONABLE_CHARGE_PWR;
		}
		
		//从裁判系统获取当前缓冲能量
		cpc_get_chassis_power_and_buffer(&cpc_buffer_energy.chassis_power, &cpc_buffer_energy.chassis_power_buffer);
		
		//识别 并处理 chassis_power 和 chassis_power_buffer 不合理数值；--- SZL: 暂时不处理 ---
		
		//judge output cut-off point based on remaining energy and set the buffer ene critical val point
		/* ENERGY_BUFF_OUTPUT_CUTOFF_POINT = 3, 6; MINIMUM_ENERGY_BUFF=10, 13*/
		if(cpc_buffer_energy.chassis_power_buffer <= ENERGY_BUFF_OUTPUT_CUTOFF_POINT)
		{//一定产生cut off条件
			cpc_buffer_energy.critical_power_buffer = MINIMUM_ENERGY_BUFF;
		}
		else if(cpc_buffer_energy.chassis_power_buffer >= MINIMUM_ENERGY_BUFF)
		{//一定关闭cut off条件
			cpc_buffer_energy.critical_power_buffer = ENERGY_BUFF_OUTPUT_CUTOFF_POINT;
		}
		else
		{// default sts
			cpc_buffer_energy.critical_power_buffer = ENERGY_BUFF_OUTPUT_CUTOFF_POINT;
		}
		
		//分层 计算当前可用功率上限
		if(cpc_buffer_energy.chassis_power_buffer >= WARNING_ENERGY_BUFF)
		{//功率限制
//			cpc_buffer_energy.p_max = (fp32)(cpc_buffer_energy.chassis_power_buffer - MINIMUM_ENERGY_BUFF) / CHASSIS_REFEREE_COMM_TIME;
			cpc_buffer_energy.p_max = MAX_POWER_VALUE;
			
			cpc_buffer_energy.p_max = fp32_constrain(cpc_buffer_energy.p_max, INITIAL_STATE_CHASSIS_POWER_LIM, MAX_POWER_VALUE);//最大功率的 限制
			//convert p_max to total_current_limit for esc raw values
		  cpc_buffer_energy.total_current_limit = (fp32)cpc_buffer_energy.p_max / 24.0f * 1000.0f;//* 1000.0f is to convert metric unit var to esc control raw value
		}
		else if(cpc_buffer_energy.chassis_power_buffer > MINIMUM_ENERGY_BUFF && cpc_buffer_energy.chassis_power_buffer < WARNING_ENERGY_BUFF)
		{//直接电流限制; 这样比较方便; 减缓
			fp32 power_scale = cpc_buffer_energy.chassis_power_buffer / WARNING_ENERGY_BUFF;
//			update_energy_buffer_debuff_total_current_limit(cpc_buffer_energy.chassis_power_limit, &cpc_buffer_energy.buffer_debuff_total_current_limit);
//			cpc_buffer_energy.total_current_limit = cpc_buffer_energy.buffer_debuff_total_current_limit * power_scale;
			cpc_buffer_energy.total_current_limit = 16000.0f * power_scale;
			
			//反着更新 p_max
			cpc_buffer_energy.p_max = cpc_buffer_energy.total_current_limit / 1000.0f * 24.0f;
		}
		else
		{//功率限制
			//缓冲能量达到或者小于危险值了, 保证当前底盘输出功率 小于等于 裁判系统的功率上限
			cpc_buffer_energy.p_max = (fp32)cpc_buffer_energy.chassis_power_limit;//-8.0f;
			
			cpc_buffer_energy.p_max = fp32_constrain(cpc_buffer_energy.p_max, INITIAL_STATE_CHASSIS_POWER_LIM, MAX_POWER_VALUE);//最大功率的 限制
			//convert p_max to total_current_limit for esc raw values
		  cpc_buffer_energy.total_current_limit = (fp32)cpc_buffer_energy.p_max / 24.0f * 1000.0f;//* 1000.0f is to convert metric unit var to esc control raw value
		}
		
//		//在这里实现 最大功率的 限制
//		if(fabs(cpc_buffer_energy.p_max) > MAX_POWER_VALUE)
//		{
//			cpc_buffer_energy.p_max = MAX_POWER_VALUE;
//		}
		
		/*---完成 动态变动的数据 的更新---*/
		
		/*先处理 裁判系统离线的情况---就只限制输出功率*/
		if(toe_is_error(REFEREE_TOE))
		{
			//就按找一个功率来限制就行了; 设备离线; 特殊情况下 的数据更新
			cpc_buffer_energy.p_max = REFEREE_OFFLINE_POWER_LIM_VAL;
			cpc_buffer_energy.p_max = fp32_constrain(cpc_buffer_energy.p_max, INITIAL_STATE_CHASSIS_POWER_LIM, REFEREE_OFFLINE_POWER_LIM_VAL);
			cpc_buffer_energy.total_current_limit = (fp32)cpc_buffer_energy.p_max / 24.0f * 1000.0f;
			
			//calculate pid
			/*调试时发现的一个现象: PID算法; set=0; fdb=0; error=0; 由于I项, Iout=922; out=530.809
			  即total_current=1500~3000时 底盘 极低功率; 
			  测试数据: 裁判系统离线时如果 p_max = 100 -> total_current_limit=2083.3; total_current = 3630.90; 操作界面显示的chassis_power = 3.5w;
			
				(已用 分段常数控制器 解决)把机器人架起来, 摇杆向前推, 轮子向前空转到最大速度后; PID set 接近 fdb; 使得 out不高
			  即低total_current=1500~3000时 较高底盘功率出现;
				测试数据: 摇杆推到最前面; total_current = 1295.1; total_current_limit=20833.3(p_max=499.9); 数据包接收到的 chassis_power = 49.43w; 使用功率计测到的功率也差不多
			  这个问题在Hero上通过 分段常数控制器 已经解决了7-20之前测试都没问题
			
			7-20晚上: 第一次测试步兵的时候, 把步兵架在架子上, 未使用超级电容 轮子空转 向前全速跑 有时会出现超功率扣血, 可能是cut-off不及时; 后来安装了 裁判系统超级电容管理模块
			可是在7-21的相同测试中该问题并未复现
			
			
				------ 第一个问题怎么解决呢? ------ 目前暂时把REFEREE_OFFLINE_POWER_LIM_VAL设高
			*/
			for (uint8_t i = 0; i < 4; i++)
			{
					PID_calc(&chassis_power_control->motor_speed_pid[i], chassis_power_control->motor_chassis[i].speed, chassis_power_control->motor_chassis[i].speed_set);
			}
				
			cpc_buffer_energy.total_current = 0.0f;
			//calculate the original motor current set
			//计算原本电机电流设定
			for(uint8_t i = 0; i < 4; i++)
			{
					cpc_buffer_energy.total_current += fabs(chassis_power_control->motor_speed_pid[i].out);
			}
			
			if(cpc_buffer_energy.total_current > cpc_buffer_energy.total_current_limit)
			{
				current_scale = cpc_buffer_energy.total_current_limit / cpc_buffer_energy.total_current;
				chassis_power_control->motor_speed_pid[0].out*=current_scale;
				chassis_power_control->motor_speed_pid[1].out*=current_scale;
				chassis_power_control->motor_speed_pid[2].out*=current_scale;
				chassis_power_control->motor_speed_pid[3].out*=current_scale;
			}
		}/*开始 分段常数控制器 + 速度自适应的功率控制*/
		else if(cpc_buffer_energy.chassis_power_buffer < cpc_buffer_energy.critical_power_buffer)
		{//when below critical pt; just cut-off output
			chassis_power_control->motor_speed_pid[0].out = 0.0f;
			chassis_power_control->motor_speed_pid[1].out = 0.0f;
			chassis_power_control->motor_speed_pid[2].out = 0.0f;
			chassis_power_control->motor_speed_pid[3].out = 0.0f;
			
			cpc_buffer_energy.ene_cutoff_sts = below_ENERGY_CRITICAL_POINT;//for debug
		}
		else
		{
			cpc_buffer_energy.ene_cutoff_sts = above_ENERGY_CRITICAL_POINT;//for debug
			
			cpc_buffer_energy.current_loop_cnt = 0;// init value
			while(1)
			{
				//calculate pid
				for (uint8_t i = 0; i < 4; i++)
				{
						PID_calc(&chassis_power_control->motor_speed_pid[i], chassis_power_control->motor_chassis[i].speed, chassis_power_control->motor_chassis[i].speed_set);
				}
				
				cpc_buffer_energy.total_current = 0.0f;
				//calculate the original motor current set
				//计算原本电机电流设定
				for(uint8_t i = 0; i < 4; i++)
				{
						cpc_buffer_energy.total_current += fabs(chassis_power_control->motor_speed_pid[i].out);
				}
				cpc_buffer_energy.total_current_unit_amp = cpc_buffer_energy.total_current / 1000.0f;//convert esc control value to unit amp current
				
				if(cpc_buffer_energy.total_current > cpc_buffer_energy.total_current_limit)//cpc_buffer_energy.total_current_unit_amp * 24.0f > cpc_buffer_energy.p_max)
				{
	//				  fp32 speed_adp_scale;
						
						cpc_buffer_energy.current_loop_cnt++;
						if(cpc_buffer_energy.current_loop_cnt >= 8)
						{
							//达到设定循环次数上限 直接削弱目标电流来保证
							current_scale = cpc_buffer_energy.total_current_limit / cpc_buffer_energy.total_current;
							chassis_power_control->motor_speed_pid[0].out*=current_scale;
							chassis_power_control->motor_speed_pid[1].out*=current_scale;
							chassis_power_control->motor_speed_pid[2].out*=current_scale;
							chassis_power_control->motor_speed_pid[3].out*=current_scale;

							cpc_buffer_energy.adp_pwr_ctrl_result_status = adp_cpc_MAX_loop_cnt_reached;
							break;
						}
						else
						{
							//adapt speed
							speed_adp_scale = 0.99f; //cpc_buffer_energy.total_current_limit / cpc_buffer_energy.total_current; //cpc_buffer_energy.p_max / (cpc_buffer_energy.total_current_unit_amp * 24.0f);
							chassis_power_control->motor_chassis[0].speed_set *= speed_adp_scale;
							chassis_power_control->motor_chassis[1].speed_set *= speed_adp_scale;
							chassis_power_control->motor_chassis[2].speed_set *= speed_adp_scale;
							chassis_power_control->motor_chassis[3].speed_set *= speed_adp_scale;
						}
				}
				else
				{
					cpc_buffer_energy.adp_pwr_ctrl_result_status = adp_cpc_NORMAL;
					break;
				}
			}
		}
		
		//values and FSM for debug regarding speed-adaptive power ctrl algorithm
		if(cpc_buffer_energy.adp_pwr_ctrl_result_status == adp_cpc_MAX_loop_cnt_reached)
		{
			cpc_buffer_energy.num_loop_limit_reached++;
		}
		else
		{
			if(cpc_buffer_energy.current_loop_cnt != 0)
			{
				cpc_buffer_energy.num_of_normal_loop++;
			}
			
			if(cpc_buffer_energy.current_loop_cnt > cpc_buffer_energy.max_speed_adp_loop_cnt)
			{
				cpc_buffer_energy.max_speed_adp_loop_cnt = cpc_buffer_energy.current_loop_cnt;
			}
		}
		
		//values for debug
		cpc_buffer_energy.motor_final_current[0] = chassis_power_control->motor_speed_pid[0].out;
		cpc_buffer_energy.motor_final_current[1] = chassis_power_control->motor_speed_pid[1].out;
		cpc_buffer_energy.motor_final_current[2] = chassis_power_control->motor_speed_pid[2].out;
		cpc_buffer_energy.motor_final_current[3] = chassis_power_control->motor_speed_pid[3].out;
		
		cpc_buffer_energy.motor_final_total_current = 0;
		for(uint8_t i = 0; i < 4; i++)
		{
			cpc_buffer_energy.motor_final_total_current += fabs(cpc_buffer_energy.motor_final_current[i]);
		}
		
}

// 速度自适应 底盘功率控制 通用接口
void general_speed_adaptive_chassis_power_control(chassis_move_t *sacpc)
{
	// 判断是否是接入了第三代超级电容
	if(get_current_superCap() == gen3Cap_ID)
	{
		// 特殊1: 当超级电容收到了错误码时 error_proc会是的toe is err, 直接不用电容 -> 以下case包括了
		
		if(toe_is_error(GEN3CAP_TOE))
		{
			// cap 离线 或 发生error code时
			speed_adaptive_chassis_power_control(sacpc);
		}
		else
		{
			// cap 无错误:
			
			if(toe_is_error(REFEREE_TOE))
			{
				// ref 离线
				gen3_superCap_ref_sys_error_case_sacpc(sacpc);
			}
			else
			{
				// ref和cap都正常工作
				gen3_superCap_speed_adaptive_chassis_power_control(sacpc);
			}
		}
	}
	else if(get_current_superCap() == gen2Cap_ID || get_current_superCap() == ZiDaCap_ID || get_current_superCap() == WuLieCap_CAN_ID)
	{
		gen2_superCap_speed_adaptive_chassis_power_control(sacpc);
	}
	else
	{
		speed_adaptive_chassis_power_control(sacpc);
	}
}

static void map_superCap_charge_pwr_to_debuff_total_current_limit(uint16_t charge_pwr, fp32* total_i_lim)
{
#ifdef HERO_CHASSIS_POWER_CONTROL
		if(charge_pwr<=30)
		{
			*total_i_lim = 8333.33f;//200w
		}
		else if(charge_pwr>30 && charge_pwr<=50)
		{
			*total_i_lim = 10000.0f;//240w
		}
		else if(charge_pwr>50 && charge_pwr<=70)
		{
			*total_i_lim = 16000.0f;//384w
		}
		else if(charge_pwr>70 && charge_pwr<=90)
		{
			*total_i_lim = 16000.0f;
		}
		else if(charge_pwr>90 && charge_pwr<=120)
		{
			*total_i_lim = 16000.0f;
		}
		else
	  {//一个典型值 = 10000
			*total_i_lim = 10000.0f;
		}
#else
		if(charge_pwr<=30)
		{
			*total_i_lim = 13000.0f;//10000.0f; //8333.33f;//200w
		}
		else if(charge_pwr>30 && charge_pwr<=40)
		{
			*total_i_lim = 12000.0f; //13000.0f;//12000.0f;//240w
		}
		else if(charge_pwr>40 && charge_pwr<=50)
		{
			*total_i_lim = 13000.0f; //14500.0f;//13000.0f;//240w
		}
		else if(charge_pwr>50 && charge_pwr<=60)
		{
			*total_i_lim = 15000.0f; //16000.0f;//14500.0f;//240w
		}
		else if(charge_pwr>60 && charge_pwr<=80)
		{
			*total_i_lim = 17500.0f; //20000.0f;//384w
		}
		else if(charge_pwr>80 && charge_pwr<=100)
		{
			*total_i_lim = 20000.0f; //22000.0f;
		}
		else
	  {//一个典型值 = 10000
			*total_i_lim = 10000.0f;
		}
#endif
}

// this function is deprecated
void update_energy_buffer_debuff_total_current_limit(uint16_t chassis_p_lim, fp32* total_i_lim) //未使用
{
#ifdef HERO_CHASSIS_POWER_CONTROL
		/* Hero 通过标定 将功率上限 映射为 电机最大电流值; 对于debuff这一档 */
		//功率优先
		if(chassis_p_lim == 50)
		{
			*total_i_lim = 16000.0f;
		}
		else if(chassis_p_lim == 70)
		{
			*total_i_lim = 16000.0f;
		}
		else if(chassis_p_lim == 90)
		{
			*total_i_lim = 16000.0f;
		}
		else if(chassis_p_lim == 120)
		{
			*total_i_lim = 16000.0f;
		}
		else if(chassis_p_lim == 55) //血量优先
		{
			*total_i_lim = 16000.0f;
		}
		else if(chassis_p_lim == 60)
		{
			*total_i_lim = 16000.0f;
		}
		else if(chassis_p_lim == 65)
		{
			*total_i_lim = 16000.0f;
		}
		else
		{
			*total_i_lim = 16000.0f;
		}
#else
		/*Infantry 通过标定 将功率上限 映射为 电机最大电流值; 对于debuff这一档 */
		//功率优先
		if(chassis_p_lim == 40)
		{
			*total_i_lim = 16000.0f;
		}
		else if(chassis_p_lim == 60)
		{
			*total_i_lim = 16000.0f;
		}
		else if(chassis_p_lim == 80)
		{
			*total_i_lim = 16000.0f;
		}
		else if(chassis_p_lim == 100)
		{
			*total_i_lim = 16000.0f;
		}
		else if(chassis_p_lim == 45) //血量优先
		{
			*total_i_lim = 16000.0f;
		}
		else if(chassis_p_lim == 50)
		{
			*total_i_lim = 16000.0f;
		}
		else if(chassis_p_lim == 55)
		{
			*total_i_lim = 16000.0f;
		}
		else
		{
			*total_i_lim = 16000.0f;
		}
#endif
}

/* 非速度自适应的控制 不再使用*/
void chassis_power_control_non_speed(chassis_move_t *chassis_power_control)
{//非超级电容; 直连 功率闭环
//    fp32 chassis_power = 0.0f;
//    fp32 chassis_power_buffer = 0.0f;
//    fp32 total_current_limit = 0.0f;
//    fp32 total_current = 0.0f;
    cpc_buffer_energy.robot_id = get_robot_id();
		
	  /*--- 更新一下需要用到的 动态变动的数据 ---*/
	  cpc_buffer_energy.chassis_power_limit = get_chassis_power_limit();
	  if(cpc_buffer_energy.chassis_power_limit > MAX_REASONABLE_CHARGE_PWR)
		{//识别不合理数值
			cpc_buffer_energy.chassis_power_limit = MAX_REASONABLE_CHARGE_PWR;
		}
		//w=VI 将等级功率上限 映射为 电机最大电流值
		//功率优先
		if(cpc_buffer_energy.chassis_power_limit == 50)
		{
			cpc_buffer_energy.buffer_debuff_total_current_limit = 16000.0f;
			cpc_buffer_energy.buffer_minimum_total_current_limit = 2080.0f;
		}
		else if(cpc_buffer_energy.chassis_power_limit == 70)
		{
			cpc_buffer_energy.buffer_debuff_total_current_limit = 16000.0f;
			cpc_buffer_energy.buffer_minimum_total_current_limit = 2916.0f;
		}
		else if(cpc_buffer_energy.chassis_power_limit == 90)
		{
			cpc_buffer_energy.buffer_debuff_total_current_limit = 16000.0f;
			cpc_buffer_energy.buffer_minimum_total_current_limit = 3750.0f;
		}
		else if(cpc_buffer_energy.chassis_power_limit == 120)
		{
			cpc_buffer_energy.buffer_debuff_total_current_limit = 16000.0f;
			cpc_buffer_energy.buffer_minimum_total_current_limit = 5000.0f;
		}
		else if(cpc_buffer_energy.chassis_power_limit == 55) //血量优先
		{
			cpc_buffer_energy.buffer_debuff_total_current_limit = 16000.0f;
			cpc_buffer_energy.buffer_minimum_total_current_limit = 2291.6f;
		}
		else if(cpc_buffer_energy.chassis_power_limit == 60)
		{
			cpc_buffer_energy.buffer_debuff_total_current_limit = 16000.0f;
			cpc_buffer_energy.buffer_minimum_total_current_limit = 2500.0f;
		}
		else if(cpc_buffer_energy.chassis_power_limit == 65)
		{
			cpc_buffer_energy.buffer_debuff_total_current_limit = 16000.0f;
			cpc_buffer_energy.buffer_minimum_total_current_limit = 2708.0f;
		}
		else
		{
			 cpc_buffer_energy.chassis_power_limit = 50;
			 cpc_buffer_energy.buffer_debuff_total_current_limit = 16000.0f;
			 cpc_buffer_energy.buffer_minimum_total_current_limit = 2080.0f;
		}
		
		//从裁判系统获取当前缓冲能量
		cpc_get_chassis_power_and_buffer(&cpc_buffer_energy.chassis_power, &cpc_buffer_energy.chassis_power_buffer);
		
		//识别 并处理 chassis_power 和 chassis_power_buffer 不合理数值；--- SZL: 暂时不处理 ---
		
		//judge output cut-off point based on remaining energy and set the buffer ene critical val point
		/* ENERGY_BUFF_OUTPUT_CUTOFF_POINT = 3; MINIMUM_ENERGY_BUFF=10*/
		if(cpc_buffer_energy.chassis_power_buffer <= ENERGY_BUFF_OUTPUT_CUTOFF_POINT)
		{//一定产生cut off条件
			cpc_buffer_energy.critical_power_buffer = MINIMUM_ENERGY_BUFF;
		}
		else if(cpc_buffer_energy.chassis_power_buffer >= MINIMUM_ENERGY_BUFF)
		{//一定关闭cut off条件
			cpc_buffer_energy.critical_power_buffer = ENERGY_BUFF_OUTPUT_CUTOFF_POINT;
		}
		else
		{// default sts
			cpc_buffer_energy.critical_power_buffer = ENERGY_BUFF_OUTPUT_CUTOFF_POINT;
		}
		/*--- 完成动态数据的更新 ---*/
		
		
		/*开始 分段常数控制器 + 底盘功率控制*/
		if(cpc_buffer_energy.chassis_power_buffer < cpc_buffer_energy.critical_power_buffer)
		{//when below critical pt; just cut-off output
			chassis_power_control->motor_speed_pid[0].out = 0.0f;
			chassis_power_control->motor_speed_pid[1].out = 0.0f;
			chassis_power_control->motor_speed_pid[2].out = 0.0f;
			chassis_power_control->motor_speed_pid[3].out = 0.0f;
			
			cpc_buffer_energy.ene_cutoff_sts = below_ENERGY_CRITICAL_POINT;//for debug
		}
		else
		{
			if(toe_is_error(REFEREE_TOE))
			{
					cpc_buffer_energy.total_current_limit = NO_JUDGE_TOTAL_CURRENT_LIMIT;
			}
			else if(cpc_buffer_energy.robot_id == RED_ENGINEER || cpc_buffer_energy.robot_id == BLUE_ENGINEER || cpc_buffer_energy.robot_id == 0)
			{
					cpc_buffer_energy.total_current_limit = NO_JUDGE_TOTAL_CURRENT_LIMIT;
			}
			else
			{
	//        cpc_get_chassis_power_and_buffer(&cpc_buffer_energy.chassis_power, &cpc_buffer_energy.chassis_power_buffer);
					// power > 80w and buffer < 60j, because buffer < 60 means power has been more than 80w
					//功率超过80w 和缓冲能量小于60j,因为缓冲能量小于60意味着功率超过80w
					/* 举例 步兵机器人或英雄机器人未触发飞坡增益时，缓冲能量上限为 60J
						缓冲能量小于60j, 说明当前功率超过 当前的功率等级上限
						缓存能量剩50j时, 不管; 当缓冲能量小于40j时, 开始管 进入这个 if
					*/
					if(cpc_buffer_energy.chassis_power_buffer < WARNING_POWER_BUFF)
					{
							fp32 power_scale;
							if(cpc_buffer_energy.chassis_power_buffer > 10.0f)
							{
									//scale down WARNING_POWER_BUFF
									//缩小WARNING_POWER_BUFF
									//SZL: 10<chassis_power_buffer<(WARNING_POWER_BUFF=40)
									power_scale = cpc_buffer_energy.chassis_power_buffer / WARNING_POWER_BUFF;
									cpc_buffer_energy.total_current_limit = cpc_buffer_energy.buffer_debuff_total_current_limit * power_scale;
									SZL_debug_place = 1;
							}
							else
							{
									//only left 10% of WARNING_POWER_BUFF//小于5的时候都用5来限制幅度
									//power_scale = 5.0f / WARNING_POWER_BUFF;
									cpc_buffer_energy.total_current_limit = cpc_buffer_energy.buffer_minimum_total_current_limit;
									SZL_debug_place = 2;
							}
							/*scale down 缩小 SZL 7-15-2022 修改
								根据当前等级 允许的充电功率 来限制, 即<=> 当前充电功率所对应的电机电流数值 * power_scale
							*/
							//cpc_buffer_energy.total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT * power_scale;
							//SZL_debug_chassis_power_buffer = chassis_power_buffer;//SZL 后来加的
					}
					else
					{
							/*power > WARNING_POWER 功率大于WARNING_POWER; WARNING_POWER=400; POWER_LIMIT=500
								
							*/
							if(cpc_buffer_energy.chassis_power > WARNING_POWER)
							{
									fp32 power_scale;
									if(cpc_buffer_energy.chassis_power < POWER_LIMIT)
									{
											/*scale down;
												WARNING_POWER=400 < chassis_power < POWER_LIMIT=500
											*/
											power_scale = (POWER_LIMIT - cpc_buffer_energy.chassis_power) / (POWER_LIMIT - WARNING_POWER);
											SZL_debug_place = 3;
									}
									else
									{
											//chassis_power > POWER_LIMIT=500
											power_scale = 0.0f;
											SZL_debug_place = 4;
									}
									//total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT * power_scale;
									cpc_buffer_energy.total_current_limit = POWER_TOTAL_CURRENT_LIMIT_WHEN_NO_BUFF_USED * power_scale;
							}
							//power < WARNING_POWER
							//功率小于WARNING_POWER
							else
							{
									//total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT;
									cpc_buffer_energy.total_current_limit = POWER_TOTAL_CURRENT_LIMIT_WHEN_NO_BUFF_USED;
									SZL_debug_place = 5;
							}
	//              cpc_buffer_energy.total_current_limit = 64000.0f; //POWER_TOTAL_CURRENT_LIMIT_WHEN_NO_BUFF_USED;
					}
			}

			
			cpc_buffer_energy.total_current = 0.0f;
			//calculate the original motor current set
			//计算原本电机电流设定
			for(uint8_t i = 0; i < 4; i++)
			{
					cpc_buffer_energy.total_current += fabs(chassis_power_control->motor_speed_pid[i].out);
			}
			

			if(cpc_buffer_energy.total_current > cpc_buffer_energy.total_current_limit)
			{
					//fp32 current_scale = cpc_buffer_energy.total_current_limit / cpc_buffer_energy.total_current;
					current_scale = cpc_buffer_energy.total_current_limit / cpc_buffer_energy.total_current;
					chassis_power_control->motor_speed_pid[0].out*=current_scale;
					chassis_power_control->motor_speed_pid[1].out*=current_scale;
					chassis_power_control->motor_speed_pid[2].out*=current_scale;
					chassis_power_control->motor_speed_pid[3].out*=current_scale;
			}
		}
		
		//values for debug
		cpc_buffer_energy.motor_final_current[0] = chassis_power_control->motor_speed_pid[0].out;
		cpc_buffer_energy.motor_final_current[1] = chassis_power_control->motor_speed_pid[1].out;
		cpc_buffer_energy.motor_final_current[2] = chassis_power_control->motor_speed_pid[2].out;
		cpc_buffer_energy.motor_final_current[3] = chassis_power_control->motor_speed_pid[3].out;
		
		cpc_buffer_energy.motor_final_total_current = 0;
		for(uint8_t i = 0; i < 4; i++)
		{
				cpc_buffer_energy.motor_final_total_current += fabs(cpc_buffer_energy.motor_final_current[i]);
		}

}
