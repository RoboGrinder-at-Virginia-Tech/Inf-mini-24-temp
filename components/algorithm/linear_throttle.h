#ifndef LINEAR_THROTTLE_H
#define LINEAR_THROTTLE_H

#include "struct_typedef.h"


#pragma pack(1) // 压缩结构体,取消字节对齐

typedef struct
{
	fp32 abs_step;        // 输入数据 每秒希望增长数值 - 油门步进
	fp32 abs_init;				// 限幅最小值, 降低加速迟滞 - 初始加速值
  fp32 abs_target;      // 限幅最大值, 最终速度 - 全油门速度
	
  fp32 frame_period; //时间间隔
	
	fp32 out;
	
} linear_throttle_t;

//斜波函数初始化
extern void ramp_init(ramp_function_source_t *ramp_source_type, fp32 frame_period, fp32 max, fp32 min);

//斜波函数计算
extern void ramp_calc(ramp_function_source_t *ramp_source_type, fp32 input);

#endif
