#pragma once

#include "Motor_HAL_Interface.h"

// 获取 BSP 实现好的 HAL 接口指针 (供 Platform 层配置用)
const Lib_Motor::MotorHAL_t* BSP_Get_HAL_Impl(void);

// 启动底层硬件 (Kick-off)
// 负责开启中断、启动定时器计数，让 Tick 跑起来
void BSP_Motor_Hardware_Start(void);