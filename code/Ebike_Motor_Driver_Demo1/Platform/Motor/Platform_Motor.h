#pragma once

#include "Motor_API.h" // 包含 API 定义 (API_Motor)

/* C++ 全局对象暴露 */
extern Lib_Motor::MotorAPI Global_Motor_0;

/* 初始化函数 */
// 这个函数会在 Usermain.cpp 中调用
void Platform_Motor_Init(void);