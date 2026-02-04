#pragma once

#include <stdint.h>

namespace Lib_Motor
{
    // ================= 类型定义 (Type Definitions) =================
    // 这些只是“类型说明书”，告诉编译器这种函数指针长什么样

    // PWM 设置回调: (u, v, w) -> void
    using Motor_HAL_SetPWMCallback     = void (*)(float u, float v, float w);

    // 相电流读取回调: (ia, ib, ic 输出指针) -> void
    using Motor_HAL_ReadCurrentCallback = void (*)(uint16_t* ia, uint16_t* ib, uint16_t* ic, uint16_t* ibus);

    // 电压读取回调: void -> float
    using Motor_HAL_ReadVbusCallback   = uint16_t (*)(void);

    // 温度读取回调: void -> uint16_t
    using Motor_HAL_ReadTempCallback = uint16_t (*)(void);

    // 通用无参回调: void -> void
    using Motor_HAL_VoidCallback       = void (*)(void);

    /* ⭐ 硬件接口结构体 ⭐ */
    struct MotorHAL_t {
        // 1. PWM 控制
        Motor_HAL_SetPWMCallback      set_duty;      // 设置占空比
        Motor_HAL_VoidCallback        pwm_enable;    // 开启驱动
        Motor_HAL_VoidCallback        pwm_disable;   // 关闭驱动

        // 2. 采样
        Motor_HAL_ReadCurrentCallback read_currents_raw;        // 电流 (I-UVW BUS)
        Motor_HAL_ReadVbusCallback    read_vbus_raw;            // 母线电压
        Motor_HAL_ReadTempCallback    read_temp_raw;            // 控制器温度

        // 3. 系统/保护
        Motor_HAL_VoidCallback        enter_critical; // 关中断
        Motor_HAL_VoidCallback        exit_critical;  // 开中断
    };
}