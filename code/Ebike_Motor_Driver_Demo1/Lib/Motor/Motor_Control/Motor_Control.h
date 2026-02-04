#pragma once

#include "../Motor_Public/Motor_Config.h"

#include "FOCMath.h"
#include "SVPWM.h"

#include "Utils/AngleGenerator.h"         // 获取角度生成器定义
#include "Utils/PidController.h"

namespace Lib_Motor {

    /**
     * @brief 电机核心算法控制器 (纯数学层)
     * @details 负责执行具体的控制算法 (VF, FOC, PID等)。
     * 它不直接持有 Manager 的 ctx 数据，而是通过参数传入。
     */
    class MotorControl {
    public:
        void init(float ctrl_dt, const MotorConfig* cfg);
        void reset();

        // [新增] 1. 手动 PWM 透传
        void processManualPWM(float u, float v, float w, float& out_da, float& out_db, float& out_dc);

        void processVF(float target_rpm,
                       float ramp_accel,
                       float duty_bias,
                       const MotorObserverParam& obs_params,
                       float v_bus,
                       float& out_angle,
                       float& out_speed_rpm,
                       float& out_vd,
                       float& out_vq,
                       float& out_duty_a,
                       float& out_duty_b,
                       float& out_duty_c);


        void processIF(float target_rpm,
                                     float ramp_accel,
                                     float target_id, // [新增] 允许设置 Id
                                     float target_iq, // 原有的 Iq
                                     float i_alpha, float i_beta,
                                     float v_bus,
                                     float& out_angle,
                                     float& out_speed_rpm,
                                     float& out_id, float& out_iq,
                                     float& out_vd, float& out_vq,
                                     float& out_duty_a, float& out_duty_b, float& out_duty_c);

        // [新增] 电流闭环核心处理函数 (用于 Lock 和 IF)
        // 这是一个通用的 FOC 电流环函数，输入是“想要的角度”和“想要的电流”
        void processCurrentLoop(float target_id, float target_iq, // 目标电流
                                float target_angle,               // 目标角度 (Lock是0，IF是生成的)
                                float i_d_meas, float i_q_meas,   // 实际电流反馈
                                float v_bus,                      // 母线电压
                                // 输出
                                float& out_vd, float& out_vq,
                                float& out_duty_a, float& out_duty_b, float& out_duty_c);

    private:
        float dt_ = 0.0001f;       // 控制周期

        // [新增] 保存配置文件的指针
        const MotorConfig* cfg_ptr_ = nullptr;

        AngleGenerator angle_gen_; // 角度生成器实例
        
        PidController pid_d_;
        PidController pid_q_;
    };
}