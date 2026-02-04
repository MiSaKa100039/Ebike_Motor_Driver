#include "Motor_Control.h"
#include <math.h>

namespace Lib_Motor {

    // 实现保存指针
    void MotorControl::init(float ctrl_dt, const MotorConfig* cfg) {
        dt_ = ctrl_dt;
        cfg_ptr_ = cfg; // 关键：保存 Manager 里的 config 地址

        angle_gen_.reset();
        // PID 不需要 init，因为现在 update 时才传参
    }

    void MotorControl::reset() {
        angle_gen_.reset(); // 必须清零，否则下次启动会“飞”
        // pid_d_.reset();
        // pid_q_.reset();

    }

    // 1. 实现手动 PWM
    void MotorControl::processManualPWM(float u, float v, float w,
                                        float& out_da, float& out_db, float& out_dc)
    {
        // 简单的限幅保护
        out_da = MotorMath::Clamp(u, 0.0f, 1.0f);
        out_db = MotorMath::Clamp(v, 0.0f, 1.0f);
        out_dc = MotorMath::Clamp(w, 0.0f, 1.0f);
    }

    // V/F 核心算法实现
    void MotorControl::processVF(float target_rpm,
                                 float ramp_accel,
                                 float duty_bias,
                                 const MotorObserverParam& obs_params,
                                 float v_bus,
                                 // --- 输出引用 ---
                                 float& out_angle,
                                 float& out_speed_rpm,
                                 float& out_vd,
                                 float& out_vq,
                                 float& out_duty_a,
                                 float& out_duty_b,
                                 float& out_duty_c)
    {
        // 1. 角度生成 (Ramp + 积分)
        // 使用内部保存的 dt_
        out_angle = angle_gen_.update(target_rpm, dt_, ramp_accel);

        // 2. 速度回传 (用于监视)
        float current_rad_s = angle_gen_.getVelocity();
        out_speed_rpm = current_rad_s * 9.54929f; // rad/s -> RPM

        // 3. V/F 曲线计算电压
        float current_speed_abs = fabsf(out_speed_rpm);

        // A. 基础电压 (克服静摩擦)
        // 使用传入的 duty_bias (即 target_id)
        // 如果 API 传了 0，则使用 Config 里的默认值
        float start_duty = (duty_bias > 0.001f) ? duty_bias : obs_params.vf_start_voltage_ratio;
        float v_min = start_duty * v_bus;

        // B. 斜率计算
        float v_slope = obs_params.vf_curve_slope;
        float target_v = v_min + (current_speed_abs * v_slope);

        // C. 限幅
        if (target_v > v_bus) target_v = v_bus;

        // 4. 设定电压矢量 (Vd=0, Vq=Target)
        out_vd = 0.0f;
        out_vq = target_v;

        // 5. 反变换 (InvPark)
        float v_alpha, v_beta;
        MotorMath::InvPark(out_vd, out_vq, out_angle, &v_alpha, &v_beta);

        // 6. SVPWM 生成占空比
        SVPWM::Calculate(v_alpha, v_beta, v_bus, &out_duty_a, &out_duty_b, &out_duty_c);
    }


    // [修改] 增加 target_id 参数
    void MotorControl::processIF(float target_rpm,
                                 float ramp_accel,
                                 float target_id, // [新增] 允许设置 Id
                                 float target_iq, // 原有的 Iq
                                 float i_alpha, float i_beta,
                                 float v_bus,
                                 float& out_angle,
                                 float& out_speed_rpm,
                                 float& out_id, float& out_iq,
                                 float& out_vd, float& out_vq,
                                 float& out_duty_a, float& out_duty_b, float& out_duty_c)
    {
        // A. 生成虚拟角度 (开环)
        out_angle = angle_gen_.update(target_rpm, dt_, ramp_accel);

        // B. 速度监视
        out_speed_rpm = angle_gen_.getVelocity() * 9.54929f;

        // C. Park 变换 (计算反馈电流)
        float c = cosf(out_angle);
        float s = sinf(out_angle);
        out_id = i_alpha * c + i_beta * s;
        out_iq = -i_alpha * s + i_beta * c;

        // D. 进入电流闭环
        // [修改] 传入 target_id 而不是 0.0f
        processCurrentLoop(target_id, target_iq, out_angle,
                           out_id, out_iq, v_bus,
                           out_vd, out_vq,
                           out_duty_a, out_duty_b, out_duty_c);
    }

    // 3. 通用电流环 (给 Lock 和 IF 用)
    // processCurrentLoop 的完整实现
    void MotorControl::processCurrentLoop(float target_id, float target_iq,
                                          float angle,
                                          float i_d_meas, float i_q_meas,
                                          float v_bus,
                                          float& out_vd, float& out_vq,
                                          float& out_duty_a, float& out_duty_b, float& out_duty_c)
    {
        // 1. 安全检查：防止 init 没调导致指针为空
        if (cfg_ptr_ == nullptr) return;

        // 2. 通过指针“零拷贝”读取最新参数 (Ozone 改这里会立即生效)
        const auto& d_cfg = cfg_ptr_->control.current_d;
        const auto& q_cfg = cfg_ptr_->control.current_q;

        // 3. 计算误差
        float err_d = target_id - i_d_meas;
        float err_q = target_iq - i_q_meas;

        // 4. 运行 PID (参数从 Config 指针现取)
        out_vd = pid_d_.update(err_d, dt_, d_cfg.kp, d_cfg.ki, d_cfg.output_limit);
        out_vq = pid_q_.update(err_q, dt_, q_cfg.kp, q_cfg.ki, q_cfg.output_limit);

        // 5. 反变换 & SVPWM
        float v_alpha, v_beta;
        MotorMath::InvPark(out_vd, out_vq, angle, &v_alpha, &v_beta);
        SVPWM::Calculate(v_alpha, v_beta, v_bus, &out_duty_a, &out_duty_b, &out_duty_c);
    }
}