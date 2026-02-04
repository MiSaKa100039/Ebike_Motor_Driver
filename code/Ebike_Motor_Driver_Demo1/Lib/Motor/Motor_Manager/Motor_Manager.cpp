#include "Motor_Manage.h"
#include "Motor_FSM.h"

#include <math.h>

#include "../Motor_Control/FocMath.h"
#include "../Motor_Control/SVPWM.h"
#include "../Motor_Control/Utils/AngleGenerator.h"

namespace Lib_Motor
{
    // ==============================================================================
    // [Group 1] 构造、初始化与生命周期
    // ==============================================================================

    MotorManager::MotorManager(const MotorConfig& config)
        : config_(config), state_(State::INIT), mode_(Mode::NONE), fault_(Fault::NONE)
    {
        ctx_ = {0};
    }

    void MotorManager::init()
    {
        state_ = State::INIT;
        mode_  = Mode::NONE;
        target_mode_ = Mode::NONE; // 复位时必须清除用户的请求，防止“诈尸”
        fault_ = Fault::NONE;
        ctx_ = {0};

        // 1. 计算 dt (防止除以0)
        if (config_.control.control_freq_hz < 1.0f) {
            dt_ = 1.0f / 10000.0f; // 默认防呆
        } else {
            dt_ = 1.0f / config_.control.control_freq_hz;
        }

        // [修改] 必须传入 config_ 的地址 (&)
        controller_.init(dt_, &config_);

        // 强制清零硬件寄存器
        if (config_.hal && config_.hal->set_duty) {
            // 把 0,0,0 写入定时器 CCR，确保物理上占空比为 0
            config_.hal->set_duty(0.0f, 0.0f, 0.0f);
        }

        // [新增] 强制清零内存中的占空比缓存
        ctx_.duty_a = ctx_.duty_b = ctx_.duty_c = 0.0f;
        ctx_.test_u = ctx_.test_v = ctx_.test_w = 0.0f;

        // --- 预计算系数 ---
        float v_ref = config_.sensor.adc_v_ref;
        float res   = config_.sensor.adc_resolution;
        float v_per_count = v_ref / res;

        // 电压系数
        float v_div_ratio = (config_.sensor.vbus_r_up + config_.sensor.vbus_r_down) / config_.sensor.vbus_r_down;
        ctx_.voltage_scale_V_per_count = v_per_count * v_div_ratio;

        // 相电流系数
        float phase_gain_total = config_.sensor.phase_shunt_resistor * config_.sensor.phase_amp_gain;
        if (phase_gain_total > 0.0f) {
            ctx_.current_scale_Phase_per_count = v_per_count / phase_gain_total;
        }

        // 母线电流系数
        float bus_gain_total = config_.sensor.bus_shunt_resistor * config_.sensor.bus_amp_gain;
        ctx_.current_scale_Bus_per_count = (bus_gain_total > 0.0f) ? (v_per_count / bus_gain_total) : 0.0f;

        // 默认中点
        float mid_point = res / 2.0f;
        ctx_.offset_ia = ctx_.offset_ib = ctx_.offset_ic = ctx_.offset_ibus = mid_point;

        if (config_.hal && config_.hal->pwm_disable) config_.hal->pwm_disable();
    }

    void MotorManager::reset() { init(); }

    void MotorManager::setState(State next_state)
    {
        // ================= [漏掉的逻辑：进入运行状态] =================
        if (next_state == State::RUN)
        {
            // 1. 同步模式：正式把用户的请求 (Target) 变成当前模式 (Mode)
            if (target_mode_ != Mode::NONE) {
                mode_ = target_mode_;
            }

            // 2. 开启硬件：调用 BSP 的 Enable
            if (config_.hal && config_.hal->pwm_enable) {
                config_.hal->pwm_enable();
            }
        }
        // ================= [原有的逻辑：退出运行状态] =================
        else if (next_state == State::STOP || next_state == State::ERROR || next_state == State::INIT)
        {
            // 1. 关闭硬件
            if (config_.hal && config_.hal->pwm_disable) {
                config_.hal->pwm_disable();
            }

            // 2. 清理数据
            ctx_.speed_rpm = 0.0f;
            ctx_.angle_elec = 0.0f;

            // 3. 复位算法
            controller_.reset();
        }

        // 正式更新状态变量
        state_ = next_state;
    }

    // ==============================================================================
    // [Group 2] 外部控制指令 (Setters)
    // ==============================================================================

    void MotorManager::setMode(Mode new_mode)
    {
        // [关键修复] 如果是从其他模式切入到“强拖/开环”模式，
        // 必须复位积分器，否则它会记着上一次的速度，导致没有软启动效果。
        if (new_mode != mode_) // 只有模式发生变化时才复位
        {
            if (new_mode == Mode::DEBUG_VF_DRAG ||new_mode == Mode::DEBUG_IF_DRAG ||new_mode == Mode::DEBUG_CURRENT_LOCK)
            {
                controller_.reset();
            }
        }

        // 原有逻辑
        target_mode_ = new_mode;

        // 如果是 debug 强制切模式，可能需要立即生效
        // 但通常由 FSM 处理，这里只设置 target_mode_ 即可
    }


    void MotorManager::setTargetSpeed(float rpm) { ctx_.target_rpm = rpm; }

    void MotorManager::setTargetTorque(float current_a) {
        ctx_.target_iq = current_a;
        // ctx_.target_id = 0.0f; // 力矩模式默认 Id=0
    }

    // [新增] 设置 D 轴电流
    void MotorManager::setTargetId(float current_a) {
        ctx_.target_id = current_a;
    }

    void MotorManager::setRawPWM(float u, float v, float w) {
        ctx_.test_u = u; ctx_.test_v = v; ctx_.test_w = w;
        setMode(Mode::DEBUG_PWM_MANUAL);
    }

    // ==============================================================================
    // [Group 3] 数据获取 (Getters)
    // ==============================================================================

    // [新增] 获取极对数
    uint8_t MotorManager::getPolePairs() const {
        return config_.physical.pole_pairs;
    }

    void MotorManager::getMonitorData(MotorMonitorData& data) const
    {
        data.state = state_;
        data.speed_rpm = ctx_.speed_rpm;
        data.v_bus = ctx_.v_bus;
        data.temperature = ctx_.temperature_c;
        data.fault_code = fault_;

        data.i_a = ctx_.i_a; data.i_b = ctx_.i_b; data.i_c = ctx_.i_c;data.i_bus = ctx_.i_bus;
        data.i_d = ctx_.i_d; data.i_q = ctx_.i_q;
        data.duty_a = ctx_.duty_a; data.duty_b = ctx_.duty_b; data.duty_c = ctx_.duty_c;
        data.angle_elec = ctx_.angle_elec;
        data.observer_error = 0.0f;

        data.mode        = mode_;
        data.target_mode = target_mode_;
        data.run_phase   = run_phase_;
        data.event       = event_;
    }

    // ==============================================================================
    // [Group 4] 核心调度 (Tick & Calibration)
    // ==============================================================================

    void MotorManager::tick()
    {
        // [状态机计时器] 放在最前面，确保任何状态下都准确计时
        ctx_.fsm_timer_ticks++;

        // Step 1: 感知
        updateSensorMeasurements();

        // Step 2: 保护检查
        runSafetyCheck();

        // ================= [优化点：故障硬拦截] =================
        // 只要 fault_ 脏了，无论状态机在哪个位置，立即关断硬件并退出
        if (fault_ != Fault::NONE)
        {
            state_ = State::ERROR; // 强制同步状态
            if (config_.hal && config_.hal->pwm_disable) {
                config_.hal->pwm_disable();
            }
            ctx_.duty_a = ctx_.duty_b = ctx_.duty_c = 0.0f;
            return; // 立即退出，不再执行后续 FSM 或 控制逻辑
        }
        // =======================================================

        // Step 3: 观测
        runObserverLoop();

        // Step 4: 决策
        Motor_FSM::step(*this);

        // Step 5: 执行 (此时 fault_ 必为 NONE)
        if (state_ == State::RUN)
        {
            runControlLoop();
            if (config_.hal && config_.hal->set_duty) {
                config_.hal->set_duty(ctx_.duty_a, ctx_.duty_b, ctx_.duty_c);
            }
        }
        else if (state_ == State::ADC_CAL)
        {
            runAdcCalibration();
        }
        else
        {
            if (config_.hal && config_.hal->pwm_disable) config_.hal->pwm_disable();
        }
    }

    void MotorManager::runAdcCalibration()
    {
        // [安全补丁] 只要进入校准，必须强制 PWM 占空比为 0
        // 防止从 RUN 状态复位进来时，硬件寄存器还残留着之前的波形
        if (config_.hal && config_.hal->set_duty) {
            config_.hal->set_duty(0.0f, 0.0f, 0.0f);
        }

        if (ctx_.calib_counter == 0) {
            ctx_.calib_accum_ia = ctx_.calib_accum_ib = ctx_.calib_accum_ic = ctx_.calib_accum_ibus = 0.0f;
            if (config_.hal && config_.hal->pwm_disable) config_.hal->pwm_disable();
        }

        // [修改] 一次性读取并累加
        if (config_.hal && config_.hal->read_currents_raw) {
            uint16_t ra, rb, rc, rbus; // 定义 4 个临时变量

            // 调用新接口
            config_.hal->read_currents_raw(&ra, &rb, &rc, &rbus);

            // 累加
            ctx_.calib_accum_ia += ra;
            ctx_.calib_accum_ib += rb;
            ctx_.calib_accum_ic += rc;
            ctx_.calib_accum_ibus += rbus;
        }

        ctx_.calib_counter++;

        if (ctx_.calib_counter >= 1000) {
            ctx_.offset_ia = ctx_.calib_accum_ia / 1000.0f;
            ctx_.offset_ib = ctx_.calib_accum_ib / 1000.0f;
            ctx_.offset_ic = (ctx_.calib_accum_ic > 10000.0f) ? (ctx_.calib_accum_ic/1000.0f) : (config_.sensor.adc_resolution/2.0f);
            ctx_.offset_ibus = ctx_.calib_accum_ibus / 1000.0f;

            // ctx_.calib_counter = 0;
        }
    }

    // ==============================================================================
    // [Group 5] 内部核心算法 (Internals)
    // ==============================================================================

    void MotorManager::updateSensorMeasurements()
    {
        // [修改] 读取所有电流
        if (config_.hal && config_.hal->read_currents_raw)
        {
            uint16_t ra, rb, rc, rbus;
            config_.hal->read_currents_raw(&ra, &rb, &rc, &rbus); // 一次读完

            // ============================================================
            // 1. 计算【瞬时】物理值 (Instantaneous Values)
            //    注意：这里带上了你之前确认的负号修正
            // ============================================================
            float ia_inst = ((float)ra - ctx_.offset_ia) * ctx_.current_scale_Phase_per_count;
            float ib_inst = ((float)rb - ctx_.offset_ib) * ctx_.current_scale_Phase_per_count;

            // ============================================================
            // 2. 进行低通滤波 (Filtering) - [关键步骤在这里！]
            //    dt_ 是控制周期，必须传进去计算 alpha
            // ============================================================
            ctx_.i_a = lpf_ia_(ia_inst, dt_);
            ctx_.i_b = lpf_ib_(ib_inst, dt_);

            // ============================================================
            // 3. 处理 Ic
            // ============================================================

            // 【方案 A：推荐】数学重构法 (最干净，无视硬件差异)
            // 哪怕你有采样 rc，也建议用这个。它能保证 Ia+Ib+Ic=0，对 PID 最友好。
            // ctx_.i_c = -ctx_.i_a - ctx_.i_b;

            // 【方案 B：如果你非要用采样值】(不推荐)
            // 既然你一定要用采样的 Ic，那它也必须滤波！
            float ic_inst = ((float)rc - ctx_.offset_ic) * ctx_.current_scale_Phase_per_count;
            ctx_.i_c = lpf_ic_(ic_inst, dt_);


            // ============================================================
            // 4. 母线电流处理 (也要滤波)
            // ============================================================
            float ibus_inst = ((float)rbus - ctx_.offset_ibus) * ctx_.current_scale_Bus_per_count;
            ctx_.i_bus = lpf_ibus_(ibus_inst, dt_);
        }

        // 2. 母线电压 (带 LPF)
        if (config_.hal && config_.hal->read_vbus_raw)
        {
            // 1. 读原始物理值
            float v_inst = (float)config_.hal->read_vbus_raw() * ctx_.voltage_scale_V_per_count;

            // 2. [修改] 使用统一的滤波器类
            // 注意：第一次运行时 v_bus 为 0，滤波器可能会有一个爬升过程。
            // LowPassFilter 类里最好加一个 auto_reset 逻辑，或者在 init 时预设初值。
            // 但对于电压来说，0.02s 的爬升人眼看不出来，直接这样写没问题。
            ctx_.v_bus = lpf_vbus_(v_inst, dt_);
        }

        // 3. 温度 (降频)
        ctx_.slow_loop_counter++;
        if (ctx_.slow_loop_counter >= 1000) {
            ctx_.slow_loop_counter = 0;
            if (config_.hal && config_.hal->read_temp_raw) {
                ctx_.temperature_c = calculateNTC(config_.hal->read_temp_raw());
            }
        }
    }

    void MotorManager::runSafetyCheck()
    {
        // 1. 电压检查
        if (ctx_.v_bus > config_.limit.over_voltage_v) {
            fault_ = (Fault)((uint16_t)fault_ | (uint16_t)Fault::OVERVOLT);
        }
        if (ctx_.v_bus < config_.limit.under_voltage_v) {
            fault_ = (Fault)((uint16_t)fault_ | (uint16_t)Fault::UNDERVOLT);
        }

        // 2. [修正] 电流保护：检查三相电流的绝对值
        // 只要任意一相超过阈值，立即保护。这是保护硬件最可靠的方法。
        float limit = config_.limit.max_current_a;

        // 取三相中的最大值 (绝对值)
        float max_i = fabsf(ctx_.i_a);
        if (fabsf(ctx_.i_b) > max_i) max_i = fabsf(ctx_.i_b);
        if (fabsf(ctx_.i_c) > max_i) max_i = fabsf(ctx_.i_c);

        if (max_i > limit) {
            fault_ = (Fault)((uint16_t)fault_ | (uint16_t)Fault::OVERCURRENT);
        }

        // 3. 温度保护
        if (ctx_.temperature_c > config_.limit.over_temp_c) {
            fault_ = (Fault)((uint16_t)fault_ | (uint16_t)Fault::OVERTEMP);
        }
    }

    void MotorManager::runObserverLoop()
    {
        // TODO: SMO / PLL 计算角度和速度
        // ctx_.angle_elec = ...
    }

    // [核心修正] 模式分流逻辑
    void MotorManager::runControlLoop()
    {
        switch (mode_)
        {
            // --- 调试模式 ---
            case Mode::DEBUG_PWM_MANUAL:
                runPWMManual();
                return; // 直接返回，不走下面的 SVPWM

            case Mode::DEBUG_VF_DRAG:
            {
                runVFControl();
                break;
            }

            case Mode::DEBUG_CURRENT_LOCK:
                runCurrentLock();
                break;

            case Mode::DEBUG_IF_DRAG:
                runIFControl();
                break;

            // --- 正常模式 ---
            case Mode::VELOCITY_CONTROL:
                break;

            case Mode::TORQUE_CONTROL:
                // 正常 FOC 逻辑...
                break;

            default:
                break;
        }

        // [通用路径] PID -> Park -> SVPWM
        // 如果是 DEBUG_PWM_MANUAL，上面已经 return 了，不会执行到这里
        // TODO: 调用 PID 和 SVPWM
    }

    // 1. 手动 PWM 的 Wrapper
    void MotorManager::runPWMManual()
    {
        controller_.processManualPWM(
            ctx_.test_u, ctx_.test_v, ctx_.test_w,
            ctx_.duty_a, ctx_.duty_b, ctx_.duty_c
        );
    }

    void MotorManager::runVFControl()
    {
        // 调用纯数学控制器，进行参数映射
        controller_.processVF(
            ctx_.target_rpm,          // [输入] 目标转速
            ctx_.debug_accel_rad_s2,  // [输入] 爬坡加速度 (由 API 计算并传入)
            ctx_.target_id,           // [输入] 复用 target_id 作为 "duty_bias" (API传入的占空比)
            config_.observer,         // [输入] 配置结构体 (包含 V/F 斜率参数)
            ctx_.v_bus,               // [输入] 当前母线电压

            // --- [输出] 下面是引用传递，Controller 算完会直接修改 ctx_ 中的值 ---
            ctx_.angle_elec,          // [输出] 电角度 (用于 FOC 变换)
            ctx_.speed_rpm,           // [输出] 实际爬坡速度 (用于 Ozone 监视)
            ctx_.v_d,                 // [输出] Vd 电压 (通常为 0)
            ctx_.v_q,                 // [输出] Vq 电压 (V/F 计算出的电压)
            ctx_.duty_a,              // [输出] A相占空比
            ctx_.duty_b,              // [输出] B相占空比
            ctx_.duty_c               // [输出] C相占空比
        );

        // 2. [新增] 为了能在监控中看到 Id Iq，必须手动做变换
        float i_alpha, i_beta;
        MotorMath::Clark(ctx_.i_a, ctx_.i_b, &i_alpha, &i_beta);
        MotorMath::Park(i_alpha, i_beta, ctx_.angle_elec, &ctx_.i_d, &ctx_.i_q);
    }

    // 3. Current Lock 的 Wrapper (保持现状，或者也改用 processCurrentLoop)
    void MotorManager::runCurrentLock()
    {
        float i_alpha, i_beta;

        // 1. Clark 变换 (调用库函数)
        MotorMath::Clark(ctx_.i_a, ctx_.i_b, &i_alpha, &i_beta);

        // 2. Park 变换
        // Lock 模式下，强制电角度为 0
        // 虽然 Park(..., 0, ...) 计算量有点浪费(因为sin0=0, cos0=1)，
        // 但为了代码通用性，直接调用 Park 是没问题的。
        // 如果极其在意性能，也可以保留之前的 ctx_.i_d = i_alpha; 写法。
        MotorMath::Park(i_alpha, i_beta, 0.0f, &ctx_.i_d, &ctx_.i_q);

        // 3. 闭环控制
        controller_.processCurrentLoop(
            ctx_.target_id, ctx_.target_iq,
            0.0f,                   // 角度
            ctx_.i_d, ctx_.i_q,     // 反馈
            ctx_.v_bus,
            ctx_.v_d, ctx_.v_q,
            ctx_.duty_a, ctx_.duty_b, ctx_.duty_c
        );
    }

    void MotorManager::runIFControl()
    {
        // Clark 变换
        float i_alpha = ctx_.i_a;
        float i_beta  = (ctx_.i_a + 2.0f * ctx_.i_b) * 0.57735f;

        // [核心修改]
        // 调试模式下：
        // 1. 加速度 -> 用 API 算好并传入的 debug_accel_rad_s2
        // 2. Id     -> 用 API 传入的 target_id
        // 3. Iq     -> 用 API 传入的 target_iq
        // 4. Config -> 完全不读！实现解耦。

        controller_.processIF(
            ctx_.target_rpm,
            ctx_.debug_accel_rad_s2,   // API 算出来的加速度
            ctx_.target_id,            // API 设置的 Id
            ctx_.target_iq,            // API 设置的 Iq
            i_alpha, i_beta,
            ctx_.v_bus,

            // 回填
            ctx_.angle_elec, ctx_.speed_rpm,
            ctx_.i_d, ctx_.i_q,
            ctx_.v_d, ctx_.v_q,
            ctx_.duty_a, ctx_.duty_b, ctx_.duty_c
        );
    }

    void MotorManager::setDebugAccel(float acc) {
        ctx_.debug_accel_rad_s2 = acc;
    }

    float MotorManager::calculateNTC(uint16_t raw_adc)
    {
        if (raw_adc == 0 || raw_adc >= config_.sensor.adc_resolution) return -99.0f;
        float r_series = config_.sensor.ntc_r_series;
        float r_ntc;

        if (config_.sensor.ntc_type == NtcTopology::LOW_SIDE_NTC)
            r_ntc = r_series / ((config_.sensor.adc_resolution / (float)raw_adc) - 1.0f);
        else
            r_ntc = (r_series * config_.sensor.adc_resolution / (float)raw_adc) - r_series;

        const float T0 = 298.15f;
        float ln_ratio = logf(r_ntc / config_.sensor.ntc_r_25);
        return (1.0f / ((1.0f / T0) + (ln_ratio / config_.sensor.ntc_beta))) - 273.15f;
    }

    bool MotorManager::checkConfigValid(const MotorConfig& cfg)
    {
        return (cfg.hal && cfg.hal->set_duty);
    }
}