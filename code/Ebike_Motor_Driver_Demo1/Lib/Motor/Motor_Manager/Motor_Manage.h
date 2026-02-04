#pragma once

#include "../Motor_Public/Motor_Definitions.h"
#include "../Motor_Public/Motor_Config.h"
#include "../Motor_Control/Motor_Control.h"
#include "../Motor_Uitls/LowPassFilter.h"

namespace Lib_Motor
{
    /**
     * @brief 电机核心管理器 (Core Manager)
     * @details 负责调度 FOC 算法、状态机 (FSM)、硬件交互 (HAL) 以及保护逻辑。
     * 它是电机库的“大脑”，由 API 层实例化并持有。
     */
    class MotorManager
    {
        friend class Motor_FSM; // 授权 FSM 访问我的私有成员(state_)

    public:
        explicit MotorManager(const MotorConfig& config);

        /* ==============================================================================
         * [Group 1] 生命周期与系统控制 (Lifecycle & System)
         * ============================================================================== */
        void init();                         // 初始化(软复位)
        void reset();                        // 复位：init()的别名
        void setState(State next_state);     // [FSM] 状态设置接口

        /* ==============================================================================
         * [Group 2] 外部控制指令 (Setters / Commands)
         * ============================================================================== */
        void setMode(Mode mode);             // 请求设置运行模式

        // --- 闭环目标设置 ---
        void setTargetSpeed(float rpm);      // 设置目标转速
        void setTargetTorque(float current_a);// 设置目标转矩 (Iq)

        // --- 调试与底层设置 ---
        void setTargetId(float current_a);   // [新增] 设置 D 轴目标电流 (用于调试/弱磁)
        void setRawPWM(float u, float v, float w); // [调试] 设置强制 PWM 占空比

        /* ==============================================================================
         * [Group 3] 数据获取与状态查询 (Getters / Monitoring)
         * ============================================================================== */
        float getSpeed() const { return ctx_.speed_rpm; }   // 获取转速
        float getBusVoltage() const { return ctx_.v_bus; }  // 获取母线电压
        float getCurrentQ() const { return ctx_.i_q; }      // 获取 Q 轴电流
        uint8_t getPolePairs() const;                       // [新增] 获取极对数

        // 获取全量监视数据
        void getMonitorData(MotorMonitorData& data) const;

        // 简单属性访问
        State    state() const      { return state_; }
        Mode     mode() const       { return mode_; }
        Mode     targetMode() const { return target_mode_; }
        Fault    fault() const      { return fault_; }
        const MotorEvent& event() const { return event_; }
        RunPhase runPhase() const            { return run_phase_; }
        void     setRunPhase(RunPhase phase) { run_phase_ = phase; }

        /* ==============================================================================
         * [Group 4] 核心调度与过程 (Core Scheduling)
         * ============================================================================== */
        // 核心心跳 (必须在中断中调用)
        void tick();

        // 运行 ADC 校准过程 (需在 ADC_CAL 状态下持续调用)
        void runAdcCalibration();

        void setDebugAccel(float acc);

        void runPWMManual();
        void runVFControl();
        void runCurrentLock();
        void runIFControl();


    private:
        /* ==================== 1. 核心状态变量 ==================== */
        State      state_;           // 当前主状态
        Mode       mode_;            // 当前工作模式
        Mode       target_mode_;     // 用户请求的目标模式
        RunPhase   run_phase_;       // RUN 状态下的子阶段
        Fault      fault_;           // 故障掩码
        MotorEvent event_;           // 系统事件标志

        // [新增] 定义滤波器对象
        // 参数 0.005f 是时间常数 Tf (5ms)，这是针对你的噪声环境的建议值
        LowPassFilter lpf_ia_{0.0025f};
        LowPassFilter lpf_ib_{0.0025f};
        LowPassFilter lpf_ic_{0.0025f};   // 如果你一定要用采样Ic，就加上这个
        LowPassFilter lpf_ibus_{0.02f};  // 母线电流通常可以用更强的滤波

        // [新增] 母线电压滤波器 (电压变化慢，可以用大一点的时间常数，比如 10ms-20ms)
        LowPassFilter lpf_vbus_{0.02f};

        float dt_;   //预计算的控制周期 (秒)

        /* ==================== 2. 配置与依赖 ==================== */
        const MotorConfig config_;   // 配置引用

        /* ==================== 3. 运行时数据上下文 (Context) ==================== */
        struct {
            /* --- [Group A] 预计算系数 --- */
            float current_scale_Phase_per_count;
            float current_scale_Bus_per_count;
            float voltage_scale_V_per_count;

            /* --- [Group B-0] 校准过程临时变量 --- */
            float calib_accum_ia;
            float calib_accum_ib;
            float calib_accum_ic;
            float calib_accum_ibus;
            uint16_t calib_counter;

            /* --- [Group B] 零点偏置 --- */
            float offset_ia;
            float offset_ib;
            float offset_ic;
            float offset_ibus;

            /* --- [Group C] 传感器反馈 --- */
            float i_a, i_b, i_c;
            float i_bus;
            float v_bus;
            float temperature_c;
            float angle_elec;
            float speed_rpm;

            /* --- [Group D] FOC 变换过程量 --- */
            float i_alpha, i_beta;
            float i_d, i_q;
            float v_d, v_q;
            float v_alpha, v_beta;

            /* --- [Group E] 控制目标 --- */
            float target_rpm;
            float target_iq;
            float target_id;

            /* --- [Group F] 最终输出 --- */
            float duty_a, duty_b, duty_c;

            /* --- [Group G] 调试与计数器 --- */
            // 1. 状态机专用计时器 (FSM Timer)
            // 用途：用于 ALIGNMENT, DRAG 等阶段的计时
            // 特点：每次进入新阶段或新状态时，需要手动清零 (Reset)
            uint32_t fsm_timer_ticks;

            // 2. 系统分频计数器 (System Divider)
            // 用途：用于温度采样等低频任务
            // 特点：一直累加，自动溢出或取模，不需要清零
            uint16_t slow_loop_counter;

            float debug_accel_rad_s2; // 调试模式专用的加速度限制

            float    test_u, test_v, test_w;
        } ctx_;

        /* ==================== 4. 内部核心算法 (Internals) ==================== */
        void updateSensorMeasurements(); // 物理量转换 (Sensing)
        void runSafetyCheck();           // 保护检查 (Protection)
        void runObserverLoop();          // 观测器 (Observer)
        void runControlLoop();           // 闭环控制与模式分发 (Control)

        MotorControl controller_;

        // 辅助计算
        float calculateNTC(uint16_t raw_adc);
        bool checkConfigValid(const MotorConfig& cfg);
    };
}