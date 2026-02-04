#pragma once

#include <stdint.h>

namespace Lib_Motor
{
    /* ================= 电机状态（State）================= */
    enum class State : uint8_t {
        INIT = 0,             // 初始化状态 (MCU外设配置、参数加载、硬件自检)
        ADC_CAL,              // 校准状态 (测量相电流零点漂移/Offset，通常在上电或 Run 前执行)
        STOP,                 // 停止状态 (PWM 输出关断/高阻态，系统待机，响应指令)
        RUN,                  // 运行状态 (PWM 输出激活，执行闭环控制，具体行为依赖 currentMode)
        ERROR,                // 故障状态 (发生 Latching 故障，PWM 关断，需显式 Reset 才能退出)
    };

    /* ================= 运行模式（Mode）================= */
    enum class Mode : uint8_t {
        NONE = 0,             // 默认/空闲状态

        /* --- [Category 1] 危险调试 (CMake Debug Only) --- */
        // 这些模式在 Release 中应该被屏蔽或删除
        DEBUG_PWM_MANUAL,     // [极度危险] 手动强制 PWM
        DEBUG_CURRENT_LOCK,   // [工程师专用] 电流闭环锁轴 (调 PID 用)
        DEBUG_IF_DRAG,        // [工程师专用] 开环强拖 (验证硬件/霍尔/编码器)
        DEBUG_VF_DRAG,        // [工程师专用] 开环强拖 (验证硬件/霍尔/编码器)

        /* --- [Category 2] 生产与校准 (Release Available) --- */
        // 这些模式是安全的，用户可以通过 UI 触发
        CALIB_RL_IDENTIFY,    // [转正] 参数辨识 (自动测电阻电感)
        CALIB_ENCODER_OFFSET, // [新增] 编码器零点搜索 (如果是有感FOC)

        /* --- [Category 3] 全闭环运行 (业务模式) --- */
        TORQUE_CONTROL,       // 力矩控制 (Iq 闭环)
        VELOCITY_CONTROL,     // 速度控制 (速度环 + 电流环)
        POSITION_CONTROL,     // 位置控制 (位置环 + 速度环 + 电流环)

        /* --- [Category 4]制动与安全 --- */
        BRAKE_PASSIVE,        // 短路制动 (被动制动)
        BRAKE_ACTIVE,         // 主动制动 (反向电流注入，用于快速停止)

        /* --- 预留扩展 (未来模式，如力控或自定义) --- */
        RESERVED              // 占位符，用于开源贡献者扩展
    };

    /* ============ 运行阶段（RUN 内部子状态）=========== */
    enum class RunPhase : uint8_t {
        NONE = 0,        // 默认/不适用阶段

        RUN_DIRECT,      // 用于 DEBUG 模式，跳过启动流程

        /* --- 启动阶段 (通用，用于有感/无感初始化) --- */
        ALIGNMENT,       // 预对齐子阶段 (可选复用主模式的ALIGNMENT)
        FORCE_DRAG,      // 开环强拖 (I/F或V/F ramp-up，用于无感启动；可配置电流开环/闭环)

        /* --- 无感观测阶段 (低速到高速过渡) --- */
        HFI_ONLY,        // 纯高频注入 (零/低速)
        HFI_TRANS,       // HFI 到其他观测器的过渡 (渐变融合)
        SMO_ONLY,        // 纯滑模观测器 (中/高速)

        /* --- 基于BEMF的观测阶段 (扩展点) --- */
        BEMF_BASIC,      // 基本反电动势观测 (e.g., 线性BEMF)
        BEMF_ADVANCED,   // 高级BEMF变体 (e.g., Luenberger 或自适应；预留扩展)

        /* --- 高级/未来观测阶段 --- */
        EKF,             // Extended Kalman Filter (噪声鲁棒，未来扩展)
        HYBRID,          // 混合观测 (e.g., HFI+SMO+EKF 动态融合)
        RESERVED         // 占位符，用于开源贡献者添加新观测器 (e.g., PLL、MRAS)
    };

    /* ================= 故障掩码（Fault Bitmask）================= */
    // 使用十六进制定义，确保每一位（Bit）代表一种独立的故障
    // 指定底层为 uint16_t，确保跨平台内存布局一致
    // enum class 限制了作用域，必须通过 MotorFault:: 来访问
    enum class Fault : uint16_t {
        NONE         = 0x0000,  // 无故障
        OVERCURRENT  = 0x0001,  // 过流 (Bit 0)
        OVERVOLT     = 0x0002,  // 过压 (Bit 1)
        STALL        = 0x0004,  // 堵转 (Bit 2)
        SENSOR_LOSS  = 0x0008,  // 传感器丢失/编码器故障 (Bit 3)
        UNDERVOLT    = 0x0010,  // 欠压 (Bit 4)
        OVERTEMP     = 0x0020,  // 过温 (Bit 5)
        PHASE_LOSS   = 0x0040,  // 缺相 (Bit 6)
        PARAM_ERROR  = 0x0080   // 参数配置非法 (Bit 7)
    };

    /* ============ 通用返回结果 ============ */
    enum class Result : uint8_t {
        Ok = 0,             // 成功
        Error,              // 通用错误
        InvalidHandle,      // 句柄/ID 无效 (API层常见错误)
        InvalidState,       // 当前状态不允许该操作 (如在运行中尝试初始化)
        InvalidParam,       // 参数超出范围
        Timeout             // 超时
    };

    /* ============ FSM 事件 / 标志 ============ */
    struct MotorEvent{
        uint8_t observer_converged : 1; // [无感] 观测器收敛标志 (SMO/HFI 进入锁定状态)
        uint8_t speed_valid        : 1; // [通用] 速度反馈有效 (PLL 稳定或编码器信号正常)
        uint8_t adc_ready          : 1; // [硬件] ADC 注入组采样完成 (用于触发 FOC 计算)
        uint8_t motor_stalled      : 1; // [保护] 堵转检测标志 (反电动势过低或长期电流饱和)

        // 使用构造函数初始化列表清零
        MotorEvent()
            : observer_converged(0),
              speed_valid(0),
              adc_ready(0),
              motor_stalled(0) {}
    };

    /* ============ 监视数据快照 (Monitor Snapshot) ============ */

    // 一次性打包返回所有关键数据
    struct MotorMonitorData {
        /* =========================================================
             * [Group 1] 用户关注 (User Dashboard)
             * ---------------------------------------------------------
             * 最基础的运行指标，类似汽车仪表盘
             * ========================================================= */
        State state;            // 当前主状态 (INIT, RUN, STOP, ERROR...)
        Fault fault_code;       // 故障码
        float v_bus;            // 母线电压 (V)
        float speed_rpm;        // 当前转速 (RPM)
        float temperature;      // 温度 (℃)

        /* =========================================================
         * [Group 2] 内核状态 (Kernel Internals) - [新增/整理]
         * ---------------------------------------------------------
         * 了解 FSM 和 控制器的内部决策逻辑
         * ========================================================= */
        Mode       mode;        // 当前实际工作模式 (如 VELOCITY_CONTROL, DEBUG_PWM...)
        Mode       target_mode; // 用户请求的目标模式 (排查为什么不启动的关键)
        RunPhase   run_phase;   // 启动子阶段 (如 ALIGNMENT, OPEN_LOOP...)
        MotorEvent event;       // 系统事件标志 (如堵转标志)

        /* =========================================================
         * [Group 3] 示波器/开发者关注 (Oscilloscope)
         * ---------------------------------------------------------
         * 用于波形分析和 PID 调试
         * ========================================================= */
        float i_a, i_b, i_c;    // 相电流 (A)
        float i_bus;            // 母线电流 (A)

        float i_d, i_q;         // DQ轴电流 (反馈值)
        float angle_elec;       // 电角度 (rad)

        float duty_a, duty_b, duty_c; // 三相占空比 (0.0 ~ 1.0)

        // --- [高级调试] ---
        float observer_error;   // 观测器误差 (用于调参)
    };

    // /* ================= 6. C++ 魔法：运算符重载 ================= */
    // /* 由于 enum class 是强类型的，默认不能做 | (或) & (与) 运算。
    //    添加以下全局重载函数，让 Fault 类型支持位运算，像 int 一样好用。
    // */
    // inline Fault operator|(Fault a, Fault b) {
    //     return static_cast<Fault>(static_cast<uint16_t>(a) | static_cast<uint16_t>(b));
    // }
    //
    // inline Fault& operator|=(Fault& a, Fault b) {
    //     a = a | b;
    //     return a;
    // }
    //
    // inline Fault operator&(Fault a, Fault b) {
    //     return static_cast<Fault>(static_cast<uint16_t>(a) & static_cast<uint16_t>(b));
    // }
    //
    // inline Fault operator~(Fault a) {
    //     return static_cast<Fault>(~static_cast<uint16_t>(a));
    // }

}
