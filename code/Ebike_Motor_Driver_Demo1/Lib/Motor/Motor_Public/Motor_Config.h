#pragma once
#include <stdint.h>
#include "Motor_HAL_Interface.h"

namespace Lib_Motor
{
    // =========================================================
    // 子模块参数定义
    // =========================================================

    /* [1] 物理参数 (不可变属性) */
    struct MotorPhysicalParam
    {
        uint8_t pole_pairs      = 7;      // 极对数
        float   rated_current   = 1.0f;   // 额定相电流 (A) - 用于归一化或保护基准
        float   rated_voltage   = 24.0f;  // 额定电压 (V)
        float   rs              = 0.1f;   // 相电阻 (Ohm) - 观测器依赖
        float   ls              = 0.001f; // 相电感 (H)   - 观测器依赖
        float   kv_rating       = 100.0f; // 电机 KV 值 (RPM/V) - 可选，用于前馈计算
    };

    /* [2] 保护限制 (Safety First) */
    struct MotorLimitParam
    {
        float max_current_a     = 5.0f;    // 最大相电流 (超过触发 OverCurrent)
        float max_speed_rpm     = 3000.0f; // 最大转速 (超过触发 OverSpeed)
        float max_duty_cycle    = 0.95f;   // 最大占空比限制 (留出采样窗口)

        float over_voltage_v    = 30.0f;   // 母线过压阈值
        float under_voltage_v   = 10.0f;   // 母线欠压阈值
        float over_temp_c       = 85.0f;   // 过温阈值
    };

    /* [3] 传感器与硬件配置 */

    // NTC 电路拓扑类型
    enum class NtcTopology : uint8_t {
        LOW_SIDE_NTC,  // NTC 在下，电阻在上 (NTC 接地，最常见) -> V_out 随温度升高而降低
        HIGH_SIDE_NTC  // NTC 在上，电阻在下 (NTC 接Vcc) -> V_out 随温度升高而升高
    };

    struct MotorSensorParam
    {
        /* --- [1] ADC 固有属性 (所有模拟采样的基准) --- */
        float adc_v_ref      = 3.3f;   // ADC 参考电压 (通常 3.3V)
        float adc_resolution = 4096.0f;// ADC 分辨率 (2^12)

        /* --- [2] 母线电压采样 (分压电路) --- */
        float vbus_r_up      = 100000.0f; // 上拉电阻 (100k)
        float vbus_r_down    = 6200.0f;   // 下拉电阻 (6.2k)

        /* --- [3] 电流采样配置 (分离配置) --- */

        // A. 相电流 (用于 FOC 控制)
        float phase_shunt_resistor = 0.001f; // 相采样电阻 (例如 1mR)
        float phase_amp_gain       = 50.0f;  // 相运放增益

        // B. 母线电流 (用于功率统计或制动保护)
        // 允许母线采样的电阻/增益与相线不同
        float bus_shunt_resistor   = 0.001f; // 母线采样电阻
        float bus_amp_gain         = 20.0f;  // 母线运放增益 (可能更小以测量更大电流)

        /* --- [4] 温度采样 (NTC) --- */
        // 硬件拓扑
        NtcTopology ntc_type = NtcTopology::LOW_SIDE_NTC;
        float ntc_r_series   = 10000.0f;  // 串联分压电阻 (通常是 10k)
        // NTC 规格
        float ntc_r_25       = 10000.0f;  // 25度时的阻值 (10k)
        float ntc_beta       = 3950.0f;   // B值 (如 3950)

        /* --- [5] 位置传感器 (有感模式) --- */
        uint32_t cpr         = 16384;  // 线数 (Counts Per Rev)
        int8_t   direction   = 1;      // 编码器方向 (1 或 -1)
        float    offset_elec = 0.0f;   // 电角度零位偏置 (Rad)
    };

    /* [4] 控制回路参数 (PID) */
    struct PIDParam
    {
        float kp        = 0.0f;
        float ki        = 0.0f;
        float kd        = 0.0f;
        float output_limit = 0.0f;       // 积分/输出限幅
        float ramp_rate    = 0.0f;       // 变化率限制 (可选)
    };

    struct MotorControlParam
    {
        float control_freq_hz = 10000.0f; // 控制频率，例如 10kHz

        PIDParam current_d;       // D轴电流环
        PIDParam current_q;       // Q轴电流环
        PIDParam speed;           // 速度环
        PIDParam position;        // 位置环
    };

    /* [5] 启动与无感策略 (Strategy) */
    enum class ObserverStrategy : uint8_t
    {
        FORCE_DRAG_TO_SMO,    // I/F 强拖 -> 切换滑模 (低成本通用)
        HFI_TO_SMO,           // 高频注入 -> 切换滑模 (低速带载能力强)
        HALL_TO_SMO,          // 霍尔启动 -> 切换滑模 (高可靠性混合)
        ENCODER_ONLY          // 纯有感 (FOC)
    };

    struct MotorObserverParam
    {
        ObserverStrategy strategy = ObserverStrategy::FORCE_DRAG_TO_SMO;

        // --- 启动/对齐参数 ---
        float align_current_a   = 1.0f;   // 预对齐电流
        float align_time_s      = 0.5f;   // 预对齐保持时间

        // --- 强拖 (I/F) 参数 ---
        float drag_current_a    = 1.5f;   // 强拖电流 (通常比额定大一点用于克服静摩擦)
        float drag_accel_rpm_s  = 1000.f; // 强拖加速度

        // --- 切换逻辑 ---
        float switch_speed_rpm  = 500.0f; // 观测器切入/融合转速
        float hysteresis_rpm    = 50.0f;  // 切换迟滞

        // --- 观测器增益 ---
        float smo_gain          = 1.0f;   // 滑模增益
        float pll_kp            = 2.0f;   // 锁相环 P (用于跟踪反电动势角度)
        float pll_ki            = 50.0f;  // 锁相环 I

        // [新增] V/F 调试参数
        float vf_start_voltage_ratio = 0.02f; // V/F 启动电压比例 (Duty)
        float vf_curve_slope         = 0.002f; // V/F 曲线斜率 (V / RPM)
    };

    // =========================================================
    // 总配置入口
    // =========================================================
    struct MotorConfig
    {
        // [关键] HAL 接口指针 (必须非空)
        const MotorHAL_t* hal = nullptr;

        MotorPhysicalParam physical; // 物理属性
        MotorLimitParam    limit;    // 保护限制 (放在前面以示重要)
        MotorSensorParam   sensor;   // 传感器/采样
        MotorObserverParam observer; // 策略/启动
        MotorControlParam  control;  // PID 参数
    };
}