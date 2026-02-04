#include "Platform_Motor.h"
#include "BSP_Motor.h"

/* 全局电机对象实例 */
Lib_Motor::MotorAPI Global_Motor_0;

void Platform_Motor_Init(void)
{
    /* 1. 准备配置结构体 (Static 确保栈不溢出，虽然在函数里也不大) */
    static Lib_Motor::MotorConfig cfg;

    // =============================================================
    // [1] 硬件绑定 (必须最先配置)
    // =============================================================
    cfg.hal = BSP_Get_HAL_Impl();

    // =============================================================
    // [2] 传感器配置 (决定电流采样精度)
    // =============================================================

    // --- A. ADC 基础属性 (根据你的 MCU 修改) ---
    cfg.sensor.adc_v_ref      = 3.28f;    // 如果你的参考电压是 3.0V，改这里
    cfg.sensor.adc_resolution = 4096.0f;  // 12-bit ADC

    // --- B. 电流采样 (Shunt) ---
    cfg.sensor.phase_shunt_resistor = 0.003f;   // 0.1mR
    cfg.sensor.bus_shunt_resistor   = 68.0f;   // 47V/V

    cfg.sensor.bus_shunt_resistor = 0.003f;   // 0.1mR
    cfg.sensor.bus_amp_gain       = 68.0f;   // 47V/V

    // --- C. 母线电压 (Resistor Divider) ---
    // 填入你原理图上的电阻值，库会自动计算分压比
    cfg.sensor.vbus_r_up      = 300000.0f;  // 上臂 300k
    cfg.sensor.vbus_r_down    = 10000.0f;   // 下臂 10k
    //!!!新的上拉电阻 = 原上拉电阻 x 真实电压 / 显示电压 用于线性校准漏电流造成的电压偏差

    // --- D. 温度保护 (NTC) ---
    // NTC 接法：通常是 Low Side (电阻接3.3V，NTC接地)
    cfg.sensor.ntc_type       = Lib_Motor::NtcTopology::LOW_SIDE_NTC;
    cfg.sensor.ntc_r_series   = 100000.0f;  // 分压电阻 100k
    // NTC 探头参数 (查 NTC 数据手册)
    cfg.sensor.ntc_r_25       = 10000.0f;  // R25 = 10k
    cfg.sensor.ntc_beta       = 3290.0f;   // B25/50 = 3290

    // =============================================================
    // [3] 电机物理参数 (决定观测器准确度)
    // =============================================================
    cfg.physical.pole_pairs    = 7;         // 极对数
    cfg.physical.rated_current = 5.0f;      // 额定电流 (A)
    cfg.physical.rated_voltage = 72.0f;     // 额定电压 (V)
    cfg.physical.rs            = 0.1f;      // 相电阻 (Ohm)
    cfg.physical.ls            = 0.0005f;   // 相电感 (H)

    // =============================================================
    // [4] 安全限制 (保护板子和电机)
    // =============================================================
    cfg.limit.max_current_a     = 40.0f;     // 绝对最大电流 (硬保护阈值)
    cfg.limit.max_speed_rpm     = 3000.0f;  // 最大转速
    cfg.limit.over_voltage_v    = 72.0f;    // 母线过压保护 (防止刹车反灌)
    cfg.limit.under_voltage_v   = 12.0f;    // 欠压保护
    cfg.limit.max_duty_cycle    = 0.95f;    // 限制最大占空比，留出采样时间
    cfg.limit.over_temp_c       = 85.0f;    //过温保护阈值

    // =============================================================
    // [5] 启动与无感策略 (决定能否转起来)
    // =============================================================
    // 采用 I/F 强拖启动，然后切换到 SMO 观测器
    cfg.observer.strategy         = Lib_Motor::ObserverStrategy::FORCE_DRAG_TO_SMO;

    // 强拖阶段参数 (Blind Run)
    cfg.observer.drag_current_a   = 1.5f;     // 强拖电流 (要足够克服静摩擦)
    cfg.observer.drag_accel_rpm_s = 1000.0f;  // 强拖加速度 (转/秒^2)

    // 切换参数
    cfg.observer.switch_speed_rpm = 500.0f;   // 在 500 RPM 时尝试闭环切换
    cfg.observer.smo_gain         = 1.0f;     // 滑模观测器增益 (需调试)

    //转速-电压补偿 克服反电动势
    cfg.observer.vf_start_voltage_ratio = 0.01f; // 1% 启动电压
    cfg.observer.vf_curve_slope         = 0.002f; // 每 1000 转增加 2V

    // =============================================================
    // [6] PID 参数 控制频率 (决定运行效果)
    // =============================================================

    cfg.control.control_freq_hz = 20000.0f; //控制频率 20KHz

    // 电流环 (D/Q 轴通常参数一致)
    // Kp 估算公式: Kp ≈ Ls * Bandwidth(比如1000Hz)
    // Ki 估算公式: Ki ≈ Rs * Bandwidth
    cfg.control.current_d.kp = 0.6f;
    cfg.control.current_d.ki = 10.0f;
    cfg.control.current_d.output_limit = 50.0f; // 电压输出限幅(Vbus)

    cfg.control.current_q.kp = 0.2f;
    cfg.control.current_q.ki = 5.0f;
    cfg.control.current_q.output_limit = 50.0f;

    // 速度环
    cfg.control.speed.kp = 0.05f;
    cfg.control.speed.ki = 0.001f;
    cfg.control.speed.output_limit = 5.0f; // 速度环输出的是目标电流(A)，限制为5A

    // =============================================================
    // [7] 初始化与启动
    // =============================================================

    // 初始化库 (分配内存、状态机复位)
    if (Global_Motor_0.init(cfg) != Lib_Motor::Result::Ok) {
        // 初始化失败 (可能是 HAL 指针为空等严重错误)
        // Handle Error...
    }

    // 启动底层硬件 (开启 ADC 中断, 定时器等)
    BSP_Motor_Hardware_Start();
}