#pragma once

#include "Motor_Config.h"
#include "Motor_Definitions.h"

namespace Lib_Motor
{
    class MotorManager; // 前向声明

    /**
     * @brief 电机控制对象 (API层)
     * @details 用户操作电机的唯一句柄。
     */
    class MotorAPI
    {
    public:
        MotorAPI();

        /**
         * @brief 初始化电机系统 (必须最先调用)
         */
        Result init(const MotorConfig& cfg);

        /* ==============================================================================
         * [Group 1] 基础启停与状态清除 (Basic Control)
         * ============================================================================== */

        /**
         * @brief 启动电机 (Enable)
         * @details 开启 PWM，默认进入速度模式。
         */
        Result start();

        /**
         * @brief 停止电机 (Disable)
         * @details 立即关闭 PWM (高阻态)。
         */
        Result stop();

        /**
         * @brief 清除故障标志
         * @details 只有清除故障后才能再次 start。
         */
        Result clearFault();

        Result reset();

        /* ==============================================================================
         * [Group 2] 闭环运动控制 (Motion Control)
         * ============================================================================== */

        /**
         * @brief 设置目标转矩 (会自动切入 TORQUE 模式)
         * @param current 目标 Iq 电流 (A)
         */
        Result setTargetTorque(float current);

        /**
         * @brief 设置目标速度 (会自动切入 VELOCITY 模式)
         * @param rpm 目标转速 (RPM)
         */
        Result setTargetSpeed(float rpm);

        /**
         * @brief 设置目标位置 (会自动切入 POSITION 模式)
         * @param angle 目标角度
         */
        Result setTargetPosition(float angle);

        /* ==============================================================================
         * [Group 3] 生产与校准 (Production & Calibration)
         * ============================================================================== */

        /**
         * @brief 启动参数辨识 (RL Identify)
         * @details 自动测量电阻电感，Release 版本可用。
         */
        Result startRLIdentification();

        /* ==============================================================================
         * [Group 4] 研发调试接口 (Debug Only)
         * ============================================================================== */
        #ifdef ENABLE_DANGEROUS_TEST_API

        /**
         * @brief [危险] 手动强制 PWM
         * @param u,v,w 占空比 (0.0~1.0)
         */
        Result debugPWMManual(float u, float v, float w);

        /**
         * @brief [调试] 电流闭环锁轴
         * @details 强制电角度为0。
         * @param i_d 直轴电流 (发热/吸死)
         * @param i_q 交轴电流 (力矩/横向推力)
         */
        Result debugCurrentLock(float i_d, float i_q);

        /**
         * @brief [调试] I/F 强拖启动
         * @details 开环角度，闭环电流。
         */
        Result debugIFControl(float target_rpm, float id_amp, float iq_amp, float ramp_time_s);

        /**
           * @brief [调试] V/F 开环旋转
           * @param duty       电压比例 (0.0 ~ 1.0)
           * @param speed_rpm  目标转速
           * @param ramp_time_s 爬升时间 (0秒表示瞬间到达，2秒表示花2秒加速到目标)
           */
        Result debugVFControl(float duty, float speed_rpm, float ramp_time_s);

        #endif

        /* ==============================================================================
         * [Group 5] 监视与数据获取 (Monitoring)
         * ============================================================================== */

        /**
         * @brief 获取全量监视数据 (推荐)
         */
        void getMonitorData(MotorMonitorData& out_data) const;

        State getState() const;
        float getSpeed() const;

    private:

        static bool isDebugOrCalib(Mode m);
        Result checkSafeModeChange(int id, Mode new_mode);

        int id_ = -1;
    };
}

// ISR 蹦床函数声明
extern "C" void Motor_Global_Process_Handler(int motor_id);