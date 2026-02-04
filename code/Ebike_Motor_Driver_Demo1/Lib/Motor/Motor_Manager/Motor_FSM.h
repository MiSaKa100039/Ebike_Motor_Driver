#pragma once
#include "../Motor_Public/Motor_Definitions.h"

namespace Lib_Motor
{
    // 前向声明：只告诉编译器 MotorManager 是个类，避免循环引用头文件
    class MotorManager;

    /**
     * @brief 状态机 (Finite State Machine)
     * @details 纯静态类，负责管理电机的状态流转逻辑。
     * 它像一个医生，拿着 Manager 的数据 (*this) 进行诊断和开方。
     */
    class Motor_FSM
    {
    public:
        // 主步进函数：在 Tick 中被调用
        static void step(MotorManager& m);

    private:
        // 各状态的处理逻辑 (Handler)
        static void handleInit(MotorManager& m);
        static void handleAdcCal(MotorManager& m);
        static void handleStop(MotorManager& m);
        static void handleRun(MotorManager& m);
        static void handleError(MotorManager& m);
    };
}