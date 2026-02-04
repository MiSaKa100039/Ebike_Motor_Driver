#include "UserAPP.h"
#include "Platform_Motor.h"

void User_Init(void)
{
    Platform_Motor_Init();
}

void User_Loop(void)
{
    static uint8_t CMD_start = 0; // 用于触发启动
    static uint8_t CMD_reset = 0; // 用于触发复位
    static uint8_t CMD_stop = 0;
    static uint8_t CMD_clearfault= 0;

    static float dutyA = 0;
    static float dutyB = 0;
    static float dutyC = 0;

    static float dbg_duty = 0.0005f;
    static float dbg_rpm  = 500.0f;
    static float dbg_ramp = 5.0f;

    // 设定 1A 或 2A，不要设太大，否则发热严重
    // static float id_target = 0.0f;
    // static float iq_target = 0.0f;


    static float RPM_target = 0.0f;
    static float id_target = 0.0f;
    static float iq_target = 0.0f;
    static float Ramp_target = 0.0f;

    // 1. 复位
    if (CMD_reset == 1) {
        Global_Motor_0.reset();
        CMD_reset = 0;
    }

    if (CMD_clearfault == 1)
    {
        Global_Motor_0.clearFault();
        CMD_clearfault = 0;
    }

    if (CMD_start == 1) {
        // 1. 调试强制发波启动 PASS
        // Global_Motor_0.debugPWMManual(dutyA, dutyB, dutyC);

        // 2.VF强拖 PASS
        Global_Motor_0.debugVFControl(dbg_duty,dbg_rpm,dbg_ramp);

        // 电机锁轴 PID测试 PASS
        // Global_Motor_0.debugCurrentLock(id_target, iq_target);

        // 4.IF强拖
        // Global_Motor_0.debugIFControl(RPM_target,id_target,iq_target,Ramp_target);

        CMD_start = 0; // 确保只触发一次
    }

    // 3. 停止
    if (CMD_stop == 1) {
        Global_Motor_0.stop();
        CMD_stop = 0;
    }

    // // --- 监控 ---
    // static Lib_Motor::MotorMonitorData data;
    // Global_Motor_0.getMonitorData(data);
}

//VF 调试api重命名，以及参数问题
//故障测试单元
//lpf utils
//sum = Ia + Ib + Ic故障检测
//电阻采样方式配置，单 双 有无母线采样
//电流采样方向配置
//focmath svpwm合并
//整理优化排序