#ifndef PID_H
#define PID_H

#include "math.h"
#include "Motor_Config.h"

typedef struct {
    float Kp;           // 比例增益 (0.2-1.0)
    float Ki;           // 积分增益 (5-20)
    float integral_d;   // d 轴积分
    float integral_q;   // q 轴积分
    float Vmax;         // 电压限幅 (Vbus / sqrt(3))
} PI_Current_Controller_t;

// PID 误差计算：输入命令 + 反馈 → Vd/Vq
typedef struct {
    float Vd;  // d 轴电压
    float Vq;  // q 轴电压
} Voltage_Output_t;

void PI_Current_Init(float Kp, float Ki, float Vbus);
Voltage_Output_t Motor_PI_Current_Calculation(float Id_cmd, float Iq_cmd, float Id_fb, float Iq_fb);

#endif //PID_H
