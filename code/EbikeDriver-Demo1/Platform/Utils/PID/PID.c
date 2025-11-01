#include "PID.h"

PI_Current_Controller_t PI_Current;

void PI_Current_Init(float Kp, float Ki, float Vbus)
{
    PI_Current.Kp = Kp;
    PI_Current.Ki = Ki;
    PI_Current.integral_d = 0.0f;
    PI_Current.integral_q = 0.0f;
    PI_Current.Vmax = Vbus / sqrtf(3.0f);  // 最大 Vd/Vq
}

Voltage_Output_t Motor_PI_Current_Calculation(float Id_cmd, float Iq_cmd, float Id_fb, float Iq_fb)
{
    // 误差
    float Id_error = Id_cmd - Id_fb;
    float Iq_error = Iq_cmd - Iq_fb;

    // 积分更新
    PI_Current.integral_d += Id_error * Tpwm;
    PI_Current.integral_q += Iq_error * Tpwm;

    // 积分限幅（防饱和，± Vmax / Ki）
    float int_limit = PI_Current.Vmax / PI_Current.Ki;
    PI_Current.integral_d = fminf(fmaxf(PI_Current.integral_d, -int_limit), int_limit);
    PI_Current.integral_q = fminf(fmaxf(PI_Current.integral_q, -int_limit), int_limit);

    // PI 输出
    float Vd = PI_Current.Kp * Id_error + PI_Current.Ki * PI_Current.integral_d;
    float Vq = PI_Current.Kp * Iq_error + PI_Current.Ki * PI_Current.integral_q;

    // 电压限幅
    Vd = fminf(fmaxf(Vd, -PI_Current.Vmax), PI_Current.Vmax);
    Vq = fminf(fmaxf(Vq, -PI_Current.Vmax), PI_Current.Vmax);

    Voltage_Output_t output = {Vd, Vq};
    return output;
}