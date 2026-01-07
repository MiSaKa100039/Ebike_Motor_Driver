#include "FOC.h"

SpeedLoop_Output_t PID_SpeedLoop_Calculation(float Ref_Speed, float Act_Speed)
{
    static float Motor_MaxIq = Motor_MaxCurrent * sqrtf(2.0f / 3.0f);
    static float Speed_Ki = 100.0f;  // 积分增益 (rad/s 或 rpm/s²)，调小防过冲
    static float Speed_Kp = 5.0f;  // 比例增益，调大加速响应

    SpeedLoop_Output_t SpeedLoop_Output = {0};
    static float Integral_Speed = 0.0f;  // 积分状态

    // 积分限幅：基于Iq_max / Ki（类似电流环）
    float IntegralLimit = Motor_MaxIq / Speed_Ki;

    // 速度误差 (rpm单位，直接用)
    float Err_Speed = Ref_Speed - Act_Speed;

    // 积分更新 (dt缩放，物理单位)
    Integral_Speed += Err_Speed * Tpwm;

    // 积分抗饱和
    Integral_Speed = fmaxf(-IntegralLimit, fminf(IntegralLimit, Integral_Speed));

    // PI输出：Iq_ref = Kp * Err + Ki * Integ
    SpeedLoop_Output.Tar_Iq = Speed_Kp * Err_Speed + Speed_Ki * Integral_Speed;

    // 输出限幅 (额定电流)
    SpeedLoop_Output.Tar_Iq = fmaxf(-Motor_MaxIq, fminf(Motor_MaxIq, SpeedLoop_Output.Tar_Iq));

    return SpeedLoop_Output;
}

CurrentLoop_Output_t PID_CurrentLoop_Calculation(float Ref_Id, float Ref_Iq, float Act_Id, float Act_Iq, float Udc)
{
    static float Current_Ki = 100.0f, Current_Kp = 0.025f;

    CurrentLoop_Output_t CurrentLoop_Output = {0};
    static float Integral_Id = 0, Integral_Iq = 0;

    float UrefMax = M_SQRT3 / 3.0f * Udc;  // ≈0.577 Udc
    float IntegralLimit = UrefMax / Current_Ki;

    // 误差：Ref - 低频反馈 (Act即i_df/i_qf)
    float Err_Id = Ref_Id - Act_Id;
    float Err_Iq = Ref_Iq - Act_Iq;

    // 积分更新 (dt缩放)
    Integral_Id += Err_Id * Tpwm;
    Integral_Iq += Err_Iq * Tpwm;

    // 积分抗饱和
    Integral_Id = fmaxf(-IntegralLimit, fminf(IntegralLimit, Integral_Id));  // 用fmax/fmin简洁
    Integral_Iq = fmaxf(-IntegralLimit, fminf(IntegralLimit, Integral_Iq));

    // PI输出
    CurrentLoop_Output.Tar_Ud = Current_Kp * Err_Id + Current_Ki * Integral_Id;
    CurrentLoop_Output.Tar_Uq = Current_Kp * Err_Iq + Current_Ki * Integral_Iq;

    // 解耦前馈 (加到PI输出)
    // CurrentLoop_Output.Tar_Ud += We * (Lq * Act_Iq);  // d轴：+ ω Lq Iq (简化，图中变体)
    // CurrentLoop_Output.Tar_Uq -= We * (Ld * Act_Id + Psi);  // q轴：- ω (Ld Id + ψ)

    // 矢量限幅 (dq圆)
    float U_mag = sqrtf(CurrentLoop_Output.Tar_Ud * CurrentLoop_Output.Tar_Ud +
                        CurrentLoop_Output.Tar_Uq * CurrentLoop_Output.Tar_Uq);
    if (U_mag > UrefMax) {
        CurrentLoop_Output.Tar_Ud *= UrefMax / U_mag;
        CurrentLoop_Output.Tar_Uq *= UrefMax / U_mag;
    }

    return CurrentLoop_Output;
}