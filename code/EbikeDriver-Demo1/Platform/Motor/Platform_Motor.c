#include "Platform_Motor.h"

#ifdef EnableFunction_Motor

static float Theta = 0.0f;
uint32_t CCR1, CCR2, CCR3;
float vofa_data [8];
uint16_t vofa_raw_data;
float MechanicaAngle, ElectricalAngle;

void Platform_Motor_Init(void)
{
    BSP_Motor_Init();
}

void Platform_Motor_Start(void)
{
    BSP_Motor_Start();
}

void Platform_Motor_Stop(void)
{
    BSP_Motor_Stop();
}

void Platform_Motor_SetSpeed(int speed)
{

}

void Platform_Motor_Loop(void)
{
    uint32_t ARR = BSP_Motor_GetAutoReload(); // ARR 对应一个半周期计数范围（中心对齐时 CCR 与 ARR 的换算按占空比)

    float dtheta = 2.0f* 10 * PI * Tpwm;

    Theta += dtheta;
    if (Theta > 2.0f * PI)
        Theta -= 2.0f * PI;

    // 电流命令（固定测试；后期速度环输出）
    // float Id_cmd = 0.0f;  // d 轴 0
    // float Iq_cmd = 0.6f;  // q 轴扭矩 (A)

    //回调Theta角度值
    // MechanicaAngle = Platform_AS5047P_GetMechanicaAngle();
    // ElectricalAngle = fmodf(MechanicaAngle * NumberOfPolePairs, 2.0f * PI);

    // vofa_data[0] = MechanicaAngle;

    //三相电流采样
    ADC_RawValue_t ADC_RawValue = BSP_Motor_GetAdcRawValue();
    Shunt_Current_t Shunt_Current = {0};

    // Shunt_Current.Ia =((ADC_RawValue.Ua * Vref / 4096.0f) - Vref_Offset) / (GAIN * 0.005f);
    // Shunt_Current.Ib =((ADC_RawValue.Ub * Vref / 4096.0f) - Vref_Offset) / (GAIN * 0.005f);
    // Shunt_Current.Ic =((ADC_RawValue.Uc * Vref / 4096.0f) - Vref_Offset) / (GAIN * 0.005f);
    // Shunt_Current.Ibus =((ADC_RawValue.Ubus * Vref / 4096.0f) - Vref_Offset) / (GAIN * 0.005f);

    vofa_data[0] = ADC_RawValue.Ua;
    vofa_data[1] = ADC_RawValue.Ub;
    vofa_data[2] = ADC_RawValue.Uc;
    vofa_data[3] = ADC_RawValue.Ubus;

    //Clark a b c ---- Alpha Beta
    // Clark_OutPut_t  Clark_OutPut= Clark_Calculation(Shunt_Current.Ia, Shunt_Current.Ib, Shunt_Current.Ic);

    //Park Alpha Beta -----Id Iq
    // Park_OutPut_t Park_Output = Park_Calculation(Clark_OutPut.Ialpha, Clark_OutPut.Ibeta,Theta);

    //PID误差计算 Id Iq  ------ Ud Uq
    // Voltage_Output_t Voltage_OutPut = Motor_PI_Current_Calculation(Id_cmd, Iq_cmd, Park_Output.Id, Park_Output.Iq);

    //InvPark Ud Uq ----- Alpha Beta
    InvPark_OutPut_t InvPark_OutPut = InvPark_Calculation(0,1,Theta);

    //SVPWM  Alpha Beta ------a b c
    SVPWM_OutPut_t SVPWM_OutPut = SVPWM_Calculation(InvPark_OutPut.Ualpha, InvPark_OutPut.Ubeta);

    //更新CCR值
    // CCR1 = (uint32_t)(SVPWM_OutPut.Ta / Tpwm * (ARR + 1) + 0.5f);
    // CCR2 = (uint32_t)(SVPWM_OutPut.Tb / Tpwm * (ARR + 1) + 0.5f);
    // CCR3 = (uint32_t)(SVPWM_OutPut.Tc / Tpwm * (ARR + 1) + 0.5f);

    CCR1 = (uint32_t)(SVPWM_OutPut.Ta / Tpwm * (ARR + 1) + 0.5f);
    CCR2 = (uint32_t)(SVPWM_OutPut.Tb / Tpwm * (ARR + 1) + 0.5f);
    CCR3 = (uint32_t)(SVPWM_OutPut.Tc / Tpwm * (ARR + 1) + 0.5f);

    BSP_Motor_SetCompare(CCR1, CCR2, CCR3);
}

#endif