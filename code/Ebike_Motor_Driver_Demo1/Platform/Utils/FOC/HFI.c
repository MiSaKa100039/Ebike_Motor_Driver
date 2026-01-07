#include "FOC.h"

HFI_Parmeter_t HFI_Parm = {0};
PLL_Parmeter_t PLL_Parm = {0};

void HFI_Init(void)
{
    HFI_Parm.HFI_INJ_A = 0.15f;
    HFI_Parm.HFI_INJ_DIR = 1;
    PLL_Parm.PLL_KP = 2.5f;
    PLL_Parm.PLL_KI = 20.0f;
}

HFI_Parmeter_t HFI_InjectPWM(float Udc)
{
    HFI_Parm.HFI_INJ_Ud = Udc * HFI_Parm.HFI_INJ_A * HFI_Parm.HFI_INJ_DIR;

    HFI_Parm.HFI_INJ_DIR = -HFI_Parm.HFI_INJ_DIR;

    return HFI_Parm;
}

HFI_Parmeter_t HFI_GetHF_IalphaIbeta(float Ialpha, float Ibeta)
{
    static float Ialpha_K1 = 0, Ialpha_K2 = 0;
    static float Ibeta_K1 = 0, Ibeta_K2 = 0;
    static float HFI_HF_Ialpha1 = 0, HFI_HF_Ibeta1 = 0;  // 确保0

    // 计算当前HF（用旧K1/K2）
    HFI_Parm.HFI_HF_Ialpha = (Ialpha - 2 * Ialpha_K1 + Ialpha_K2) / 4.0f;
    HFI_Parm.HFI_HF_Ibeta = (Ibeta - 2 * Ibeta_K1 + Ibeta_K2) / 4.0f;

    // 计算差分（用上一HF1）
    HFI_Parm.HFI_Ialpha = (HFI_Parm.HFI_HF_Ialpha - HFI_HF_Ialpha1) * HFI_Parm.HFI_INJ_DIR;
    HFI_Parm.HFI_Ibeta = (HFI_Parm.HFI_HF_Ibeta - HFI_HF_Ibeta1) * HFI_Parm.HFI_INJ_DIR;

    // 更新延迟
    Ialpha_K2 = Ialpha_K1;
    Ibeta_K2 = Ibeta_K1;
    Ialpha_K1 = Ialpha;
    Ibeta_K1 = Ibeta;

    // 更新上一HF
    HFI_HF_Ialpha1 = HFI_Parm.HFI_HF_Ialpha;
    HFI_HF_Ibeta1 = HFI_Parm.HFI_HF_Ibeta;

    return HFI_Parm;
}

HFI_Parmeter_t HFI_GetLF_IdIq(float Id, float Iq)
{
    static float Id_K1 = 0, Id_K2 = 0;
    static float Iq_K1 = 0, Iq_K2 = 0;

    // 二阶低通滤波：(1 + 2 z^{-1} + z^{-2}) / 4
    HFI_Parm.HFI_LF_Id = (Id + 2 * Id_K1 + Id_K2) / 4.0f;
    HFI_Parm.HFI_LF_Iq = (Iq + 2 * Iq_K1 + Iq_K2) / 4.0f;

    // 更新延迟（计算后更新，逻辑清晰）
    Id_K2 = Id_K1;
    Iq_K2 = Iq_K1;
    Id_K1 = Id;
    Iq_K1 = Iq;

    return HFI_Parm;
}

PLL_Parmeter_t HFI_PLLProc(float HFI_Ialpha, float HFI_Ibeta)
{
    PLL_Parm.PLL_Thetae = Range_to_2PI(PLL_Parm.PLL_Thetae);

    PLL_Parm.PLL_x = HFI_Ialpha * sinf(PLL_Parm.PLL_Thetae);
    PLL_Parm.PLL_y = HFI_Ibeta * cosf(PLL_Parm.PLL_Thetae);
    PLL_Parm.PLL_z = PLL_Parm.PLL_x - PLL_Parm.PLL_y;

    PLL_Parm.PLL_Parm_P = PLL_Parm.PLL_z * PLL_Parm.PLL_KP;
    PLL_Parm.PLL_Parm_I += PLL_Parm.PLL_z * PLL_Parm.PLL_KI;
    PLL_Parm.PLL_Parm_I = PLL_Limit(PLL_Parm.PLL_Parm_I, 0.1f);

    PLL_Parm.PLL_Omegae = PLL_Parm.PLL_Parm_P + PLL_Parm.PLL_Parm_I;
    PLL_Parm.PLL_OmegaeF = PLL_LPF(PLL_Parm.PLL_Omegae);
    PLL_Parm.PLL_Thetae += PLL_Parm.PLL_Omegae * Tpwm * Motor_Pairs;

    return PLL_Parm;
}

static float PLL_Limit(float data, float lim)
{
    return fabsf(data) > lim ? (data > 0 ? lim : -lim) : data;
}

static float PLL_LPF(float data)
{
    static float alpha = 0.001247f;
    static float y = 0.0f, Last_y = 0.0f;

    y = alpha * data + Last_y * (1.0f - alpha);
    Last_y = y;

    return y;
}

static float Range_to_2PI(float in)
{
    static float temp = 0.0f;

    temp = in;
    if (temp > PI)
        temp -= 2.0f * PI;
    else if (temp < -PI)
        temp += 2.0f * PI;

    return temp;
}