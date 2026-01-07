#ifndef FOC_H
#define FOC_H

#include "Platform_Motor.h"
#include <math.h>
#include <stdint.h>

typedef struct {
    float Ta;
    float Tb;
    float Tc;
}SVPWM_Output_t;

typedef struct
{
    float Ialpha;
    float Ibeta;
}Clark_Output_t;

typedef struct
{
    float Ualpha;
    float Ubeta;
}InvPark_Output_t;

typedef struct
{
    float Id;
    float Iq;
}Park_Output_t;

typedef struct
{
    float Tar_Iq;           //目标Iq
}SpeedLoop_Output_t;

typedef struct
{
    float Tar_Ud;               //目标Ud
    float Tar_Uq;               //目标Uq
}CurrentLoop_Output_t;

typedef struct
{
    float HFI_INJ_A;        // 注入幅度 HFI_INJF * Udc
    float HFI_INJ_Ud;
    float HFI_INJ_DIR;      //注入方向
    float HFI_Ialpha;       //电流包络线
    float HFI_Ibeta;
    float HFI_LF_Id;
    float HFI_LF_Iq;
    float HFI_HF_Ialpha;
    float HFI_HF_Ibeta;
    float HFI_ErrTheat;
}HFI_Parmeter_t;

typedef struct
{
    float PLL_Thetae;
    float PLL_Omegae;
    float PLL_OmegaeF;
    float PLL_x;
    float PLL_y;
    float PLL_z;
    float PLL_KP;
    float PLL_KI;
    float PLL_Parm_P;
    float PLL_Parm_I;
}PLL_Parmeter_t;

Clark_Output_t Clark_Calculation(float Ia, float Ib, float Ic);
InvPark_Output_t InvPark_Calculation(float Tar_Ud, float Tar_Uq, float Theta);
Park_Output_t Park_Calculation(float Ialpha, float Ibeta, float Theta);
SVPWM_Output_t SVPWM_Calculation(float Ualpha, float Ubeta ,float Udc);

SpeedLoop_Output_t PID_SpeedLoop_Calculation(float Vre_Speed, float Act_Speed);
CurrentLoop_Output_t PID_CurrentLoop_Calculation(float Ref_Id, float Ref_Iq, float Act_Id, float Act_Iq, float Udc);

void HFI_Init(void);
HFI_Parmeter_t HFI_InjectPWM(float Udc);
HFI_Parmeter_t HFI_GetHF_IalphaIbeta(float Ialpha, float Ibeta);
HFI_Parmeter_t HFI_GetLF_IdIq(float Id, float Iq);
PLL_Parmeter_t HFI_PLLProc(float HFI_Ialpha, float HFI_Ibeta);

static float PLL_Limit(float data, float lim);
static float PLL_LPF(float data);
static float Range_to_2PI(float in);

#endif //FOC_H
