#include "Platform_Motor.h"

MotorSensor_t MotorSensor = {0};
ShuntOffset_t ShuntOffset = {0};
MotorStatus_t MotorStatus = {0};
ADCRawValue_t ADCRawValue = {0};
HFI_Parmeter_t HFI_Parmeter = {0};
PLL_Parmeter_t PLL_Parmeter = {0};

uint32_t CCR1,CCR2,CCR3;
static int8_t flag = 0;

void Platform_Motor_Init(void)
{
    BSP_Motor_Init();
    HFI_Init();
    MotorStatus.StatusADCOffset = false;
}

void Platform_Motor_Start(void)
{
    // HAL_Delay(1);
    // while (MotorStatus.StatusADCOffset == false)
    // {
    //     __NOP();
    // };
    Bsp_Motor_Start();
}

void Platform_Motor_Stop(void)
{
    Bsp_Motor_Stop();
}

MotorStatus_t Platform_Motor_Status(void)
{
    Platform_Motor_GetVoltage();
    // Platform_Motor_GetTemp();
}

static void Platform_Motor_GetADCOffset(bool StatusADCOffset)
{
    if (StatusADCOffset == false)
    {
        static uint8_t num = 1;

        if (num <= 10)
        {

            ShuntOffset.ShuntA += ADCRawValue.ShuntA;
            ShuntOffset.ShuntB += ADCRawValue.ShuntB;
            ShuntOffset.ShuntC += ADCRawValue.ShuntC;
            ShuntOffset.ShuntBus += ADCRawValue.ShuntBus;

            if (num == 10)
            {
                ShuntOffset.ShuntA = ShuntOffset.ShuntA / 10;
                ShuntOffset.ShuntB = ShuntOffset.ShuntB / 10;
                ShuntOffset.ShuntC = ShuntOffset.ShuntC / 10;
                ShuntOffset.ShuntBus = ShuntOffset.ShuntBus / 10;

                MotorStatus.StatusADCOffset = true;
            }

            num ++;
        }
    }
}

static void Platform_Motor_GetCurrent(void)
{
    MotorSensor.ShuntA = (float)(ADCRawValue.ShuntA - ShuntOffset.ShuntA) * ADC_VREF / 4096 / GainUVW / ((float)Res_UVW / 1000);
    MotorSensor.ShuntB = (float)(ADCRawValue.ShuntB - ShuntOffset.ShuntB) * ADC_VREF / 4096 / GainUVW / ((float)Res_UVW / 1000);
    MotorSensor.ShuntC = (float)(ADCRawValue.ShuntC - ShuntOffset.ShuntC) * ADC_VREF / 4096 / GainUVW / ((float)Res_UVW / 1000);
    MotorSensor.ShuntBus = (float)(ADCRawValue.ShuntBus - ShuntOffset.ShuntBus) * ADC_VREF / 4096 / GainBus / ((float)Res_Bus / 1000);
}

static void Platform_Motor_GetVoltage(void)
{
    MotorSensor.Udc = (float)ADCRawValue.Udc / 4096 * ADC_VREF * ((float)(Res_Pullup + Res_Pulldown) / Res_Pulldown);
}

static float Platform_Motor_GetTemp(uint16_t ADCRawValue)
{
    float Temperature;
    float Uadc;

    float Res_NTC = 0;

    Uadc = (float)ADCRawValue * ADC_VREF / 4096;

    switch (NTC_Type)
    {
        case 0: //高侧NTC
            Res_NTC = NTC_Pull * (ADC_VREF - Uadc) /Uadc;   break;
        case 1: //低侧NTC
            Res_NTC = NTC_Pull * Uadc / (ADC_VREF - Uadc);  break;
        default: break;
    }

    float inv_T =1.0f / NTC_K + (1.0f / (float)NTC_Beat) * logf(Res_NTC / (float)NTC_Resistor);
    float T_K = 1.0f / inv_T;

    Temperature = T_K - 273.15f;

    return Temperature;
}

static void Platform_Motor_Loop(void)
{
    uint32_t ARR = BSP_Motor_GetAutoReload();

    Platform_Motor_GetADCOffset(MotorStatus.StatusADCOffset);

    Platform_Motor_GetCurrent();
    Platform_Motor_GetVoltage();

    MotorSensor.Udc = 54;

    static float Theta = 0.0f;
    float dtheta = -2.0f* 15 * PI * Tpwm;

    Theta += dtheta;
    if (Theta > PI)
        Theta -= 2.0f * PI;
    else if (Theta < -PI)
        Theta += 2.0f * PI;

    //Clark a b c ---- Alpha Beta
    // Clark_Output_t  Clark_Output= Clark_Calculation(MotorSensor.ShuntA, MotorSensor.ShuntB, MotorSensor.ShuntC);

    //HFI Get High Frequency Band
    // HFI_Parmeter = HFI_GetHF_IalphaIbeta(Clark_Output.Ialpha, Clark_Output.Ibeta);
    //
    // PLL_Parmeter = HFI_PLLProc(HFI_Parmeter.HFI_Ialpha, HFI_Parmeter.HFI_Ibeta);

    //Park Alpha Beta -----Id Iq
    // Park_Output_t Park_Output = Park_Calculation(Clark_Output.Ialpha, Clark_Output.Ibeta,PLL_Parmeter.PLL_Thetae);

    //HFI Get Low Frequency Band
    // HFI_Parmeter = HFI_GetLF_IdIq(Park_Output.Id, Park_Output.Iq);

    //SpeedLoop
    // SpeedLoop_Output_t SpeedLoop_Output = PID_SpeedLoop_Calculation(100,PLL_Parmeter.PLL_OmegaeF);

    //CurrentLoop
    // CurrentLoop_Output_t CurrentLoop_Output = PID_CurrentLoop_Calculation(0.0f, 3.0f, HFI_Parmeter.HFI_LF_Id, HFI_Parmeter.HFI_LF_Iq, MotorSensor.Udc);

    //HFI
    // HFI_Parmeter = HFI_InjectPWM(MotorSensor.Udc);

    //InvPark Ud Uq ----- Alpha Beta
    InvPark_Output_t InvPark_Output = InvPark_Calculation(0, 6.0f, Theta);

    //SVPWM  Alpha Beta ------a b c
    SVPWM_Output_t SVPWM_Output = SVPWM_Calculation(InvPark_Output.Ualpha, InvPark_Output.Ubeta, MotorSensor.Udc);

    CCR1 = (uint32_t) (SVPWM_Output.Ta / Tpwm * (float) (ARR + 1) + 0.5f);
    CCR2 = (uint32_t) (SVPWM_Output.Tb / Tpwm * (float) (ARR + 1) + 0.5f);
    CCR3 = (uint32_t) (SVPWM_Output.Tc / Tpwm * (float) (ARR + 1) + 0.5f);

    BSP_Motor_SetCompare(CCR1, CCR2, CCR3);

    vofa_data[0] = MotorSensor.Udc;
    vofa_data[1] = MotorSensor.ShuntA;
    vofa_data[2] = MotorSensor.ShuntB;
    vofa_data[3] = MotorSensor.ShuntC;
    vofa_data[4] = MotorSensor.ShuntBus;
    // vofa_data[5] = MotorStatus.LVP;
    // vofa_data[6] = MotorSensor.ShuntA;
    // vofa_data[7] = MotorSensor.ShuntC;
}

// void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
// {
//     ADCRawValue_t ADCRawValue_Regular = Bsp_Motor_GetADCRawValue(Regular);
//
//     // ADCRawValue.NTC1 = ADCRawValue_Regular.NTC1;
//     // ADCRawValue.NTC2 = ADCRawValue_Regular.NTC2;
//     // ADCRawValue.Udc = ADCRawValue_Regular.Udc;
//
//     // Platform_VOFA_SendFloat(vofa_data,8);
// }

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{

    // HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);

    ADCRawValue_t ADCRawValue_Injected = Bsp_Motor_GetADCRawValue(Injected);

    ADCRawValue.ShuntA = ADCRawValue_Injected.ShuntA;
    ADCRawValue.ShuntB = ADCRawValue_Injected.ShuntB;
    ADCRawValue.ShuntC = ADCRawValue_Injected.ShuntC;
    ADCRawValue.ShuntBus = ADCRawValue_Injected.ShuntBus;

    Platform_VOFA_SendFloat(vofa_data,8);

    Platform_Motor_Loop();

    // HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
}
