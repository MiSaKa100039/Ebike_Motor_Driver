#ifndef PLATFORM_MOTOR_H
#define PLATFORM_MOTOR_H

#include <stdbool.h>
#include "BSP_Motor.h"
#include "math.h"
#include "FOC.h"

#include "Platform_VOFA.h"

// 38.3V 142.31Hz

#define ADC_VREF 3.3f               //ADC参考电压
#define Tpwm  (1.0f / 20000.0f)     //PWM频率
//电机参数设定
#define Motor_Rs 95.5 mR
#define Motor_Ld 69 uh
#define Motor_Lq 69 uh
#define Motor_Pairs 7               //电机极对数
#define Motor_MaxCurrent 10
//保护阈值设定
#define OCP_Value 5                 //过流阈值 单位A
#define OVP_Value 24                //过压阈值 单位V
#define OTP_Value 100               //过温阈值 单位℃
//电流采样检测
#define Res_UVW 0.1f                   //UVW采样电阻阻值 单位mR
#define Res_Bus 0.1f                   //母线采样电阻阻值 单位mR
#define GainUVW 47                  //UVW运放增益系数
#define GainBus 47                  //母线运放增益系数
//母线电压检测
#define Res_Pullup 300               //分压上拉电阻 单位K
#define Res_Pulldown 10              //分压下拉电阻
//NTC温度检测
#define NTC_Type 1                  //0高侧NTC 1低侧NTC
#define NTC_Resistor 10             //10K or 100K
#define NTC_Pull 10                 //上下拉分压电阻
#define NTC_Beat 3950.f             //10K--3950 100K--4250
#define NTC_K 298.15f               //25°C (K)

typedef struct
{
    float ShuntA;
    float ShuntB;
    float ShuntC;
    float ShuntBus;
    float Theta;
    float Udc;
    float NTC1;
    float NTC2;
    float NTC3;
    float Pot;
}MotorSensor_t;

typedef struct
{
    uint16_t ShuntA;
    uint16_t ShuntB;
    uint16_t ShuntC;
    uint16_t ShuntBus;
}ShuntOffset_t;

typedef struct
{
    bool StatusOK;
    bool StatusBusy;
    bool StatusADCOffset;
    bool OTP;
    bool OVP;
    bool OCP;
    bool LVP;
}MotorStatus_t;

typedef enum {
    OBSERVER_MODE_HFI = 0,      // HFI低速
    OBSERVER_MODE_SMO = 1,      // SMO高速
    OBSERVER_MODE_SIXSTEP = 2   // 六步启动
} Observer_Mode_t;

void Platform_Motor_Init(void);
void Platform_Motor_Start(void);
void Platform_Motor_Stop(void);

static void Platform_Motor_Loop(void);
MotorStatus_t Platform_Motor_Status(void);

static void Platform_Motor_GetADCOffset(bool Status);

static void Platform_Motor_GetCurrent(void);
static void Platform_Motor_GetVoltage(void);
static float Platform_Motor_GetTemp(uint16_t ADCRawValue);

#endif //PLATFORM_MOTOR_H
