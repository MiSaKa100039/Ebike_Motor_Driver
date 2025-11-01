#ifndef PLATFORM_MOTOR_H
#define PLATFORM_MOTOR_H

#include "Motor_Config.h"

#ifdef EnableFunction_Motor

#include "BSP_Motor.h"
#include "SVPWM.h"
#include "Park.h"
#include "Clark.h"
#include "PID.h"

typedef struct
{
    uint32_t CCRa;
    uint32_t CCRb;
    uint32_t CCRc;
} PwmCCR_OutPut_t;

typedef struct
{
    float Ia;
    float Ib;
    float Ic;
    float Ibus;
}Shunt_Current_t;

extern float vofa_data [8];
extern uint16_t vofa_raw_data;

void Platform_Motor_Init(void);
void Platform_Motor_Start(void);
void Platform_Motor_Stop(void);
void Platform_Motor_Loop(void);

#endif //PLAT

#endif //PLATFORM_MOTOR_H
