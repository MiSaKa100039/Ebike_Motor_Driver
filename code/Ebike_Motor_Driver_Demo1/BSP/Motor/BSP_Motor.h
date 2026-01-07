#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include <sys/types.h>

#include "main.h"

typedef struct
{
    uint16_t ShuntA;
    uint16_t ShuntB;
    uint16_t ShuntC;
    uint16_t ShuntBus;
    uint16_t NTC1;
    uint16_t NTC2;
    uint16_t Udc;
} ADCRawValue_t;

#define Injected 0
#define Regular 1

#define PI 3.14159265358979323846f

void BSP_Motor_Init(void);
void Bsp_Motor_Start(void);
void Bsp_Motor_Stop(void);

uint32_t BSP_Motor_GetAutoReload(void);
void BSP_Motor_SetCompare(uint32_t CCR1, uint32_t CCR2, uint32_t CCR3);

ADCRawValue_t Bsp_Motor_GetADCRawValue(uint8_t Type);

#endif //BSP_MOTOR_H
