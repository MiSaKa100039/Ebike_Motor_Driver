#ifndef PLATFORM_VOFA_H
#define PLATFORM_VOFA_H

#include "VOFA_Config.h"
#include "BSP_VOFA.h"
#include "string.h"

#ifdef EnableFunction_VOFA

void Platform_VOFA_Init(void);
void Platform_VOFA_SendFloat(float *Data, uint8_t Num);
void Platform_VOFA_SendRawData(uint16_t Data);

#endif

#endif //PLATFORM_VOFA_H
