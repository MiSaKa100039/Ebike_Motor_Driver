#ifndef PLATFORM_VOFA_H
#define PLATFORM_VOFA_H

#include "BSP_VOFA.h"
#include <stdio.h>
#include <string.h>

extern float vofa_data [8];

void Platform_VOFA_SendFloat(float *Data, uint8_t Num);

#endif //PLATFORM_VOFA_H
