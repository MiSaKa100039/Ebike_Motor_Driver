#ifndef PLATFORM_LED_H
#define PLATFORM_LED_H

#include "LED_Config.h"
#include "main.h"

#if EnableFunction_LED == 1

void Platform_LED_Init(GPIO_TypeDef  *LED_Port , uint16_t LED_Pin , uint8_t LED_State);
void Platform_LED_SetState(GPIO_TypeDef  *LED_Port , uint16_t LED_Pin, uint8_t LED_State);

#endif

#endif
