#ifndef BSP_LED_H
#define BSP_LED_H

#include "LED_Config.h"
#include "Platform_LED.h"

#if EnableFunction_LED == 1

void BSP_LED_Init(void);
void BSP_LED_SetState(GPIO_TypeDef  *LED_Port , uint16_t LED_Pin, uint8_t LED_State);

#endif

#endif //BSP_LED_H
