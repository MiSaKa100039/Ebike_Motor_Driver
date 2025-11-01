#include "BSP_LED.h"

#if EnableFunction_LED == 1

void BSP_LED_Init(void)
{
    Platform_LED_Init(GPIOC,GPIO_PIN_13,LED_OFF);
    Platform_LED_Init(GPIOC,GPIO_PIN_14,LED_OFF);
    Platform_LED_Init(GPIOC,GPIO_PIN_15,LED_OFF);
}

void BSP_LED_SetState(GPIO_TypeDef  *LED_Port , uint16_t LED_Pin, uint8_t LED_State)
{
    Platform_LED_SetState(LED_Port, LED_Pin, LED_State);
}

#endif