#include "Platform_LED.h"

#if EnableFunction_LED == 1

void Platform_LED_Init(GPIO_TypeDef  *LED_Port , uint16_t LED_Pin , uint8_t LED_State)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (LED_Port == GPIOA) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    } else if (LED_Port == GPIOB) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
    } else if (LED_Port == GPIOC) {
        __HAL_RCC_GPIOC_CLK_ENABLE();
    } else if (LED_Port == GPIOD) {
        __HAL_RCC_GPIOD_CLK_ENABLE();
    } else if (LED_Port == GPIOE) {
        __HAL_RCC_GPIOE_CLK_ENABLE();
    }

    GPIO_InitStruct.Pin = LED_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(LED_Port, &GPIO_InitStruct);

    if (LED_State == LED_ON)
    {
        HAL_GPIO_WritePin(LED_Port, LED_Pin, GPIO_PIN_SET);
    }
    else if (LED_State == LED_OFF)
    {
        HAL_GPIO_WritePin(LED_Port, LED_Pin, GPIO_PIN_RESET);
    }
}


void Platform_LED_SetState(GPIO_TypeDef  *LED_Port , uint16_t LED_Pin, uint8_t LED_State)
{
    switch (LED_State)
    {
        case LED_ON:
            HAL_GPIO_WritePin(LED_Port, LED_Pin, GPIO_PIN_SET);break;
        case LED_OFF:
            HAL_GPIO_WritePin(LED_Port, LED_Pin, GPIO_PIN_RESET);break;
        case LED_TOGGLE:
            HAL_GPIO_TogglePin(LED_Port, LED_Pin);break;
        default:break;
    }
}

#endif
