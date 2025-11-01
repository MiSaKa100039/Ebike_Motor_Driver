#include "UserApp.h"
#include "stdio.h"

void init(void)
{
    HAL_Delay(1000);

    Platform_VOFA_Init();
    Platform_Motor_Init();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    Platform_Motor_Start();
}

void loop(void)
{
    // Platform_Motor_Loop();
    //
    // Platform_VOFA_SendFloat(vofa_data, 1);
    // HAL_Delay(10);
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC2)
    {
        Platform_Motor_Loop();
        Platform_VOFA_SendFloat(vofa_data, 4);
        // HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_5);
    }
}

void User_Error_Handler(void)
{

}
