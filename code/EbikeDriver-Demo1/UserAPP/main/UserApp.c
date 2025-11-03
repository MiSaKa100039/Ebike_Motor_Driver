#include "UserApp.h"
#include "stdio.h"

void init(void)
{
    HAL_Delay(1000);

    Platform_VOFA_Init();
    Platform_Motor_Init();

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
