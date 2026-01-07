#include "BSP_Motor.h"

// uint16_t ADC1_DMA_Buffer[1] = {0};
// uint16_t ADC2_DMA_Buffer[2] = {0};

void BSP_Motor_Init(void)
{
    HAL_TIM_Base_Start(&htim1);
    // HAL_TIM_Base_Start(&htim2);

    // HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

    // HAL_ADC_Start_IT(&hadc1);
    HAL_ADCEx_InjectedStart_IT(&hadc2);

    // HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC1_DMA_Buffer, 1);
    // HAL_ADC_Start_DMA(&hadc2, (uint32_t*)ADC2_DMA_Buffer, 3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 4000);

}

void Bsp_Motor_Start(void)
{
    HAL_TIM_Base_Start(&htim1);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 4000);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

}

void Bsp_Motor_Stop(void)
{
    /* 停止TIM1时间基 */
    HAL_TIM_Base_Stop(&htim1);

    // 停止 PWM 和计数
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);

    // HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
}

uint32_t BSP_Motor_GetAutoReload(void)
{
    uint32_t ARR = __HAL_TIM_GET_AUTORELOAD(&htim1);
    return ARR;
}

//设置ARR值
void BSP_Motor_SetCompare(uint32_t CCR1, uint32_t CCR2, uint32_t CCR3)
{
    // 设置CCR寄存器
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, CCR1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, CCR2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, CCR3);
}

ADCRawValue_t Bsp_Motor_GetADCRawValue(uint8_t Type)
{
    ADCRawValue_t ADCRawValue = {0};

    switch(Type)
    {
        case Injected:
            ADCRawValue.ShuntA = HAL_ADCEx_InjectedGetValue(&hadc2,ADC_INJECTED_RANK_1);
            ADCRawValue.ShuntB = HAL_ADCEx_InjectedGetValue(&hadc2,ADC_INJECTED_RANK_2);
            ADCRawValue.ShuntC = HAL_ADCEx_InjectedGetValue(&hadc2,ADC_INJECTED_RANK_3);
            ADCRawValue.ShuntBus = HAL_ADCEx_InjectedGetValue(&hadc2,ADC_INJECTED_RANK_4);
            break ;
        case Regular:
            // ADCRawValue.Udc = ADC1_DMA_Buffer[0];
            break ;
        default: break ;
    }

    return ADCRawValue;
}