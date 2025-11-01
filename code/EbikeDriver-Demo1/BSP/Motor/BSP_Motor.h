#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include "Motor_Config.h"

#ifdef EnableFunction_Motor

#include "stm32g4xx_hal.h"
#include "UserApp.h"

typedef struct {
    uint16_t Ua;
    uint16_t Ub;
    uint16_t Uc;
    uint16_t Ubus;
    // float Udc;
}ADC_RawValue_t;

// extern uint16_t ADC_Buffer[4];   //ADC采样4通道

// extern TIM_HandleTypeDef htim1;
// extern ADC_HandleTypeDef hadc1;

void BSP_Motor_Init(void);
void BSP_Motor_Start(void);
void BSP_Motor_Stop(void);

ADC_RawValue_t BSP_Motor_GetAdcRawValue(void);

uint32_t BSP_Motor_GetAutoReload(void);
void BSP_Motor_SetCompare(uint32_t CCR1, uint32_t CCR2, uint32_t CCR3);

//PER_Init
static void MX_TIM1_Init(void);
static void MX_ADC2_Init(void);
static void MX_DMA_Init(void);
//Msp_Init
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* htim_pwm);
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc);
void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc);
//NVIC Setting
void ADC1_2_IRQHandler(void);
#endif

#endif //BSP_MOTOR_H
