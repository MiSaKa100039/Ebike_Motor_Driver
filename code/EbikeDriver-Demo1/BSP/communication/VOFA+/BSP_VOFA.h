#ifndef BSP_VOFA_H
#define BSP_VOFA_H

#include "VOFA_Config.h"

#ifdef EnableFunction_VOFA

#include "stm32g4xx_hal.h"
#include "UserApp.h"

extern  UART_HandleTypeDef huart3;
// DMA_HandleTypeDef hdma_usart3_tx;

void BSP_VOFA_Init(void);
void BSP_VOFA_Transmit(uint8_t *Data, uint16_t Length);

//PER_Init
static void MX_UART4_UART_Init(void);
static void MX_DMA_Init(void);
//MspInit
void HAL_UART_MspInit(UART_HandleTypeDef* huart);
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart);
//NVIC Setting
void DMA1_Channel1_IRQHandler(void);
void UART4_IRQHandler(void);
#endif

#endif //BSP_VOFA_H
