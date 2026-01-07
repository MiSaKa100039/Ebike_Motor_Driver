#include "BSP_VOFA.h"

void BSP_VOFA_Transmit(uint8_t *Data, uint16_t Length)
{
    // HAL_UART_Transmit(&huart4, (uint8_t*)Data, Length, HAL_MAX_DELAY);
    HAL_UART_Transmit_DMA(&huart4, (uint8_t*)Data, Length);
}