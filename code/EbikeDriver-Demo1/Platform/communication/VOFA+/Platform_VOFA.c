#include "Platform_VOFA.h"

#include <stdio.h>

void Platform_VOFA_Init(void)
{
    BSP_VOFA_Init();
}

void Platform_VOFA_SendFloat(float *Data, uint8_t Num)
{
    if (Data == NULL || Num == 0) return;

    static uint8_t bufA[64];
    static uint8_t bufB[64];
    static uint8_t useA = 0;

    uint8_t *buf = useA ? bufA : bufB;
    useA = !useA;

    uint16_t len = 4 * Num + 4;

    if (len > sizeof(bufA)) return;  // 安全检查

    // 按字节复制 float 数据
    memcpy(buf, (uint8_t*)Data, 4 * Num);

    // 添加 VOFA 帧尾
    buf[4 * Num + 0] = 0x00;
    buf[4 * Num + 1] = 0x00;
    buf[4 * Num + 2] = 0x80;
    buf[4 * Num + 3] = 0x7F;

    BSP_VOFA_Transmit(buf,len);
}

uint8_t uart_dma_busy = 0;  // 全局标志

void Platform_VOFA_SendRawData(uint16_t Data)
{
    char buf[16];
    sprintf(buf, "%u\r\n", Data);
    uint16_t Length = strlen(buf);

    if (!uart_dma_busy)
    {
        uart_dma_busy = 1;
        HAL_UART_Transmit_DMA(&huart3, (uint8_t*)buf, Length);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        uart_dma_busy = 0;  // 完成
    }
}

// // 发送前检查
// if (uart_dma_busy) return;  // 上次未完，跳过
// uart_dma_busy = 1;
// HAL_UART_Transmit_DMA(&huart3, Data, Length);