#include "Platform_VOFA.h"

float vofa_data [8];

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
