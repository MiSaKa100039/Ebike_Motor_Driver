#include "UserAPP.h"

void Init(void)
{
    Platform_Motor_Init();
    Platform_Motor_Start();

    // HAL_Delay(5000);
    // Platform_Motor_Stop();
}

void loop(void)
{
    // Platform_VOFA_SendFloat(vofa_data,4);
    // HAL_Delay(10);
}
