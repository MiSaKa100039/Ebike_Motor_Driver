#ifndef PARK_H
#define PARK_H

#include "math.h"
#include "Motor_Config.h"

typedef struct
{
    float Ualpha;
    float Ubeta;
}InvPark_OutPut_t;

typedef struct
{
    float Id;
    float Iq;
}Park_OutPut_t;

InvPark_OutPut_t InvPark_Calculation(float Ud, float Uq, float Theta);
Park_OutPut_t Park_Calculation(float Ialpha, float Ibeta, float Theta);

#endif //PARK_H
