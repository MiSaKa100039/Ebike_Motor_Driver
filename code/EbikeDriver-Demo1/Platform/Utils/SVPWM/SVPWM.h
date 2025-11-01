#ifndef SVPWM_H
#define SVPWM_H

#include <stdint.h>
#include <math.h>
#include <sys/types.h>

#include "Motor_Config.h"

typedef struct {
    float Ta;
    float Tb;
    float Tc;
}SVPWM_OutPut_t;

SVPWM_OutPut_t SVPWM_Calculation(float Ualpha, float Ubeta);

#endif //SVPWM_H

