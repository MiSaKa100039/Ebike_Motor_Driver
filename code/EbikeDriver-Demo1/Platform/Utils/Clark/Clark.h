#ifndef CLARK_H
#define CLARK_H

#include "math.h"

typedef struct
{
    float Ialpha;
    float Ibeta;
}Clark_OutPut_t;

Clark_OutPut_t Clark_Calculation(float Ia, float Ib, float Ic);

#endif //CLARK_H
