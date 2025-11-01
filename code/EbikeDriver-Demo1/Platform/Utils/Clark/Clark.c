#include "Clark.h"

Clark_OutPut_t Clark_Calculation(float Ia, float Ib, float Ic)
{
    Clark_OutPut_t Clark_OutPut = {0};

    Clark_OutPut.Ialpha = Ia;
    Clark_OutPut.Ibeta = (Ia + 2.0f * Ib) / sqrtf(3.0f);

    return Clark_OutPut;
}