#include "Park.h"

/**
 * Park逆变换
 * @param Ud 输入d轴电压
 * @param Uq 输入q轴电压
 * @param Theta 电角度
 * @return 返回Ualpha，Ubeta
 */
InvPark_OutPut_t InvPark_Calculation(float Ud, float Uq, float Theta)
{
    InvPark_OutPut_t InvPark_OutPut = {0};

    InvPark_OutPut.Ualpha = Ud * cosf(Theta) - Uq * sinf(Theta);
    InvPark_OutPut.Ubeta = Ud * sinf(Theta) + Uq * cosf(Theta);

    return InvPark_OutPut;
}

Park_OutPut_t Park_Calculation(float Ialpha, float Ibeta, float Theta)
{
    Park_OutPut_t Park_OutPut = {0};

    Park_OutPut.Id = Ialpha * cosf(Theta) + Ibeta * sinf(Theta);
    Park_OutPut.Iq = - Ialpha * sinf(Theta) + Ibeta * cosf(Theta);

    return Park_OutPut;
}