#include "FOC.h"

InvPark_Output_t InvPark_Calculation(float Tar_Ud, float Tar_Uq, float Theta)
{
    InvPark_Output_t InvPark_OutPut = {0};

    InvPark_OutPut.Ualpha = Tar_Ud * cosf(Theta) - Tar_Uq * sinf(Theta);
    InvPark_OutPut.Ubeta = Tar_Ud * sinf(Theta) + Tar_Uq * cosf(Theta);

    return InvPark_OutPut;
}

Park_Output_t Park_Calculation(float Ialpha, float Ibeta, float Theta)
{
    Park_Output_t Park_OutPut = {0};

    Park_OutPut.Id = Ialpha * cosf(Theta) + Ibeta * sinf(Theta);
    Park_OutPut.Iq = - Ialpha * sinf(Theta) + Ibeta * cosf(Theta);

    return Park_OutPut;
}