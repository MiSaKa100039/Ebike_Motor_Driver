#include "SVPWM.h"

/**
 * @param Ualpha
 * @param Ubeta
 * @return
 */
SVPWM_OutPut_t SVPWM_Calculation(float Ualpha, float Ubeta)
{
  SVPWM_OutPut_t SVPWM_OutPut = {0};

  uint8_t Sector;

  float Uref, Theta, Theta1;
  float T1, T2, T3, T4, T5, T6, T0;
  float Ta, Tb, Tc, Ta1, Tb1, Tc1;

  Uref = sqrtf(Ualpha * Ualpha + Ubeta * Ubeta);
  if (Uref > UrefMax) Uref = UrefMax;

  Theta = atan2f(Ubeta , Ualpha);
  if (Theta < 0) Theta = Theta + 2.0f * PI;

  Sector = (int)(Theta / (PI / 3.0f)) + 1;
  if (Sector > 6) Sector = 6;

  Theta1 = Theta - (Sector - 1) * (PI / 3.0f);

  float K = (Sqrt3 * Tpwm) / Udc * Uref;

  switch (Sector)
  {
    case 1:
      T4 = K * sinf((PI / 3) - Theta1);
      T6 = K * sinf(Theta1);
      T0 = Tpwm - T4 - T6;

      Ta = T4 + T6 + (T0 / 2);
      Tb = T6 + (T0 / 2);
      Tc = T0 /2; break;
    case 2:
      T6 = K * sinf((PI / 3) - Theta1);
      T2 = K * sinf(Theta1);
      T0 = Tpwm - T6 - T2;

      Ta = T6 + (T0 / 2);
      Tb = T2 + T6 + (T0 / 2);
      Tc = T0 / 2; break;
    case 3:
      T2 = K * sinf((PI / 3) - Theta1);
      T3 = K * sinf(Theta1);
      T0 = Tpwm - T2 - T3;

      Ta = T0 / 2;
      Tb = T2 + T3 + (T0 / 2);
      Tc = T3 + (T0 / 2); break;
    case 4:
      T3 = K * sinf((PI / 3) - Theta1);
      T1 = K * sinf(Theta1);
      T0 = Tpwm - T3 - T1;

      Ta = T0 / 2;
      Tb =T3 + (T0 / 2);
      Tc =T1 + T3 + (T0 / 2); break;
    case 5:
      T1 = K * sinf((PI / 3) - Theta1);
      T5 = K * sinf(Theta1);
      T0 = Tpwm - T1 - T5;

      Ta = T5 + (T0 / 2);
      Tb = T0 / 2;
      Tc = T1 + T5 + (T0 / 2); break;
    case 6:
      T5 = K * sinf((PI / 3) - Theta1);
      T4 = K * sinf(Theta1);
      T0 = Tpwm - T5- T4;

      Ta = T4 + T5 + (T0 / 2);
      Tb = T0 / 2;
      Tc = T5 + (T0 / 2); break;
  }

  Ta1 = (Tpwm - Ta) / 2;
  Tb1 = (Tpwm - Tb) / 2;
  Tc1 = (Tpwm - Tc) / 2;

  SVPWM_OutPut.Ta = Ta1;
  SVPWM_OutPut.Tb = Tb1;
  SVPWM_OutPut.Tc = Tc1;

  return SVPWM_OutPut;
}


