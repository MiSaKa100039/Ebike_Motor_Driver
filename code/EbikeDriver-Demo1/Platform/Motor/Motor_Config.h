#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

#define EnableFunction_Motor  1

#if EnableFunction_Motor

#define Tpwm  (1.0f / 20000.0f)   //PWM频率
#define Udc 24.0f               //母线电压
#define Vref 3.3f               //参考电压
#define Vref_Offset 1.65f       //电压偏置
#define GAIN 20                 //运放增益
#define NumberOfPolePairs   7

#define PI 3.14159265358979323846f
#define Sqrt3   1.73205080757f
#define UrefMax (Sqrt3 / 3 * Udc)

#endif

#endif //MOTOR_CONFIG_H
