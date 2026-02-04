#pragma once
#include <math.h>

namespace Lib_Motor {

    constexpr float SQRT3_DIV_2 = 0.86602540378f;
    constexpr float ONE_DIV_SQRT3 = 0.57735026919f;
    constexpr float TWO_PI = 6.28318530718f;

    class MotorMath {
    public:
        static inline float Clamp(float val, float min, float max) {
            if (val > max) return max;
            if (val < min) return min;
            return val;
        }

        // [新增] Clark 变换 (3相 -> 2相静止)
        // 输入: Ia, Ib (Ic = -Ia - Ib)
        // 输出: I_alpha, I_beta
        static inline void Clark(float i_a, float i_b, float* i_alpha, float* i_beta) {
            *i_alpha = i_a;
            *i_beta  = (i_a + 2.0f * i_b) * ONE_DIV_SQRT3;
        }

        // [新增] Park 变换 (2相静止 -> 2相旋转)
        // 输入: I_alpha, I_beta, theta (电角度)
        // 输出: Id, Iq
        static inline void Park(float i_alpha, float i_beta, float theta, float* i_d, float* i_q) {
            float c = cosf(theta);
            float s = sinf(theta);
            *i_d = i_alpha * c + i_beta * s;
            *i_q = -i_alpha * s + i_beta * c;
        }

        // [现有] 反 Park 变换
        static inline void InvPark(float v_d, float v_q, float theta, float* v_alpha, float* v_beta) {
            float c = cosf(theta);
            float s = sinf(theta);
            *v_alpha = v_d * c - v_q * s;
            *v_beta  = v_d * s + v_q * c;
        }
    };
}