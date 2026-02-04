#pragma once
#include "FOCMath.h"

namespace Lib_Motor {

    class SVPWM {
    public:
        static void Calculate(float v_alpha, float v_beta, float v_bus,
                              float* out_da, float* out_db, float* out_dc)
        {
            if (v_bus < 0.1f) {
                *out_da = *out_db = *out_dc = 0.0f;
                return;
            }

            float v_a = v_alpha;
            float v_b = -0.5f * v_alpha + SQRT3_DIV_2 * v_beta;
            float v_c = -0.5f * v_alpha - SQRT3_DIV_2 * v_beta;

            float v_max = v_a;
            float v_min = v_a;

            if (v_b > v_max) v_max = v_b;
            if (v_c > v_max) v_max = v_c;

            if (v_b < v_min) v_min = v_b;
            if (v_c < v_min) v_min = v_c;

            float v_offset = -0.5f * (v_max + v_min);

            *out_da = MotorMath::Clamp((v_a + v_offset) / v_bus + 0.5f, 0.0f, 1.0f);
            *out_db = MotorMath::Clamp((v_b + v_offset) / v_bus + 0.5f, 0.0f, 1.0f);
            *out_dc = MotorMath::Clamp((v_c + v_offset) / v_bus + 0.5f, 0.0f, 1.0f);
        }
    };
}