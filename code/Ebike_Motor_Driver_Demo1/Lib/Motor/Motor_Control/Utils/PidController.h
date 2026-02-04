#pragma once
#include "../FOCMath.h"

namespace Lib_Motor {

    class PidController {
    public:
        void reset() { integral_ = 0.0f; }

// [核心] update 函数接收实时参数
        float update(float error, float dt, float kp, float ki, float limit) {
            float p_out = kp * error;
            integral_ += ki * error * dt;

            // 积分抗饱和 (Clamp)
            // 注意：这里用 limit 来限制积分项是个好习惯
            integral_ = MotorMath::Clamp(integral_, -limit, limit);

            // 总输出
            float output = p_out + integral_;
            return MotorMath::Clamp(output, -limit, limit);
        }

    private:
        float integral_ = 0.0f;
    };
}