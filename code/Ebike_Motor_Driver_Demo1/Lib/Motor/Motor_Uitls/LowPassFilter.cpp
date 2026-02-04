#include "LowPassFilter.h"

namespace Lib_Motor {

    LowPassFilter::LowPassFilter(float time_constant_s)
        : tf_(time_constant_s), y_prev_(0.0f) {
    }

    float LowPassFilter::operator()(float input, float dt) {
        // 算法核心：一阶低通滤波
        // alpha = dt / (Tf + dt)
        // 这种计算方式比 alpha = dt/Tf 更稳定，能防止 dt > Tf 时发散
        float alpha = dt / (tf_ + dt);

        // 滤波公式: out = prev + alpha * (in - prev)
        float output = y_prev_ + alpha * (input - y_prev_);

        y_prev_ = output;
        return output;
    }

    void LowPassFilter::reset(float val) {
        y_prev_ = val;
    }

    void LowPassFilter::setTimeConstant(float time_constant_s) {
        tf_ = time_constant_s;
    }

    float LowPassFilter::getValue() const {
        return y_prev_;
    }
}