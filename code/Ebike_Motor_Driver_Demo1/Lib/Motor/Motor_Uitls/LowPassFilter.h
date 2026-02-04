#pragma once

namespace Lib_Motor {

    class LowPassFilter {
    public:
        // 构造函数：传入时间常数 tf (单位秒)
        explicit LowPassFilter(float time_constant_s);

        // 核心计算函数：输入新值 input，传入采样周期 dt
        float operator()(float input, float dt);

        // 重置滤波器
        void reset(float val = 0.0f);

        // 修改时间常数
        void setTimeConstant(float time_constant_s);

        // 获取当前值
        float getValue() const;

    private:
        float tf_;      // 时间常数
        float y_prev_;  // 上一次的输出值
    };
}