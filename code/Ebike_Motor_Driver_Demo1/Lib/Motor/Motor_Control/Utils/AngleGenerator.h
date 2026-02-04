#pragma once
#include "../FOCMath.h"

namespace Lib_Motor {

    /**
     * @brief 开环角度生成器 (带速度斜坡)
     * @details 用于 V/F 启动、I/F 强拖或调试模式。
     * 它负责把目标转速平滑地转换为电角度，模拟转子的旋转。
     */
    class AngleGenerator {
    public:
        // 复位状态 (例如从停止进入启动时调用)
        void reset() {
            angle_ = 0.0f;
            current_speed_rad_s_ = 0.0f;
        }

        float getVelocity() const { return current_speed_rad_s_; }

        /**
         * @brief 更新角度状态
         * @param target_rpm   目标转速 (RPM)
         * @param dt           控制周期 (秒)
         * @param accel_rad_s2 加速度限制 (rad/s^2)。传 0 表示无斜坡直接跳变。
         * @return 当前电角度 (-PI ~ PI)
         */
        float update(float target_rpm, float dt, float accel_rad_s2)
        {
            // 1. 单位转换: RPM -> rad/s
            float target_rad_s = target_rpm * (TWO_PI / 60.0f);

            // ================= [这里就是 Ramp 斜坡逻辑] =================
            // 作用：让 current_speed_rad_s_ 慢慢接近 target_rad_s，而不是瞬间跳变
            // 防止电机跟不上磁场变化而失步
            if (accel_rad_s2 > 0.0f) {
                float max_delta = accel_rad_s2 * dt; // 这一步允许的最大速度变化量

                if (current_speed_rad_s_ < target_rad_s) {
                    // 加速
                    current_speed_rad_s_ += max_delta;
                    if (current_speed_rad_s_ > target_rad_s) current_speed_rad_s_ = target_rad_s;
                }
                else if (current_speed_rad_s_ > target_rad_s) {
                    // 减速
                    current_speed_rad_s_ -= max_delta;
                    if (current_speed_rad_s_ < target_rad_s) current_speed_rad_s_ = target_rad_s;
                }
            } else {
                // 无斜坡，直接赋值
                current_speed_rad_s_ = target_rad_s;
            }

            // ================= [这里就是 角度积分生成] =================
            // 角度 = 速度 * 时间 (积分)
            angle_ += current_speed_rad_s_ * dt;

            // 归一化到 -PI ~ PI
            if (angle_ > 3.14159265f) angle_ -= 6.28318531f;
            if (angle_ < -3.14159265f) angle_ += 6.28318531f;

            return angle_;
        }

    private:
        float angle_ = 0.0f;               // 当前积分出来的角度
        float current_speed_rad_s_ = 0.0f; // 当前实际速度 (经过斜坡处理后的)
    };
}