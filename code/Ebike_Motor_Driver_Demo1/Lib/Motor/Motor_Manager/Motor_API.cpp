#include "../Motor_Public/Motor_API.h"
#include "../Motor_Manager/Motor_Manage.h"

namespace Lib_Motor
{
    // 静态实例池
    static MotorManager* instances[4] = { nullptr };
    static int instance_count = 0;

    MotorAPI::MotorAPI() : id_(-1) {}

    Result MotorAPI::init(const MotorConfig& cfg)
    {
        if (instance_count >= 4) return Result::Error;
        instances[instance_count] = new MotorManager(cfg);
        instances[instance_count]->init();
        this->id_ = instance_count;
        instance_count++;
        return Result::Ok;
    }

    // ==============================================================================
    // [Group 1] 基础启停与状态清除
    // ==============================================================================

    Result MotorAPI::start()
    {
        if (id_ < 0 || !instances[id_]) return Result::InvalidHandle;
        State s = instances[id_]->state();
        if (s == State::RUN) return Result::Ok;
        if (s == State::ERROR) return Result::InvalidState;

        // 默认进入速度模式
        instances[id_]->setMode(Mode::VELOCITY_CONTROL);
        return Result::Ok;
    }

    Result MotorAPI::stop()
    {
        if (id_ < 0 || !instances[id_]) return Result::InvalidHandle;
        instances[id_]->setMode(Mode::NONE); // FSM 会自动处理切入 STOP
        return Result::Ok;
    }

    Result MotorAPI::clearFault()
    {
        if (id_ < 0 || !instances[id_]) return Result::InvalidHandle;
        // 仅在 ERROR 下有效
        if (instances[id_]->state() == State::ERROR) {
            instances[id_]->reset();
            return Result::Ok;
        }
        return Result::InvalidState;
    }

    // 2. [新增] 实现强制 reset
    Result MotorAPI::reset()
    {
        if (id_ < 0 || !instances[id_]) return Result::InvalidHandle;

        instances[id_]->reset(); // 再复位内部变量

        return Result::Ok;
    }

    // ==============================================================================
    // [Group 2] 闭环运动控制
    // ==============================================================================

    Result MotorAPI::setTargetTorque(float current)
    {
        if (id_ < 0 || !instances[id_]) return Result::InvalidHandle;

        // 1. 安全检查
        if (checkSafeModeChange(id_, Mode::TORQUE_CONTROL) != Result::Ok)
            return Result::InvalidState;

        // 2. 设定参数
        instances[id_]->setTargetTorque(current);
        instances[id_]->setMode(Mode::TORQUE_CONTROL);

        return Result::Ok;
    }

    Result MotorAPI::setTargetSpeed(float rpm)
    {
        if (id_ < 0 || !instances[id_]) return Result::InvalidHandle;

        // 1. 安全检查
        if (checkSafeModeChange(id_, Mode::VELOCITY_CONTROL) != Result::Ok)
            return Result::InvalidState;

        // 2. 设定参数
        instances[id_]->setTargetSpeed(rpm);
        instances[id_]->setMode(Mode::VELOCITY_CONTROL);

        return Result::Ok;
    }

    Result MotorAPI::setTargetPosition(float angle)
    {
        if (id_ < 0 || !instances[id_]) return Result::InvalidHandle;

        // 1. 安全检查
        if (checkSafeModeChange(id_, Mode::POSITION_CONTROL) != Result::Ok)
            return Result::InvalidState;

        // 2. 设定参数
        // TODO: 待实现位置模式
        instances[id_]->setMode(Mode::POSITION_CONTROL);

        return Result::Error;
    }

    // ==============================================================================
    // [Group 3] 生产与校准
    // ==============================================================================

    Result MotorAPI::startRLIdentification()
    {
        if (id_ < 0 || !instances[id_]) return Result::InvalidHandle;

        // 使用统一的安全检查
        if (checkSafeModeChange(id_, Mode::CALIB_RL_IDENTIFY) != Result::Ok) {
            return Result::InvalidState;
        }

        instances[id_]->setMode(Mode::CALIB_RL_IDENTIFY);

        return Result::Ok;
    }

    // ==============================================================================
    // [Group 4] 研发调试接口 (Debug Only)
    // ==============================================================================
    #ifdef ENABLE_DANGEROUS_TEST_API

    Result MotorAPI::debugPWMManual(float u, float v, float w)
    {
        if (id_ < 0 || !instances[id_]) return Result::InvalidHandle;

        // 使用统一的安全检查
        if (checkSafeModeChange(id_, Mode::DEBUG_PWM_MANUAL) != Result::Ok) {
            return Result::InvalidState;
        }

        // 简单限幅
        if (u > 0.95f) u = 0.95f; if (u < 0.0f) u = 0.0f;
        if (v > 0.95f) v = 0.95f; if (v < 0.0f) v = 0.0f;
        if (w > 0.95f) w = 0.95f; if (w < 0.0f) w = 0.0f;

        instances[id_]->setRawPWM(u, v, w); // 内部已包含 setMode

        return Result::Ok;
    }

    Result MotorAPI::debugCurrentLock(float i_d, float i_q)
    {
        if (id_ < 0 || !instances[id_]) return Result::InvalidHandle;

        // 使用统一的安全检查
        if (checkSafeModeChange(id_, Mode::DEBUG_CURRENT_LOCK) != Result::Ok) {
            return Result::InvalidState;
        }

        instances[id_]->setTargetId(i_d);     // 设 Id
        instances[id_]->setTargetTorque(i_q); // 设 Iq (复用 Torque 接口)
        instances[id_]->setMode(Mode::DEBUG_CURRENT_LOCK);

        return Result::Ok;
    }

    // 接口定义：目标转速(RPM), Id(A), Iq(A), 爬坡时间(s)
    Result MotorAPI::debugIFControl(float target_rpm, float id_amp, float iq_amp, float ramp_time_s)
    {
        if (id_ < 0 || !instances[id_]) return Result::InvalidHandle;

        // 1. 安全检查
        if (checkSafeModeChange(id_, Mode::DEBUG_IF_DRAG) != Result::Ok) {
            return Result::InvalidState;
        }

        auto* mgr = instances[id_];

        // 2. 设定电流目标 (支持双轴设置)
        mgr->setTargetId(id_amp);     // 设置 D 轴
        mgr->setTargetTorque(iq_amp); // 设置 Q 轴 (复用 Torque 接口)
        mgr->setTargetSpeed(target_rpm);

        // 3. [核心] 自动计算加速度 (RPM/s -> rad/s^2)
        // 加速度 = 速度变化量 / 时间
        float accel_rad_s2 = 0.0f;

        if (ramp_time_s < 0.001f) {
            // 时间极短 -> 无穷大加速度 (阶跃)
            accel_rad_s2 = 0.0f; // AngleGenerator 约定 0 为直接赋值
        } else {
            // 计算目标角速度 (绝对值)
            float target_rad_s = fabsf(target_rpm) * 0.10472f; // RPM -> rad/s

            // 计算加速度
            accel_rad_s2 = target_rad_s / ramp_time_s;
        }

        // 4. 将计算好的加速度传给 Manager
        mgr->setDebugAccel(accel_rad_s2);

        // 5. 切换模式
        mgr->setMode(Mode::DEBUG_IF_DRAG);

        return Result::Ok;
    }

    Result MotorAPI::debugVFControl(float duty, float speed_rpm, float ramp_time_s)
    {
        if (id_ < 0 || !instances[id_]) return Result::InvalidHandle;

        // 安全检查...

        // 1. 设置目标
        instances[id_]->setTargetId(duty);     // 复用 Id 存 Duty
        instances[id_]->setTargetSpeed(speed_rpm);

        // 2. [关键] 计算加速度并传给 Manager
        // 如果时间极短，给一个巨大的加速度(视为阶跃)
        float accel = 0.0f;
        if (ramp_time_s < 0.001f) {
            accel = 0.0f; // 0 代表无限制 (AngleGenerator 的逻辑)
        } else {
            // 加速度 = 速度差 / 时间
            // 注意：这里简单用 目标速度/时间 计算，假定从0启动。
            // 如果想更严谨，Manager 内部处理会更好，但在 API 层估算通常够用了。
            float target_rad_s = fabsf(speed_rpm) * 0.10472f;
            accel = target_rad_s / ramp_time_s;
        }

        instances[id_]->setDebugAccel(accel); // 需要去 Manager 加这个 setter

        instances[id_]->setMode(Mode::DEBUG_VF_DRAG);

        return Result::Ok;
    }

    #endif

    // ==============================================================================
    // [Group 5] 监视与数据获取
    // ==============================================================================

    void MotorAPI::getMonitorData(MotorMonitorData& out_data) const
    {
        if (id_ < 0 || !instances[id_]) {
            out_data = {};
            return;
        }
        instances[id_]->getMonitorData(out_data);
    }

    State MotorAPI::getState() const
    {
        if (id_ < 0 || !instances[id_]) return State::INIT;
        return instances[id_]->state();
    }

    float MotorAPI::getSpeed() const
    {
        if (id_ < 0 || !instances[id_]) return 0.0f;
        // 需 Manager 实现 getter
        return instances[id_]->getSpeed();
    }

    // 增加一个辅助判断：是否是调试/校准模式
    bool MotorAPI:: isDebugOrCalib(Mode m) {
        return (m == Mode::DEBUG_PWM_MANUAL || m == Mode::DEBUG_CURRENT_LOCK ||
                m == Mode::DEBUG_IF_DRAG || m == Mode::DEBUG_VF_DRAG ||
                m == Mode::CALIB_RL_IDENTIFY); // RL 也是一种特殊的“独占”模式
    }

    Result MotorAPI::checkSafeModeChange(int id, Mode new_mode) {
        auto& m = instances[id];
        if (m->fault() != Fault::NONE) return Result::InvalidState;
        if (m->state() != State::RUN) return Result::Ok;

        Mode current = m->mode();
        if (current == new_mode) return Result::Ok;

        // [isDebugOrCalib]
        // 1. 如果当前是调试/校准模式，严禁直接切到任何其他模式 (必须先 Stop)
        if (isDebugOrCalib(current)) return Result::InvalidState;

        // 2. 如果要切去调试/校准模式，也严禁 (必须先 Stop)
        if (isDebugOrCalib(new_mode)) return Result::InvalidState;

        // 3. 剩下的就是 闭环切闭环 (Speed <-> Torque)，允许
        return Result::Ok;
    }

}

// ==============================================================================
// ISR 入口
// ==============================================================================
extern "C" void Motor_Global_Process_Handler(int motor_id)
{
    if (motor_id < 0 || motor_id >= 4) return;
    if (Lib_Motor::instances[motor_id]) {
        Lib_Motor::instances[motor_id]->tick();
    }
}