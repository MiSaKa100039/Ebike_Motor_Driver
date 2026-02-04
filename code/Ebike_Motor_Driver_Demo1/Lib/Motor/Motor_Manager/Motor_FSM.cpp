#include "Motor_FSM.h"
#include "Motor_Manage.h"
#include <cmath>

namespace Lib_Motor
{
    // =============================================================
    // [关键修复] 必须把 step 函数的完整代码写在这里！
    // =============================================================
    void Motor_FSM::step(MotorManager& m)
    {
        // 1. 全局最高优先级故障检查
        if (m.fault() != Fault::NONE) {
            if (m.state() != State::ERROR) {
                m.setState(State::ERROR);
            }
        }

        // 2. 状态分发
        switch (m.state()) {
            case State::INIT:    handleInit(m);   break;
            case State::ADC_CAL: handleAdcCal(m); break;
            case State::STOP:    handleStop(m);   break;
            case State::RUN:     handleRun(m);    break;
            case State::ERROR:   handleError(m);  break;
            default:             handleInit(m);   break;
        }
    }

    // =============================================================
    // 其他状态处理逻辑
    // =============================================================

    /* --- INIT: 初始化 --- */
    void Motor_FSM::handleInit(MotorManager& m)
    {
        m.ctx_.calib_counter = 0;
        m.setState(State::ADC_CAL);
    }

    /* --- ADC_CAL: 电流零点校准 --- */
    void Motor_FSM::handleAdcCal(MotorManager& m)
    {
        // Manager 的 runAdcCalibration 负责计数和重置 counter
        // FSM 负责监测结果并切换状态
        if (m.ctx_.calib_counter >= 1000) {
            m.setState(State::STOP);
        }
    }

    /* --- STOP: 待机 --- */
    void Motor_FSM::handleStop(MotorManager& m)
    {
        Mode target = m.targetMode();

        // 检查是否有启动请求
        bool is_debug = (target == Mode::DEBUG_PWM_MANUAL ||
                         target == Mode::DEBUG_CURRENT_LOCK ||
                         target == Mode::DEBUG_IF_DRAG ||
                         target == Mode::DEBUG_VF_DRAG);

        bool is_control = (target == Mode::VELOCITY_CONTROL ||
                           target == Mode::TORQUE_CONTROL ||
                           target == Mode::POSITION_CONTROL);

        // [新增] 允许 RL 辨识模式启动
        bool is_calib = (target == Mode::CALIB_RL_IDENTIFY);

        if (is_control || is_debug || is_calib)
        {
            m.setState(State::RUN);

            // 进入 RUN 瞬间，重置状态机计时器
            m.ctx_.fsm_timer_ticks = 0;

            if (is_debug) {
                m.setRunPhase(RunPhase::RUN_DIRECT);
            }
            else {
                auto strategy = m.config_.observer.strategy;
                switch (strategy) {
                    case ObserverStrategy::FORCE_DRAG_TO_SMO:
                        m.setRunPhase(RunPhase::ALIGNMENT);
                        break;
                    default:
                        m.setRunPhase(RunPhase::ALIGNMENT);
                        break;
                }
            }
        }
    }

    /* --- RUN: 运行 --- */
    void Motor_FSM::handleRun(MotorManager& m)
    {
        // 1. 用户关机检查
        if (m.targetMode() == Mode::NONE) {
            m.setState(State::STOP);
            return;
        }

        // 2. 运行阶段流转
        const auto& obs_cfg = m.config_.observer;

        switch (m.runPhase())
        {
            case RunPhase::RUN_DIRECT:
                break;

            case RunPhase::ALIGNMENT:
                {
                    // 假设 10kHz loop
                    uint32_t align_ticks = (uint32_t)(obs_cfg.align_time_s * 10000.0f);

                    if (m.ctx_.fsm_timer_ticks > align_ticks) {
                        m.setRunPhase(RunPhase::FORCE_DRAG);
                        m.ctx_.fsm_timer_ticks = 0; // 重置计时器
                    }
                }
                break;

            case RunPhase::FORCE_DRAG:
                {
                    bool speed_ok = (fabsf(m.getSpeed()) > obs_cfg.switch_speed_rpm);
                    if (speed_ok) {
                        m.setRunPhase(RunPhase::SMO_ONLY);
                    }
                }
                break;

            case RunPhase::SMO_ONLY:
                if (fabsf(m.getSpeed()) < obs_cfg.switch_speed_rpm * 0.5f) {
                     m.setRunPhase(RunPhase::FORCE_DRAG);
                }
                break;

            default:
                break;
        }
    }

    /* --- ERROR: 故障 --- */
    void Motor_FSM::handleError(MotorManager& m)
    {
        // 等待外部调用 reset() 或 clearFault()
    }
}