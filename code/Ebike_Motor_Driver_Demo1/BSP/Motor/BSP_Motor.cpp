#include "BSP_Motor.h"
#include "Motor_API.h"
#include "main.h" // 包含 CubeMX 生成的定义 (stm32g4xx_hal.h 等)

/* ==============================================================================
 * [0] 全局变量与外部引用
 * ============================================================================== */

// 必须是 volatile，防止编译器优化
// 长度对应 "Number of Conversion"
volatile uint16_t adc_regular_buffer[1];

extern TIM_HandleTypeDef htim1; // PWM 定时器
extern ADC_HandleTypeDef hadc1; // 电流采样 ADC (相电流)
extern ADC_HandleTypeDef hadc2; // 辅助采样 ADC (母线电压/电流)

/* ==============================================================================
 * [1] 接口实现函数 (Implementation)
 * ============================================================================== */

/* ----------------- 1.1 运动控制 (Actuation) ----------------- */

// [PWM] 设置占空比
static void BSP_SetPWM_Impl(float u, float v, float w)
{
    // 假设 ARR = 4000
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim1);

    // 简单的饱和保护
    if(u > 1.0f) u = 1.0f; else if(u < 0.0f) u = 0.0f;
    if(v > 1.0f) v = 1.0f; else if(v < 0.0f) v = 0.0f;
    if(w > 1.0f) w = 1.0f; else if(w < 0.0f) w = 0.0f;

    // 直接写寄存器效率更高
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(u * period));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(v * period));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)(w * period));
}

// [PWM] 开启输出
static void BSP_PWM_Enable_Impl(void)
{
    // 开启 PWM 输出
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    // 互补输出开启 (如果是高级定时器 TIM1/TIM8)
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    // [必须加上] 既然你之前手动 Disable 了 MOE，这里必须手动 Enable
    // 就像你关了家里的总闸，只开房间灯(PWM Start)是没电的，必须拉起总闸(MOE)
    __HAL_TIM_MOE_ENABLE(&htim1);
}

// [PWM] 关闭输出 (高阻态)
static void BSP_PWM_Disable_Impl(void)
{
    // 最快关断方式：关闭主输出使能 (MOE)
    __HAL_TIM_MOE_DISABLE(&htim1);

    // 2. [新增] 强制停止所有通道的 PWM 生成
    // 这样即便 MOE 被意外开启，硬件也不会有波形输出
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);

    // 3. 互补通道也停止
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
}

/* ----------------- 1.2 传感器采样 (Sensing) ----------------- */

// [ADC] 读取相电流原始值 (Raw)
static void BSP_ReadCurrent_Raw_Impl(uint16_t* raw_a, uint16_t* raw_b, uint16_t* raw_c, uint16_t* raw_bus)
{
    // 读取 ADC1 的注入组数据
    *raw_a = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);
    *raw_b = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_3);
    *raw_c = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_4);

    *raw_bus = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);

    // 如果是三电阻采样，读取第三路；如果是双电阻，这里通常读到0或无效值
    // 假设 raw_c 暂时未用或由 KCL 计算，这里先置 0
    // *raw_c = 0;
}

// [ADC] 读取母线电压原始值 (Raw)
static uint16_t BSP_ReadVbus_Raw_Impl(void)
{
    // 直接返回 DMA 里的值 (ADC2 Regular Rank 1)
    return adc_regular_buffer[0];
}

// [ADC] 读取温度原始值 (Raw)
static uint16_t BSP_ReadTemp_Raw_Impl(void)
{
    // 直接返回 DMA 里的值 (ADC2 Regular Rank 2)
    // return adc_regular_buffer[1];
    return  0;  //此版本无温度采样
}

/* ==============================================================================
 * [2] 接口组装 (Interface Binding)
 * ============================================================================== */

// 定义实体变量 (这就是要传给库的东西)
static Lib_Motor::MotorHAL_t bsp_hal_impl =
{
    /* [Group 1] 运动控制接口 */
    .set_duty             = BSP_SetPWM_Impl,
    .pwm_enable           = BSP_PWM_Enable_Impl,
    .pwm_disable          = BSP_PWM_Disable_Impl,

    /* [Group 2] 传感器接口 */
    .read_currents_raw    = BSP_ReadCurrent_Raw_Impl,
    .read_vbus_raw        = BSP_ReadVbus_Raw_Impl,
    .read_temp_raw        = BSP_ReadTemp_Raw_Impl,

    /* [Group 3] 系统保护 */
    .enter_critical       = __disable_irq,
    .exit_critical        = __enable_irq
};

// 对外接口：返回指针
const Lib_Motor::MotorHAL_t* BSP_Get_HAL_Impl(void)
{
    return &bsp_hal_impl;
}

/* ==============================================================================
 * [3] 硬件启动序列 (Hardware Startup)
 * ============================================================================== */

void BSP_Motor_Hardware_Start(void)
{
    // 1. ADC 校准 (两个 ADC 都要校准！)
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED); // [关键新增]

    // 2. 设置 PWM 触发点 (ADC Trigger)
    // 将 CCR4 设置为 ARR - 1，确保在计数器计到顶端时触发采样
    // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, __HAL_TIM_GET_AUTORELOAD(&htim1) - 1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 100);

    // 3. 启动 ADC1 常规组 (DMA) - 用于 Vbus 和 Temp
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_regular_buffer, 1) != HAL_OK) {
        Error_Handler();
    }

    // 4. 启动注入组 (Injected Group)
    // ADC1: 相电流 (开启中断，作为主 Tick 源)
    HAL_ADCEx_InjectedStart_IT(&hadc2);

    // ADC2: 母线电流 (必须开启，否则 ADC2 不会响应触发信号)
    // HAL_ADCEx_InjectedStart(&hadc2); // [关键新增]

    // 5. 开启 PWM 定时器
    // 开启 Channel 4 以产生 TRGO/CC4 触发信号给 ADC
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    // 开启主计数器
    __HAL_TIM_ENABLE(&htim1);
}

/* ==============================================================================
 * [4] 中断回调 (Interrupt Callbacks)
 * ============================================================================== */

/**
 * @brief ADC 注入组转换完成回调
 * @note 当 ADC1 完成采样时 (即 PWM 中心点)，触发此中断，驱动电机库运行
 */
extern "C" void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    // 确保是 ADC1 触发的 (相电流采样完成)
    if (hadc->Instance == ADC2)
    {
        // 调试 IO 翻转 (测量 ISR 执行时间)
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

        // 进入电机库核心计算
        Motor_Global_Process_Handler(0);

        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    }
}