#include "lamp.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"

#define ADC_CHANNEL_NUM 3
#define PWM_PERIOD 1000
#define MAX_INTEGRAL 1000
#define KP 20.f
#define KI 1.f
#define KD 1.f

static control_mode_e control_mode = LIGHT_CONTROL;
static uint8_t rx_data;

static uint16_t adc_values[ADC_CHANNEL_NUM];
static float voltage_out = 0.f;
static float voltage_ref = 0.f;
static float light_intensity = 0.f;
static float light_intensity_max = 0.f;
static float light_intensity_min = 0.f;
static uint8_t light_flag = 1;

static float sound_intensity = 0.f;
static float sound_intensity_max = 0.f;
static float sound_intensity_min = 0.f;
static uint8_t sound_flag = 1;

static float error = 0.f;
static float last_error = 0.f;
static float integral = 0.f;
static float derivative = 0.f;
static float control_output = 0.f;

static void light_control();
static void sound_control();
float map_light_intensity(float value, float in_min, float in_max, float out_min, float out_max);

void lamp_init()
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);                           // 启动PWM
    HAL_UART_Receive_IT(&huart1, &rx_data, 1);                          // 启动串口接收中断
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_values, ADC_CHANNEL_NUM); // 启动ADC转换
    HAL_Delay(100);                                                     // 等待ADC稳定
}

void lamp_task()
{
    switch (control_mode)
    {
    case LIGHT_CONTROL:
        // 亮度控制
        light_control();
        break;
    case SOUND_CONTROL:
        // 声音控制
        sound_control();
        break;
    default:
        light_control();
        break;
    }
}

/**
 * @brief 串口接收中断回调函数
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        if (rx_data == LIGHT_CONTROL || rx_data == SOUND_CONTROL)
        {
            control_mode = (control_mode_e)rx_data;
        }
        else
        {
            control_mode = LIGHT_CONTROL; // 或者设置为默认模式
        }
        // 再次启用接收中断
        HAL_UART_Receive_IT(&huart1, &rx_data, 1);
    }
}

/**
 * @brief 亮度控制
 */
static void light_control()
{
    voltage_out = adc_values[0] * 3.3f / 4096 * 780 * 20000 / 2500 / 1000;
    light_intensity = adc_values[1] * 3.3f / 4096;

    // 如果是第一次采样，初始化最大值和最小值
    if (light_flag)
    {
        light_intensity_max = light_intensity;
        light_intensity_min = light_intensity;
        light_flag = 0; // 重置标志位
    }
    else
    {
        // 更新最大值
        if (light_intensity > light_intensity_max)
        {
            light_intensity_max = light_intensity;
        }
        // 更新最小值
        if (light_intensity < light_intensity_min)
        {
            light_intensity_min = light_intensity;
        }
    }

    // 将light_intensity映射到8-12范围
    voltage_ref = map_light_intensity(light_intensity, light_intensity_min, light_intensity_max, 8.0f, 12.0f);

    // 计算误差
    error = voltage_ref - voltage_out;
    // 积分计算
    integral += error;
    // 积分限幅
    if (integral > MAX_INTEGRAL)
        integral = MAX_INTEGRAL;
    if (integral < -MAX_INTEGRAL)
        integral = -MAX_INTEGRAL;
    // 微分计算
    derivative = error - last_error;
    last_error = error;
    // 计算PID
    control_output = KP * error + KI * integral + KD * derivative;
    // 限制控制量，防止过饱和
    if (control_output > 999.0f)
    {
        control_output = 999.0f;
    }
    else if (control_output < 0.0f)
    {
        control_output = 0.0f;
    }
    // 更新PWM占空比
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, (uint32_t)control_output);
}

/**
 * @brief 声音控制
 */
static void sound_control()
{
}

// 映射函数
float map_light_intensity(float value, float in_min, float in_max, float out_min, float out_max)
{
    if (in_max - in_min == 0)
    {
        return out_min;
    }
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}