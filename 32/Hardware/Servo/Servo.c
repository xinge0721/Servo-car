#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "Servo.h"
#include "delay.h"

// 假设 SysTick 中断周期为 1ms
// PWM 周期 = 20ms = 20 * 1ms ticks
#define SERVO_PERIOD_TICKS 20

// 舵机连接在 PA4
#define SERVO_PIN   GPIO_Pin_4
#define SERVO_PORT  GPIOA

// 脉冲宽度范围: 500-2500us (对应0.5-2.5ms)
#define SERVO_MIN_PULSE 500
#define SERVO_MAX_PULSE 2500

// 全局变量，用于在中断中确定脉冲宽度(单位: us)
static int servo_pulse_us = 1500; 

// 中间位置脉冲宽度
#define SERVO_MIDDLE_PULSE 650

// 直接设置脉冲宽度(us)
// 舵机脉宽范围: 500us ~ 2500us 对应 0~180度
void Servo_SetPulse(int Pulse_us)
{
    if (Pulse_us < SERVO_MIN_PULSE) Pulse_us = SERVO_MIN_PULSE;
    if (Pulse_us > SERVO_MAX_PULSE) Pulse_us = SERVO_MAX_PULSE;
    
    servo_pulse_us = Pulse_us;
}

// 使用角度设置舵机位置 (角度范围: 0~180度)
void Servo_SetAngle_Int16(int16_t angle)
{
    // 限制角度范围
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    
    // 角度转换为脉冲宽度
    // 脉冲宽度(μs) = 500 + 角度 × 11.11
    int pulse = SERVO_MIN_PULSE + (int)((float)angle * 11.11f);
    
    // 设置脉冲宽度
    Servo_SetPulse(pulse);
}

void Servo_Init(void)
{
    // 1. 初始化舵机控制IO口
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = SERVO_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SERVO_PORT, &GPIO_InitStructure);

    // 2. 默认设置舵机脉冲为中间位置 (1500us)
    Servo_SetPulse(SERVO_MIDDLE_PULSE);
    
    // 注意: SysTick 的初始化通常在其他地方完成 (例如 delay_init)，
    // 这里不再重复初始化。只需确保 SysTick 中断已开启且周期为1ms。
}


// 此函数需要被 SysTick_Handler 以1ms的频率调用
void Servo_Tick_Handler(void)
{
    static int tick_counter = 0;
    for(int i=0;i<2500;i++)
    {
        if(i <= servo_pulse_us)
        {
            GPIO_SetBits(SERVO_PORT, SERVO_PIN);
        }
        else
        {
            GPIO_ResetBits(SERVO_PORT, SERVO_PIN);
        }
        delay_us(1);
    }
} 