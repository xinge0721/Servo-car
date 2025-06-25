#ifndef __SERVO_H__
#define __SERVO_H__

void Servo_Init(void);
void Servo_SetAngle(float Angle);
void Servo_SetAngle_Int16(int16_t angle);
void Servo_Tick_Handler(void); // 此函数需要被 SysTick_Handler 调用

#endif 