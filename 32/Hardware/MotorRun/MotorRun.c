#include "stm32f10x.h"                  
#include "Motor.h"
#include "PWM.h"
#include "oled.h"
//小车前进函数
void run(uint16_t LCompare,uint16_t RCompare)
{
	PWM_SetCompare1(LCompare); 	//调速
	PWM_SetCompare2(RCompare); 	//调速
	Right_moto_go();       		//往前
}
 
//小车后退函数
void backrun(uint16_t LCompare,uint16_t RCompare)
{
	PWM_SetCompare1(LCompare); 	//调速
	PWM_SetCompare2(RCompare); 	//调速
	Right_moto_back();     		//往后
}
 
//小车停车函数
void stop(void)         		
{   	
	Right_moto_Stop();     		//停止
}

void left(int32_t speed)       
{
     if(speed>=0)
    {
		PWM_SetCompare1(speed);       
		GPIO_SetBits(GPIOB, GPIO_Pin_14);
		GPIO_ResetBits(GPIOB, GPIO_Pin_15);
    }
    else
    {
		PWM_SetCompare1(-speed);       
		GPIO_ResetBits(GPIOB, GPIO_Pin_14);
		GPIO_SetBits(GPIOB, GPIO_Pin_15);
    }
			
		
}

void right(int32_t speed)       
{
	if(speed>=0)
    {
		PWM_SetCompare2(speed); 
		GPIO_SetBits(GPIOD, GPIO_Pin_0);
		GPIO_ResetBits(GPIOB, GPIO_Pin_13);
    }
    else
    {
		PWM_SetCompare2(-speed);       
		GPIO_ResetBits(GPIOD, GPIO_Pin_0);
		GPIO_SetBits(GPIOB, GPIO_Pin_13);
    }
   
}

//左转（仅控制舵机）
// 参数：角度
void turn_left(int32_t angle)
{
	Servo_SetPulse(SERVO_MIDDLE_PULSE + angle);
}
//右转（仅控制舵机）
// 参数：角度
void turn_right(int32_t angle)
{
	Servo_SetPulse(SERVO_MIDDLE_PULSE - angle);
}


