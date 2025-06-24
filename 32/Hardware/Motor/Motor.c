#include "stm32f10x.h"             
#include "PWM.h"
 
//左右电机初始化
void Motor_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOD, ENABLE);
	//电机控制模拟输入端
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	//控制右电机->PD0 PB13
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//控制左电机->PB14 PB15
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	PWM_Init();
}
 
//向前
void Right_moto_go(void)
{
	//正转
	GPIO_ResetBits(GPIOB, GPIO_Pin_13);
	GPIO_SetBits(GPIOD, GPIO_Pin_0);//右
	GPIO_ResetBits(GPIOB, GPIO_Pin_15);
	GPIO_SetBits(GPIOB, GPIO_Pin_14);//左
}
 
//向后
void Right_moto_back(void)
{
	//反转
	GPIO_SetBits(GPIOB, GPIO_Pin_13);
	GPIO_ResetBits(GPIOD, GPIO_Pin_0);
	GPIO_SetBits(GPIOB, GPIO_Pin_15);
	GPIO_ResetBits(GPIOB, GPIO_Pin_14);
    
}


//电机停止
void Right_moto_Stop(void)
{
	//停车
	GPIO_ResetBits(GPIOD, GPIO_Pin_0);
	GPIO_ResetBits(GPIOB, GPIO_Pin_13);
	GPIO_ResetBits(GPIOB, GPIO_Pin_14);
	GPIO_ResetBits(GPIOB, GPIO_Pin_15);
}
