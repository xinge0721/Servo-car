#include "LED.h"
void LED_Init()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOF,ENABLE);
	GPIO_InitTypeDef GPIO_LED_InitStructure;
	GPIO_LED_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_LED_InitStructure.GPIO_Pin=GPIO_Pin_5|GPIO_Pin_1;
	GPIO_LED_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_LED_InitStructure);//LED
	
	GPIO_LED_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_LED_InitStructure.GPIO_Pin=GPIO_Pin_5;
	GPIO_LED_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOF,&GPIO_LED_InitStructure);//·äÃùÆ÷
	PBout(5)=1;
	Beep=1;
}
void LED1_Turn()
{
	if(GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_5)==0)
		GPIO_SetBits(GPIOB,GPIO_Pin_5);
	else
		GPIO_ResetBits(GPIOB,GPIO_Pin_5);
}
void LED2_Turn()
{
	if(GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_1)==0)
		GPIO_SetBits(GPIOA,GPIO_Pin_1);
	else
		GPIO_ResetBits(GPIOA,GPIO_Pin_1);
}

void LED1_on()
{
		GPIO_ResetBits(GPIOB,GPIO_Pin_5);
}

void LED1_off()
{
		GPIO_SetBits(GPIOB,GPIO_Pin_5);
}

void LED2_on()
{
		GPIO_ResetBits(GPIOB,GPIO_Pin_1);
}

void LED2_off()
{
		GPIO_SetBits(GPIOB,GPIO_Pin_1);
}