#include "stm32f10x.h"                  // Device header
#include "key.h"
#include "Delay.h"
/**
  * 函    数：按键初始化
  * 参    数：无
  * 返 回 值：无
  */
void Key_Init()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE);
	GPIO_InitTypeDef GPIO_Key_InitStructure;
	GPIO_Key_InitStructure.GPIO_Mode=GPIO_Mode_IPD;
	GPIO_Key_InitStructure.GPIO_Pin=GPIO_Pin_4;
	GPIO_Key_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOE,&GPIO_Key_InitStructure);
	
		GPIO_Key_InitStructure.GPIO_Mode=GPIO_Mode_IPD;
	GPIO_Key_InitStructure.GPIO_Pin=GPIO_Pin_0;
	GPIO_Key_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_Key_InitStructure);	
}
uint8_t Get_Key1Num()
{
	uint8_t KeyNum=0;
	if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_4)==1)
	{
		delay_ms(20);
		while(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_4)==1);
		delay_ms(20);
		KeyNum=1;
	}
	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)==1)
	{
		delay_ms(20);
		while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)==1);
		delay_ms(20);
		KeyNum=2;
	}
	return KeyNum;
}
