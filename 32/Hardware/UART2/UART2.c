#include <string.h>
#include <math.h>
#include <stdio.h>
#include "misc.h"
#include "wit_c_sdk.h"
#include "UART2.h"
/*               串口4            */ 
void Usart4Init(unsigned int uiBaud)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = uiBaud;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init( UART4, &USART_InitStructure);
	
//	USART_ITConfig( UART4, USART_IT_RXNE, ENABLE);
//	
//	
//	NVIC_InitTypeDef NVIC_InitStructure;
//	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//	NVIC_Init(&NVIC_InitStructure);
	
	USART_Cmd( UART4, ENABLE);
}

//void UART4_IRQHandler(void)//zigbee   对面发02
//{
//	if (USART_GetITStatus( UART4, USART_IT_RXNE) == SET)
//	{
//		unsigned char ucTemp;
//		ucTemp = USART_ReceiveData( UART4);
//		WitSerialDataIn(ucTemp);
//		USART_ClearITPendingBit( UART4, USART_IT_RXNE);
//	}
//}

void Uart4Send(unsigned char *p_data, unsigned int uiSize)
{	
	unsigned int i;
	for(i = 0; i < uiSize; i++)
	{
		while(USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
		USART_SendData(UART4, *p_data++);		
	}
	while(USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET);
}



/* 串口5                    */
uint8_t u5_RxData;
uint8_t u5_RxFlag;

void u5_Init(int uiBaud)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = uiBaud;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init( UART5, &USART_InitStructure);
	
	
	USART_Cmd( UART5, ENABLE);
}

void u5_SendByte(uint8_t Byte)
{
	USART_SendData( UART5, Byte);
	while (USART_GetFlagStatus( UART5, USART_FLAG_TXE) == RESET);
}

void u5_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)
	{
		u5_SendByte(Array[i]);
	}
}

void u5_SendString(char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)
	{
		u5_SendByte(String[i]);
	}
}

uint32_t u5_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y --)
	{
		Result *= X;
	}
	return Result;
}

void u5_SendNumber(uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i ++)
	{
		u5_SendByte(Number / u5_Pow(10, Length - i - 1) % 10 + '0');
	}
}
















//串口接收缓存区 	
u8 USART2_RX_BUF[USART3_MAX_RECV_LEN]; 				//接收缓冲,最大USART2_MAX_RECV_LEN个字节. 
u8  USART2_TX_BUF[USART3_MAX_SEND_LEN]; 			//发送缓冲,最大USART2_MAX_SEND_LEN字节

/*******************************USART2_DMA***********************************************************/
uint8_t  DMA_Rece_Buf2[DMA_Rec_Len2];	   //DMA接收串口数据缓冲区
u16  Usart2_Rec_Cnt=0;             //本帧数据长度	

u8  DMA_Tx_Buf2[DMA_Tx_Len2];	   //DMA接收串口数据缓冲区
u16  Usart2_Tx_Cnt=0;             //本帧数据长度	



//UART2的中断优先级初始化
void UART2_NVIC_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;		//串口2接收中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure); //根据指定的参数初始化VIC寄存器
}

//UART2_RX的DMA初始化
void UART2_DMA_RX_Init(void)
{
	DMA_InitTypeDef DMA_UART2_RX;
	
	//相应的DMA配置
	DMA_DeInit(DMA2_Channel3);   //将DMA的通道6寄存器重设为缺省值  串口2对应的是DMA通道6
	DMA_UART2_RX.DMA_PeripheralBaseAddr = (u32)&UART4->DR;  //DMA外设ADC基地址
	DMA_UART2_RX.DMA_MemoryBaseAddr = (u32)DMA_Rece_Buf2;  //DMA内存基地址
	DMA_UART2_RX.DMA_DIR = DMA_DIR_PeripheralSRC;  //数据传输方向，从外设读取发送到内存
	DMA_UART2_RX.DMA_BufferSize = DMA_Rec_Len2;  //DMA通道的DMA缓存的大小
	DMA_UART2_RX.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //外设地址寄存器不变
	DMA_UART2_RX.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存地址寄存器递增
	DMA_UART2_RX.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //数据宽度为8位
	DMA_UART2_RX.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //数据宽度为8位
	DMA_UART2_RX.DMA_Mode = DMA_Mode_Normal;  //工作在正常缓存模式
	DMA_UART2_RX.DMA_Priority = DMA_Priority_Medium; //DMA通道 x拥有中优先级 
	DMA_UART2_RX.DMA_M2M = DMA_M2M_Disable;  //DMA通道x没有设置为内存到内存传输
	DMA_Init(DMA2_Channel3, &DMA_UART2_RX);  //根据DMA_InitStruct中指定的参数初始化DMA的通道USART2_Tx_DMA_Channel所标识的寄存器
		
	DMA_Cmd(DMA2_Channel3, ENABLE);  //正式驱动DMA传输
}

//UART3_TX的DMA初始化
void UART2_DMA_TX_Init(void)
{
	DMA_InitTypeDef DMA_UART2_TX;
	
	//相应的DMA配置
	DMA_DeInit(DMA1_Channel5);   //将DMA的通道7寄存器重设为缺省值  串口2对应的是DMA通道7
	DMA_UART2_TX.DMA_PeripheralBaseAddr = (u32)&UART4->DR;  //DMA外设ADC基地址
	DMA_UART2_TX.DMA_MemoryBaseAddr = (u32)DMA_Tx_Buf2;  //DMA内存基地址
	DMA_UART2_TX.DMA_DIR = DMA_DIR_PeripheralDST;  //数据传输方向，从内存读取发送到外设
	DMA_UART2_TX.DMA_BufferSize = DMA_Tx_Len2;  //DMA通道的DMA缓存的大小
	DMA_UART2_TX.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //外设地址寄存器不变
	DMA_UART2_TX.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存地址寄存器递增
	DMA_UART2_TX.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //数据宽度为8位
	DMA_UART2_TX.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //数据宽度为8位
	DMA_UART2_TX.DMA_Mode = DMA_Mode_Normal;  //工作在正常缓存模式
	DMA_UART2_TX.DMA_Priority = DMA_Priority_Medium; //DMA通道 x拥有中优先级 
	DMA_UART2_TX.DMA_M2M = DMA_M2M_Disable;  //DMA通道x没有设置为内存到内存传输
	DMA_Init(DMA2_Channel5, &DMA_UART2_TX);  //根据DMA_InitStruct中指定的参数初始化DMA的通道USART2_Tx_DMA_Channel所标识的寄存器
		
	DMA_Cmd(DMA2_Channel5, ENABLE);  //正式驱动DMA传输
}

//初始化IO 串口2
//bound:波特率
void uart4_init_DMA(u32 bound)
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);	//使能DMA传输
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init( UART4, &USART_InitStructure);
	
	
	
 	NVIC_InitTypeDef NVIC_InitStructure;
	
		
		NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn; //使能串口4中断
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //先占优先级2级
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; //从优先级2级
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //使能外部中断通道
		NVIC_Init(&NVIC_InitStructure); //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
	 
		USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//开启中断   
		USART_Cmd(UART4, ENABLE);                    //使能串口2
	
		USART_ITConfig(UART4, USART_IT_IDLE, ENABLE); //开启空闲中断
		USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE); //使能串口2的DMA接收
		USART_DMACmd(UART4, USART_DMAReq_Tx, ENABLE); //使能串口2的DMA发送
		USART_Cmd(UART4, ENABLE); //使能串口 
		
		UART2_DMA_RX_Init(); //UART3_RX的EDMA功能初始化
		UART2_DMA_TX_Init(); //UART3_TX的EDMA功能初始化

		
}

//重新使能UART3_RX的DMA功能
void UART2_RX_DMA_Enable(void)
{ 
	USART_DMACmd(UART4, USART_DMAReq_Rx, DISABLE); //先停止串口1的DMA接收
	DMA_Cmd(DMA2_Channel3, DISABLE ); //先停止DMA，暂停接收 
 	DMA_SetCurrDataCounter(DMA2_Channel3, DMA_Rec_Len2); //DMA通道的DMA缓存的大小
 	DMA_Cmd(DMA2_Channel3, ENABLE); //使能USART3 TX DMA1 所指示的通道 
	USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE); //使能串口1的DMA接收
}	

//重新使能UART3_TX的DMA功能
void UART2_TX_DMA_Enable(void)
{ 
	USART_DMACmd(UART4, USART_DMAReq_Tx, DISABLE); //先停止串口1的DMA发送
	DMA_Cmd(DMA2_Channel5, DISABLE ); //先停止DMA，暂停发送 
 	DMA_SetCurrDataCounter(DMA2_Channel5, Usart2_Rec_Cnt); //DMA通道的DMA缓存的大小
 	DMA_Cmd(DMA2_Channel5, ENABLE); //使能USART3 TX DMA1 所指示的通道 
	USART_DMACmd(UART4, USART_DMAReq_Tx, ENABLE); //使能串口1的DMA发送
}	

//发送len个字节.
//buf:发送区首地址
//len:发送的字节数
void Usart2_Send(u8 *buf,u8 len)
{	
	 u8 t;

  	for(t=0;t<len;t++)		//循环发送数据
	{		   
		while(USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET);	  
		USART_SendData(UART4,buf[t]);
	}
	  while(USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET);		

}

void UART4_IRQHandler(void)                	//串口2中断服务程序
{
	//串口2空闲中断
	if(USART_GetITStatus(UART4, USART_IT_IDLE) != RESET)  
	{	
		/* 1.清除串口2空闲中断标志  (先读USART_SR，然后读USART_DR)*/
		USART_ClearITPendingBit(UART4, USART_IT_IDLE); //读USART_SR
		USART_ReceiveData(UART4); 											//然后读USART_DR
		
		/* 2.读取DMA */
		Usart2_Rec_Cnt = DMA_Rec_Len2 - DMA_GetCurrDataCounter(DMA2_Channel3); //接收个数等于接收缓冲区总大小减已经接收的个数
		
		/* 3.搬移数据进行其他处理 */
		memset(DMA_Tx_Buf2,'\0',sizeof(DMA_Tx_Buf2));				//先清空DMA_Tx_Buf
		//将 DMA_Rece_Buf2 中的前 Usart2_Rec_Cnt 个字节复制到 USART2_RX_BUF 中。
		memcpy(USART2_RX_BUF, DMA_Rece_Buf2, Usart2_Rec_Cnt); //将接收转移通过串口2的DMA方式发送出去测试
		// 逐个字节调用WitSerialDataIn处理数据
			for(uint16_t i = 0; i < Usart2_Rec_Cnt; i++)
			{
					WitSerialDataIn(USART2_RX_BUF[i]); // 处理每个接收到的字节 //维特陀螺仪数据处理
			}
		memset(DMA_Rece_Buf2,'\0',sizeof(DMA_Rece_Buf2));				//先清空DMA_Tx_Buf
			
		memcpy(DMA_Tx_Buf2, USART2_RX_BUF, Usart2_Rec_Cnt); //将接收转移通过串口2的DMA方式发送出去测试		
		UART2_TX_DMA_Enable(); //开启一次DMA发送，实现转发
	  while(USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET);	
	
				
		/* 4.开启新的一次DMA接收 */
		UART2_RX_DMA_Enable(); //重新使能DMA，等待下一次的接收
		memset(DMA_Rece_Buf2,'\0',sizeof(DMA_Rece_Buf2));//清空DMA_Rece_Buf
  }
	
} 


