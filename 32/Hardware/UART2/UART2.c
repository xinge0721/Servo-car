#include <string.h>
#include <math.h>
#include <stdio.h>
#include "misc.h"
#include "wit_c_sdk.h"
#include "UART2.h"
/*               ����4            */ 
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

//void UART4_IRQHandler(void)//zigbee   ���淢02
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



/* ����5                    */
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
















//���ڽ��ջ����� 	
u8 USART2_RX_BUF[USART3_MAX_RECV_LEN]; 				//���ջ���,���USART2_MAX_RECV_LEN���ֽ�. 
u8  USART2_TX_BUF[USART3_MAX_SEND_LEN]; 			//���ͻ���,���USART2_MAX_SEND_LEN�ֽ�

/*******************************USART2_DMA***********************************************************/
uint8_t  DMA_Rece_Buf2[DMA_Rec_Len2];	   //DMA���մ������ݻ�����
u16  Usart2_Rec_Cnt=0;             //��֡���ݳ���	

u8  DMA_Tx_Buf2[DMA_Tx_Len2];	   //DMA���մ������ݻ�����
u16  Usart2_Tx_Cnt=0;             //��֡���ݳ���	



//UART2���ж����ȼ���ʼ��
void UART2_NVIC_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;		//����2�����ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure); //����ָ���Ĳ�����ʼ��VIC�Ĵ���
}

//UART2_RX��DMA��ʼ��
void UART2_DMA_RX_Init(void)
{
	DMA_InitTypeDef DMA_UART2_RX;
	
	//��Ӧ��DMA����
	DMA_DeInit(DMA2_Channel3);   //��DMA��ͨ��6�Ĵ�������Ϊȱʡֵ  ����2��Ӧ����DMAͨ��6
	DMA_UART2_RX.DMA_PeripheralBaseAddr = (u32)&UART4->DR;  //DMA����ADC����ַ
	DMA_UART2_RX.DMA_MemoryBaseAddr = (u32)DMA_Rece_Buf2;  //DMA�ڴ����ַ
	DMA_UART2_RX.DMA_DIR = DMA_DIR_PeripheralSRC;  //���ݴ��䷽�򣬴������ȡ���͵��ڴ�
	DMA_UART2_RX.DMA_BufferSize = DMA_Rec_Len2;  //DMAͨ����DMA����Ĵ�С
	DMA_UART2_RX.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //�����ַ�Ĵ�������
	DMA_UART2_RX.DMA_MemoryInc = DMA_MemoryInc_Enable;  //�ڴ��ַ�Ĵ�������
	DMA_UART2_RX.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //���ݿ��Ϊ8λ
	DMA_UART2_RX.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //���ݿ��Ϊ8λ
	DMA_UART2_RX.DMA_Mode = DMA_Mode_Normal;  //��������������ģʽ
	DMA_UART2_RX.DMA_Priority = DMA_Priority_Medium; //DMAͨ�� xӵ�������ȼ� 
	DMA_UART2_RX.DMA_M2M = DMA_M2M_Disable;  //DMAͨ��xû������Ϊ�ڴ浽�ڴ洫��
	DMA_Init(DMA2_Channel3, &DMA_UART2_RX);  //����DMA_InitStruct��ָ���Ĳ�����ʼ��DMA��ͨ��USART2_Tx_DMA_Channel����ʶ�ļĴ���
		
	DMA_Cmd(DMA2_Channel3, ENABLE);  //��ʽ����DMA����
}

//UART3_TX��DMA��ʼ��
void UART2_DMA_TX_Init(void)
{
	DMA_InitTypeDef DMA_UART2_TX;
	
	//��Ӧ��DMA����
	DMA_DeInit(DMA1_Channel5);   //��DMA��ͨ��7�Ĵ�������Ϊȱʡֵ  ����2��Ӧ����DMAͨ��7
	DMA_UART2_TX.DMA_PeripheralBaseAddr = (u32)&UART4->DR;  //DMA����ADC����ַ
	DMA_UART2_TX.DMA_MemoryBaseAddr = (u32)DMA_Tx_Buf2;  //DMA�ڴ����ַ
	DMA_UART2_TX.DMA_DIR = DMA_DIR_PeripheralDST;  //���ݴ��䷽�򣬴��ڴ��ȡ���͵�����
	DMA_UART2_TX.DMA_BufferSize = DMA_Tx_Len2;  //DMAͨ����DMA����Ĵ�С
	DMA_UART2_TX.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //�����ַ�Ĵ�������
	DMA_UART2_TX.DMA_MemoryInc = DMA_MemoryInc_Enable;  //�ڴ��ַ�Ĵ�������
	DMA_UART2_TX.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //���ݿ��Ϊ8λ
	DMA_UART2_TX.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //���ݿ��Ϊ8λ
	DMA_UART2_TX.DMA_Mode = DMA_Mode_Normal;  //��������������ģʽ
	DMA_UART2_TX.DMA_Priority = DMA_Priority_Medium; //DMAͨ�� xӵ�������ȼ� 
	DMA_UART2_TX.DMA_M2M = DMA_M2M_Disable;  //DMAͨ��xû������Ϊ�ڴ浽�ڴ洫��
	DMA_Init(DMA2_Channel5, &DMA_UART2_TX);  //����DMA_InitStruct��ָ���Ĳ�����ʼ��DMA��ͨ��USART2_Tx_DMA_Channel����ʶ�ļĴ���
		
	DMA_Cmd(DMA2_Channel5, ENABLE);  //��ʽ����DMA����
}

//��ʼ��IO ����2
//bound:������
void uart4_init_DMA(u32 bound)
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);	//ʹ��DMA����
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
	
		
		NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn; //ʹ�ܴ���4�ж�
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //��ռ���ȼ�2��
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; //�����ȼ�2��
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //ʹ���ⲿ�ж�ͨ��
		NVIC_Init(&NVIC_InitStructure); //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
	 
		USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//�����ж�   
		USART_Cmd(UART4, ENABLE);                    //ʹ�ܴ���2
	
		USART_ITConfig(UART4, USART_IT_IDLE, ENABLE); //���������ж�
		USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE); //ʹ�ܴ���2��DMA����
		USART_DMACmd(UART4, USART_DMAReq_Tx, ENABLE); //ʹ�ܴ���2��DMA����
		USART_Cmd(UART4, ENABLE); //ʹ�ܴ��� 
		
		UART2_DMA_RX_Init(); //UART3_RX��EDMA���ܳ�ʼ��
		UART2_DMA_TX_Init(); //UART3_TX��EDMA���ܳ�ʼ��

		
}

//����ʹ��UART3_RX��DMA����
void UART2_RX_DMA_Enable(void)
{ 
	USART_DMACmd(UART4, USART_DMAReq_Rx, DISABLE); //��ֹͣ����1��DMA����
	DMA_Cmd(DMA2_Channel3, DISABLE ); //��ֹͣDMA����ͣ���� 
 	DMA_SetCurrDataCounter(DMA2_Channel3, DMA_Rec_Len2); //DMAͨ����DMA����Ĵ�С
 	DMA_Cmd(DMA2_Channel3, ENABLE); //ʹ��USART3 TX DMA1 ��ָʾ��ͨ�� 
	USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE); //ʹ�ܴ���1��DMA����
}	

//����ʹ��UART3_TX��DMA����
void UART2_TX_DMA_Enable(void)
{ 
	USART_DMACmd(UART4, USART_DMAReq_Tx, DISABLE); //��ֹͣ����1��DMA����
	DMA_Cmd(DMA2_Channel5, DISABLE ); //��ֹͣDMA����ͣ���� 
 	DMA_SetCurrDataCounter(DMA2_Channel5, Usart2_Rec_Cnt); //DMAͨ����DMA����Ĵ�С
 	DMA_Cmd(DMA2_Channel5, ENABLE); //ʹ��USART3 TX DMA1 ��ָʾ��ͨ�� 
	USART_DMACmd(UART4, USART_DMAReq_Tx, ENABLE); //ʹ�ܴ���1��DMA����
}	

//����len���ֽ�.
//buf:�������׵�ַ
//len:���͵��ֽ���
void Usart2_Send(u8 *buf,u8 len)
{	
	 u8 t;

  	for(t=0;t<len;t++)		//ѭ����������
	{		   
		while(USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET);	  
		USART_SendData(UART4,buf[t]);
	}
	  while(USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET);		

}

void UART4_IRQHandler(void)                	//����2�жϷ������
{
	//����2�����ж�
	if(USART_GetITStatus(UART4, USART_IT_IDLE) != RESET)  
	{	
		/* 1.�������2�����жϱ�־  (�ȶ�USART_SR��Ȼ���USART_DR)*/
		USART_ClearITPendingBit(UART4, USART_IT_IDLE); //��USART_SR
		USART_ReceiveData(UART4); 											//Ȼ���USART_DR
		
		/* 2.��ȡDMA */
		Usart2_Rec_Cnt = DMA_Rec_Len2 - DMA_GetCurrDataCounter(DMA2_Channel3); //���ո������ڽ��ջ������ܴ�С���Ѿ����յĸ���
		
		/* 3.�������ݽ����������� */
		memset(DMA_Tx_Buf2,'\0',sizeof(DMA_Tx_Buf2));				//�����DMA_Tx_Buf
		//�� DMA_Rece_Buf2 �е�ǰ Usart2_Rec_Cnt ���ֽڸ��Ƶ� USART2_RX_BUF �С�
		memcpy(USART2_RX_BUF, DMA_Rece_Buf2, Usart2_Rec_Cnt); //������ת��ͨ������2��DMA��ʽ���ͳ�ȥ����
		// ����ֽڵ���WitSerialDataIn��������
			for(uint16_t i = 0; i < Usart2_Rec_Cnt; i++)
			{
					WitSerialDataIn(USART2_RX_BUF[i]); // ����ÿ�����յ����ֽ� //ά�����������ݴ���
			}
		memset(DMA_Rece_Buf2,'\0',sizeof(DMA_Rece_Buf2));				//�����DMA_Tx_Buf
			
		memcpy(DMA_Tx_Buf2, USART2_RX_BUF, Usart2_Rec_Cnt); //������ת��ͨ������2��DMA��ʽ���ͳ�ȥ����		
		UART2_TX_DMA_Enable(); //����һ��DMA���ͣ�ʵ��ת��
	  while(USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET);	
	
				
		/* 4.�����µ�һ��DMA���� */
		UART2_RX_DMA_Enable(); //����ʹ��DMA���ȴ���һ�εĽ���
		memset(DMA_Rece_Buf2,'\0',sizeof(DMA_Rece_Buf2));//���DMA_Rece_Buf
  }
	
} 


