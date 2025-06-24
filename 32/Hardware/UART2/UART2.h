#ifndef __UART2_H
#define __UART2_H
#include "stm32f10x.h"                  // Device header
#include "sys.h"

void u5_Init(int uiBaud);
void u5_SendByte(uint8_t Byte);
void u5_SendArray(uint8_t *Array, uint16_t Length);
void u5_SendString(char *String);
void u5_SendNumber(uint32_t Number, uint8_t Length);

//如果想串口中断接收，请不要注释以下宏定义
#define USART3_MAX_RECV_LEN		600					//最大接收缓存字节数
#define USART3_MAX_SEND_LEN		600					//最大发送缓存字节数
#define EN_USART2_RX 	1			//0,不接收;1,接收.


#define DMA_Rec_Len2 200
#define DMA_Tx_Len2  200	




extern u16  Usart2_Rec_Cnt;             //本帧数据长度	



void uart4_init_DMA(u32 bound);
void Usart2_Send(u8 *buf,u8 len);
void UART2_TX_DMA_Enable(void);
#endif

//------------------End of File----------------------------

