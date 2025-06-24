#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdio.h>
#include "stm32f10x.h"                  // Device header

void Usart1Init(unsigned int uiBaud);
void Serial_SendByte1(uint8_t Byte);
#endif
