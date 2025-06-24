#ifndef __ENCODER_H__
#define __ENCODER_H__
#include "stm32f10x.h"                  // Device header

void Encoder_Init_Right(void);
void Encoder_Init_Left(void);
int16_t Encoder_Get_Right(void);
int16_t Encoder_Get_Left(void);
#endif
