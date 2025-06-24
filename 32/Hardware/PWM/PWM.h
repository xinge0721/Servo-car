#ifndef __PWM_H__
#define __PWM_H__
#include "stm32f10x.h"                  // Device header

void PWM_Init(void);
void PWM_SetCompare1(uint16_t Compare);
void PWM_SetCompare2(uint16_t Compare);
#endif
