#ifndef __MOTORRUN_H__
#define __MOTORRUN_H__
#include "stm32f10x.h"                  // Device header

void run(uint16_t LCompare,uint16_t RCompare);
void backrun(uint16_t LCompare,uint16_t RCompare);
void stop(void);
void right(int32_t speed);
void left(int32_t speed) ;
#endif
