#ifndef __LED_H__
#define	__LED_H__
void LED_Init(void);
void LED1_Turn(void);
void LED1_on(void);
void LED1_off(void);
void LED2_on(void);
void LED2_off(void);
#include "sys.h"                  // Device header
#define Beep PFout(5)
#endif
