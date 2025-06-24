#ifndef __MITHON_H
#define __MITHON_H
#include "stm32f10x.h"                  // Device header


extern	float zangle;
extern  int Gyroz; 
extern	int16_t now_Lspeed;		//小车左电机编码器速度
extern	int16_t now_Rspeed;		//小车右电机编码器速度
extern	int16_t ADD;					//小车转向环参数
void xunxian(int16_t ADD);
void zhuanwan(float i,float speed);//转弯
void stopcar(void);
void posithon_w(int posithon);
void zhixian(float i,float speed);
extern int now_Lposition,now_Rposition;
extern int NowposithonL;
extern int NowposithonR;
extern int Nowposithonall;
#endif
