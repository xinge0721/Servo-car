#ifndef __MITHON_H
#define __MITHON_H
#include "stm32f10x.h"                  // Device header


extern	float zangle;
extern  int Gyroz; 
extern	int16_t now_Lspeed;		//С�������������ٶ�
extern	int16_t now_Rspeed;		//С���ҵ���������ٶ�
extern	int16_t ADD;					//С��ת�򻷲���
void xunxian(int16_t ADD);
void zhuanwan(float i,float speed);//ת��
void stopcar(void);
void posithon_w(int posithon);
void zhixian(float i,float speed);
extern int now_Lposition,now_Rposition;
extern int NowposithonL;
extern int NowposithonR;
extern int Nowposithonall;
#endif
