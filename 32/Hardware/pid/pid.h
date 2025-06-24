#ifndef __PID_H
#define __PID_H
#include "stm32f10x.h"                  // Device header

int turn_pid(int now_position,float tar_position);
float pwm_control(float pwm);
int LVelocity_FeedbackControl(int TargetVelocity, int CurrentVelocity);
int RVelocity_FeedbackControl(int TargetVelocity, int CurrentVelocity);
float p_pid(int16_t now_position,float tar_position);
float p_pidR(int16_t now_position1,float tar_position1);
int Velocity_Restrict(int PWM_P, int TargetVelocity);
extern int   TargetVelocity,RControlVelocity,ControlVelocity,Last_bias,Last_bias1;
extern float p_p,p_d;//转向参数
extern float P_angle_i;
extern float Velcity_Kp,  Velcity_Ki,  Velcity_Kd;//左轮速度环
extern float Velcity_RKp, Velcity_RKi,  Velcity_RKd;//右轮速度环
extern float pid_p,pid_i,pid_d;//左轮位置环
extern float pid_p1,pid_i1,pid_d1;//右轮位置环
#endif
