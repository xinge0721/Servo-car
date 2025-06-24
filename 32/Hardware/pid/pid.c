#include "stm32f10x.h"                 
#include "pid.h"

int16_t myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}

float pwm_control(float pwm)
{
    if(pwm>3599)
        pwm=3599;
    else if(pwm<-3599)
        pwm=-3599;
    return pwm;
}


/************
通用转向环或者角度环：输入目标位置和当前位置
*************/
float p_Err3,p_last_err3,p_pwm3,p_p=0,p_d=0,ERR_turn;
float P_angle_i;
int turn_pid(int now_position,float tar_position)//当前脉冲，目标脉冲 ,角速度
{
    p_Err3=tar_position-now_position;//目标脉冲-现在脉冲=误差脉冲
//	if(p_Err3>=-2 || p_Err3<=2) {p_Err3*=2;}
	if(p_Err3>180) {p_Err3-=360;}
    else if(p_Err3<-180) {p_Err3+=360;} 
    p_pwm3=p_p*p_Err3 + p_d*(p_Err3-p_last_err3);//
    p_last_err3=p_Err3;
    return p_pwm3;
}

/************
增量式速度环——左轮
先加i消除误差，再加p消除静态误差
*************/
float Velcity_Kp=0,  Velcity_Ki=0,  Velcity_Kd=0; //相关速度PID参数，50,25
int ControlVelocity=0, Last_bias=0, Bias=0;
int LVelocity_FeedbackControl(int TargetVelocity, int CurrentVelocity)
{
		
		Bias=TargetVelocity-CurrentVelocity; //求速度偏差
		
		ControlVelocity+=Velcity_Kp*(Bias-Last_bias)+Velcity_Ki*Bias;  //增量式PI控制器
                                                                   //Velcity_Kp*(Bias-Last_bias) 作用为限制加速度
	                                                                 //Velcity_Ki*Bias             速度控制值由Bias不断积分得到 偏差越大加速度越大
		Last_bias=Bias;	
		ControlVelocity=pwm_control(ControlVelocity);//限幅
		return ControlVelocity; //返回速度控制值
}


//增量式速度环—右轮
//先加i消除误差，再加p消除静态误差
float Velcity_RKp=0,  Velcity_RKi=0,  Velcity_RKd=0; //相关速度PID参数
int RControlVelocity=0, Last_bias1=0,Bias1=0; //函数调用结束后其值依然存在
int RVelocity_FeedbackControl(int TargetVelocity, int CurrentVelocity)
{
		
		
		Bias1=TargetVelocity-CurrentVelocity; //求速度偏差
		
		RControlVelocity+=Velcity_RKp*(Bias1-Last_bias1)+Velcity_RKi*Bias1;  //增量式PI控制器
                                                                   //Velcity_Kp*(Bias-Last_bias) 作用为限制加速度
	                                                                 //Velcity_Ki*Bias             速度控制值由Bias不断积分得到 偏差越大加速度越大
		Last_bias1=Bias1;	
		RControlVelocity=pwm_control(RControlVelocity);//限幅
		return RControlVelocity; //返回速度控制值
}


//位置式pid位置控制
//左
float p_Err=0,p_last_err=0,Integral=0,p_pwm=0,pid_p=0,pid_i=0,pid_d=0;

float p_pid(int16_t now_position,float tar_position)//积累脉冲(现在脉冲个数)，目标脉冲
{
    now_position=myabs(now_position);//转成正数
    p_Err=tar_position-now_position;//目标脉冲-现在脉冲=误差脉冲
		Integral+=p_Err;
		if(Integral> 970) Integral= 970;	//积分限幅
	  if(Integral<-970) Integral=-970;	//积分限幅
	//kp参数如果过大，设置电机转一圈就会过冲在回位，调到回位误差很小，这时候就可以调kd压制过冲
    p_pwm=pid_p*p_Err+pid_i*Integral+pid_d*(p_Err-p_last_err);
    p_pwm=pwm_control(p_pwm);
    p_last_err=p_Err;
    return p_pwm;
}

//位置式pid位置控制 
//右
float p_Err1=0,p_last_err1=0,Integral1=0,p_pwm1=0,pid_p1=0,pid_i1=0,pid_d1=0;

float p_pidR(int16_t now_position1,float tar_position1)//积累脉冲(现在脉冲个数)，目标脉冲
{
    now_position1=myabs(now_position1);//转成正数
    p_Err1=tar_position1-now_position1;//目标脉冲-现在脉冲=误差脉冲
		Integral1+=p_Err1;
		if(Integral1> 970) Integral1= 970;	//积分限幅
	  if(Integral1<-970) Integral1=-970;	//积分限幅
	//kp参数如果过大，设置电机转一圈就会过冲在回位，调到回位误差很小，这时候就可以调kd压制过冲
    p_pwm1=pid_p1*p_Err1+pid_i1*Integral1+pid_d1*(p_Err1-p_last_err1);
    p_pwm1=pwm_control(p_pwm1);
    p_last_err1=p_Err1;
    return p_pwm1;
}

/**************************************************************************
函数功能：速度(PWM)限幅
入口参数：PWM_P:位置环输出的PWM值 TargetVelocity:目标速度(速度限制值)
返回  值：无
**************************************************************************/
int   TargetVelocity=60;//此处可调速度
int Velocity_Restrict(int PWM_P, int TargetVelocity)
{
    if     (PWM_P>+TargetVelocity*49) PWM_P=+TargetVelocity*49;
	  else if(PWM_P<-TargetVelocity*49) PWM_P=-TargetVelocity*49;
	  else PWM_P=PWM_P;
	  return PWM_P;
}



