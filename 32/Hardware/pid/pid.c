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
ͨ��ת�򻷻��߽ǶȻ�������Ŀ��λ�ú͵�ǰλ��
*************/
float p_Err3,p_last_err3,p_pwm3,p_p=0,p_d=0,ERR_turn;
float P_angle_i;
int turn_pid(int now_position,float tar_position)//��ǰ���壬Ŀ������ ,���ٶ�
{
    p_Err3=tar_position-now_position;//Ŀ������-��������=�������
//	if(p_Err3>=-2 || p_Err3<=2) {p_Err3*=2;}
	if(p_Err3>180) {p_Err3-=360;}
    else if(p_Err3<-180) {p_Err3+=360;} 
    p_pwm3=p_p*p_Err3 + p_d*(p_Err3-p_last_err3);//
    p_last_err3=p_Err3;
    return p_pwm3;
}

/************
����ʽ�ٶȻ���������
�ȼ�i�������ټ�p������̬���
*************/
float Velcity_Kp=0,  Velcity_Ki=0,  Velcity_Kd=0; //����ٶ�PID������50,25
int ControlVelocity=0, Last_bias=0, Bias=0;
int LVelocity_FeedbackControl(int TargetVelocity, int CurrentVelocity)
{
		
		Bias=TargetVelocity-CurrentVelocity; //���ٶ�ƫ��
		
		ControlVelocity+=Velcity_Kp*(Bias-Last_bias)+Velcity_Ki*Bias;  //����ʽPI������
                                                                   //Velcity_Kp*(Bias-Last_bias) ����Ϊ���Ƽ��ٶ�
	                                                                 //Velcity_Ki*Bias             �ٶȿ���ֵ��Bias���ϻ��ֵõ� ƫ��Խ����ٶ�Խ��
		Last_bias=Bias;	
		ControlVelocity=pwm_control(ControlVelocity);//�޷�
		return ControlVelocity; //�����ٶȿ���ֵ
}


//����ʽ�ٶȻ�������
//�ȼ�i�������ټ�p������̬���
float Velcity_RKp=0,  Velcity_RKi=0,  Velcity_RKd=0; //����ٶ�PID����
int RControlVelocity=0, Last_bias1=0,Bias1=0; //�������ý�������ֵ��Ȼ����
int RVelocity_FeedbackControl(int TargetVelocity, int CurrentVelocity)
{
		
		
		Bias1=TargetVelocity-CurrentVelocity; //���ٶ�ƫ��
		
		RControlVelocity+=Velcity_RKp*(Bias1-Last_bias1)+Velcity_RKi*Bias1;  //����ʽPI������
                                                                   //Velcity_Kp*(Bias-Last_bias) ����Ϊ���Ƽ��ٶ�
	                                                                 //Velcity_Ki*Bias             �ٶȿ���ֵ��Bias���ϻ��ֵõ� ƫ��Խ����ٶ�Խ��
		Last_bias1=Bias1;	
		RControlVelocity=pwm_control(RControlVelocity);//�޷�
		return RControlVelocity; //�����ٶȿ���ֵ
}


//λ��ʽpidλ�ÿ���
//��
float p_Err=0,p_last_err=0,Integral=0,p_pwm=0,pid_p=0,pid_i=0,pid_d=0;

float p_pid(int16_t now_position,float tar_position)//��������(�����������)��Ŀ������
{
    now_position=myabs(now_position);//ת������
    p_Err=tar_position-now_position;//Ŀ������-��������=�������
		Integral+=p_Err;
		if(Integral> 970) Integral= 970;	//�����޷�
	  if(Integral<-970) Integral=-970;	//�����޷�
	//kp��������������õ��תһȦ�ͻ�����ڻ�λ��������λ����С����ʱ��Ϳ��Ե�kdѹ�ƹ���
    p_pwm=pid_p*p_Err+pid_i*Integral+pid_d*(p_Err-p_last_err);
    p_pwm=pwm_control(p_pwm);
    p_last_err=p_Err;
    return p_pwm;
}

//λ��ʽpidλ�ÿ��� 
//��
float p_Err1=0,p_last_err1=0,Integral1=0,p_pwm1=0,pid_p1=0,pid_i1=0,pid_d1=0;

float p_pidR(int16_t now_position1,float tar_position1)//��������(�����������)��Ŀ������
{
    now_position1=myabs(now_position1);//ת������
    p_Err1=tar_position1-now_position1;//Ŀ������-��������=�������
		Integral1+=p_Err1;
		if(Integral1> 970) Integral1= 970;	//�����޷�
	  if(Integral1<-970) Integral1=-970;	//�����޷�
	//kp��������������õ��תһȦ�ͻ�����ڻ�λ��������λ����С����ʱ��Ϳ��Ե�kdѹ�ƹ���
    p_pwm1=pid_p1*p_Err1+pid_i1*Integral1+pid_d1*(p_Err1-p_last_err1);
    p_pwm1=pwm_control(p_pwm1);
    p_last_err1=p_Err1;
    return p_pwm1;
}

/**************************************************************************
�������ܣ��ٶ�(PWM)�޷�
��ڲ�����PWM_P:λ�û������PWMֵ TargetVelocity:Ŀ���ٶ�(�ٶ�����ֵ)
����  ֵ����
**************************************************************************/
int   TargetVelocity=60;//�˴��ɵ��ٶ�
int Velocity_Restrict(int PWM_P, int TargetVelocity)
{
    if     (PWM_P>+TargetVelocity*49) PWM_P=+TargetVelocity*49;
	  else if(PWM_P<-TargetVelocity*49) PWM_P=-TargetVelocity*49;
	  else PWM_P=PWM_P;
	  return PWM_P;
}



