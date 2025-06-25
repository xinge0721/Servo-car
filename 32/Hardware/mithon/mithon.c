#include "mithon.h"
#include "pid.h"
#include "Motor.h"
#include "MotorRun.h"
#include "key.h"
#include "sys.h"
#include "mithon.h"
#include "led.h"
#include "MotorRun.h"
#include "Serial.h"
#include "UART2.h"
#include "Servo.h"
int Lpwm=0,Rpwm=0;
int16_t ADD=0;
int16_t now_Lspeed=0,now_Rspeed=0;
int now_Lposition,now_Rposition;
extern int zanglex;
int NowposithonL;
int NowposithonR;
int Nowposithonall;

/**
 * @brief Ѱ�߹��ܺ�����ʹС���غ�����ʻ
 * @param ADD ��ǰƫ��ֵ����Ѱ����������ȡ��
 */
void xunxian(int16_t ADD)
{
    // ��������PID�ٶȻ�����
    Velcity_Kp = 0.1;   // �ٶȻ�����ϵ��
    Velcity_Ki = 15;    // �ٶȻ�����ϵ��
    
    // ��������PID�ٶȻ�����
    Velcity_RKp = 0.1;  // �ٶȻ�����ϵ��
    Velcity_RKi = 15;   // �ٶȻ�����ϵ��

    // ʹ��PD���Ƽ���ת������ֵ
    ADD = turn_pid(ADD, 0);   // Ŀ��ƫ��Ϊ0����������ʻ
    
    // �����ٶ� = ��׼�ٶ�(50) - ����ֵ��ƫ��Խ�󣬼���Խ��
    Lpwm = LVelocity_FeedbackControl(50 - ADD, now_Lspeed);
    
    // �����ٶ� = ��׼�ٶ�(50) + ����ֵ���γɲ���ת��
    Rpwm = RVelocity_FeedbackControl(50 + ADD, now_Rspeed);
    
    // ���PWM���������
    right(Rpwm);  // ���ֵ������
    left(Lpwm);   // ���ֵ������

    // �����Ǳ��ô��룬ʹ�ù̶��ٶȿ��ƣ��ݲ�ʹ�ã�
    /*
    Lpwm = LVelocity_FeedbackControl(25, now_Lspeed);  // �����ٶȿ��ƣ��̶��ٶ�
    Rpwm = RVelocity_FeedbackControl(25, now_Rspeed);  // �����ٶȿ��ƣ��̶��ٶ�
    right(Rpwm);
    left(Lpwm);
    */
}


/**
 * @brief ת�亯����ʹС��ת��ָ���Ƕ�
 * @param i Ŀ��Ƕȣ������ǲο�ϵ��
 * @param speed ��׼�ٶ�
 */
void zhuanwan(float i, float speed)
{
    // �����ٶȻ�PID����
    Velcity_Kp = 1.2;   // ���ֱ���ϵ��
    Velcity_Ki = 15;    // ���ֻ���ϵ��
    Velcity_RKp = 1.2;  // ���ֱ���ϵ��
    Velcity_RKi = 15;   // ���ֻ���ϵ��
    
    // ���ýǶȻ�PD���� - ��ֱ����ʻ��Ȳ�����ͬ
    p_p = 0.6;          // �ǶȻ�����ϵ������ֱ����ʻС��ת��ƽ�ȣ�
    p_d = 0;            // �ǶȻ�΢��ϵ��
    
    // ���㵱ǰ�Ƕ���Ŀ��Ƕȵ�PID������
    ADD = turn_pid(zangle, i);
    
    // �����ٶ� = ��׼�ٶ� - ����ֵ������ת��
    Lpwm = LVelocity_FeedbackControl(speed - ADD, now_Lspeed);
    // �����ٶ� = ��׼�ٶ� + ����ֵ������ת��
    Rpwm = RVelocity_FeedbackControl(speed + ADD, now_Rspeed);
    
    // ���PWM���������
    right(Rpwm);
    left(Lpwm);
}

/**
 * @brief ͣ��������ʹС��ƽ��ֹͣ
 */
void stopcar(void)
{
    // �����ٶȻ�PID�������ýϸ�����ʵ�ֿ����ƶ�
    Velcity_Kp = 20;    // ���ֱ���ϵ��
    Velcity_Ki = 16;    // ���ֻ���ϵ��
    Velcity_RKp = 20;   // ���ֱ���ϵ��
    Velcity_RKi = 16;   // ���ֻ���ϵ��

    // �ٶ�Ŀ��ֵ��Ϊ0��ʵ���ƶ�
    Lpwm = LVelocity_FeedbackControl(0, now_Lspeed);  // �����ٶȻ����ٶȻ���������
    Rpwm = RVelocity_FeedbackControl(0, now_Rspeed);  // �����ٶȻ����ٶȻ���������
    
    // ���PWM���������
    right(Rpwm);
    left(Lpwm);
}


/**
 * @brief ֱ����ʻ����������ָ���Ƕ�ֱ����ʻ
 * @param i Ŀ��Ƕȣ������ǲο�ϵ��
 * @param speed ��ʻ�ٶ�
 */
void zhixian(float i, float speed)
{
    // �����ٶȻ�PID����
    Velcity_Kp = 1.2;   // ���ֱ���ϵ��
    Velcity_Ki = 15;    // ���ֻ���ϵ��
    Velcity_RKp = 1.2;  // ���ֱ���ϵ��
    Velcity_RKi = 15;   // ���ֻ���ϵ��
    
    // ���ýǶȻ�PD����
    p_p = 0.9;          // �ǶȻ�����ϵ��
    p_d = 0;            // �ǶȻ�΢��ϵ��
    P_angle_i = 0;      // �ǶȻ�����ϵ����δʹ�ã�
    
    // ���㵱ǰ�Ƕ���Ŀ��ǶȵĲ�ֵ������ת������ֵ
    ADD = turn_pid(zangle, i);   // ��ǰ�Ƕ���Ŀ��Ƕ�
    // ���PID������ݹ�����
    printf("PID����: ADD=%d, ", ADD);
		
		Servo_SetAngle_Int16(13+ADD);
    // �����ٶ� = ��׼�ٶ� - ����ֵ
    Lpwm = LVelocity_FeedbackControl(speed - ADD, now_Lspeed);
    // �����ٶ� = ��׼�ٶ� + ����ֵ
    Rpwm = RVelocity_FeedbackControl(speed + ADD, now_Rspeed);
    printf("Lpwm=%d, Rpwm=%d\r\n", Lpwm, Rpwm);
    // ���PWM���������
    right(Rpwm);
    left(Lpwm);
}




void posithon_w(int posithon)
{
		 pid_p=-35;pid_d=-120;pid_i=0;
		 pid_p1=35;pid_d1=120;pid_i1=0;
		 Velcity_Ki=10;Velcity_Kp=30;
		 Velcity_RKi=10;Velcity_RKp=30;
		 NowposithonL+=now_Lspeed;//λ�û����ۻ�������
		 NowposithonR+=now_Rspeed;//λ�û����ۻ�������
		 Lpwm=p_pid(NowposithonL,posithon);//��������(�����������)��Ŀ������
		 Rpwm=p_pidR(NowposithonR,posithon);
		 //333(����ת��)/60=5.55ת(1s)��ÿ10ms��ȡ�ı�������ֵ5.55 * 30 * 4*11/100=73
		 //3599���أ�3599/73=49��1�������ٶ�=49��pwm
		 
		 Lpwm=Velocity_Restrict(Lpwm,TargetVelocity);//�޷�λ�û������PWM TargetVelocity
		 Rpwm=Velocity_Restrict(Rpwm,TargetVelocity);//�޷�λ�û������PWM TargetVelocity

		 Lpwm=Lpwm/49;//PWMֵת��Ϊ�ٶ�ֵ 49Ϊת������
		 Rpwm=Rpwm/49;//PWMֵת��Ϊ�ٶ�ֵ 49Ϊת������
		
		 Lpwm=LVelocity_FeedbackControl(Lpwm,now_Lspeed);//���ٶȻ�,�ٶȻ��ջ����� �൱��λ�û������Ϊ�ٶȻ������룬�γɴ���PID
		 Rpwm =RVelocity_FeedbackControl(Rpwm,now_Rspeed);//���ٶȻ�,�ٶȻ��ջ����� �൱��λ�û������Ϊ�ٶȻ������룬�γɴ���PID
		 right(Rpwm);
		 left(Lpwm);
}
