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
 * @brief 寻线功能函数，使小车沿黑线行驶
 * @param ADD 当前偏差值（由寻迹传感器获取）
 */
void xunxian(int16_t ADD)
{
    // 设置左轮PID速度环参数
    Velcity_Kp = 0.1;   // 速度环比例系数
    Velcity_Ki = 15;    // 速度环积分系数
    
    // 设置右轮PID速度环参数
    Velcity_RKp = 0.1;  // 速度环比例系数
    Velcity_RKi = 15;   // 速度环积分系数

    // 使用PD控制计算转向修正值
    ADD = turn_pid(ADD, 0);   // 目标偏差为0，即沿线行驶
    
    // 左轮速度 = 基准速度(50) - 修正值，偏差越大，减速越多
    Lpwm = LVelocity_FeedbackControl(50 - ADD, now_Lspeed);
    
    // 右轮速度 = 基准速度(50) + 修正值，形成差速转弯
    Rpwm = RVelocity_FeedbackControl(50 + ADD, now_Rspeed);
    
    // 输出PWM到电机驱动
    right(Rpwm);  // 右轮电机驱动
    left(Lpwm);   // 左轮电机驱动

    // 下面是备用代码，使用固定速度控制（暂不使用）
    /*
    Lpwm = LVelocity_FeedbackControl(25, now_Lspeed);  // 左轮速度控制，固定速度
    Rpwm = RVelocity_FeedbackControl(25, now_Rspeed);  // 右轮速度控制，固定速度
    right(Rpwm);
    left(Lpwm);
    */
}


/**
 * @brief 转弯函数，使小车转向指定角度
 * @param i 目标角度（陀螺仪参考系）
 * @param speed 基准速度
 */
void zhuanwan(float i, float speed)
{
    // 设置速度环PID参数
    Velcity_Kp = 1.2;   // 左轮比例系数
    Velcity_Ki = 15;    // 左轮积分系数
    Velcity_RKp = 1.2;  // 右轮比例系数
    Velcity_RKi = 15;   // 右轮积分系数
    
    // 设置角度环PD参数 - 与直线行驶相比参数不同
    p_p = 0.6;          // 角度环比例系数（比直线行驶小，转弯平稳）
    p_d = 0;            // 角度环微分系数
    
    // 计算当前角度与目标角度的PID控制量
    ADD = turn_pid(zangle, i);
    
    // 左轮速度 = 基准速度 - 修正值，差速转弯
    Lpwm = LVelocity_FeedbackControl(speed - ADD, now_Lspeed);
    // 右轮速度 = 基准速度 + 修正值，差速转弯
    Rpwm = RVelocity_FeedbackControl(speed + ADD, now_Rspeed);
    
    // 输出PWM到电机驱动
    right(Rpwm);
    left(Lpwm);
}

/**
 * @brief 停车函数，使小车平稳停止
 */
void stopcar(void)
{
    // 设置速度环PID参数，用较高增益实现快速制动
    Velcity_Kp = 20;    // 左轮比例系数
    Velcity_Ki = 16;    // 左轮积分系数
    Velcity_RKp = 20;   // 右轮比例系数
    Velcity_RKi = 16;   // 右轮积分系数

    // 速度目标值设为0，实现制动
    Lpwm = LVelocity_FeedbackControl(0, now_Lspeed);  // 左轮速度环，速度环级联控制
    Rpwm = RVelocity_FeedbackControl(0, now_Rspeed);  // 右轮速度环，速度环级联控制
    
    // 输出PWM到电机驱动
    right(Rpwm);
    left(Lpwm);
}


/**
 * @brief 直线行驶函数，保持指定角度直线行驶
 * @param i 目标角度（陀螺仪参考系）
 * @param speed 行驶速度
 */
void zhixian(float i, float speed)
{
    // 设置速度环PID参数
    Velcity_Kp = 1.2;   // 左轮比例系数
    Velcity_Ki = 15;    // 左轮积分系数
    Velcity_RKp = 1.2;  // 右轮比例系数
    Velcity_RKi = 15;   // 右轮积分系数
    
    // 设置角度环PD参数
    p_p = 0.9;          // 角度环比例系数
    p_d = 0;            // 角度环微分系数
    P_angle_i = 0;      // 角度环积分系数（未使用）
    
    // 计算当前角度与目标角度的差值，计算转向修正值
    ADD = turn_pid(zangle, i);   // 当前角度与目标角度
    // 输出PID相关数据供调试
    printf("PID数据: ADD=%d, ", ADD);
		
		Servo_SetAngle_Int16(13+ADD);
    // 左轮速度 = 基准速度 - 修正值
    Lpwm = LVelocity_FeedbackControl(speed - ADD, now_Lspeed);
    // 右轮速度 = 基准速度 + 修正值
    Rpwm = RVelocity_FeedbackControl(speed + ADD, now_Rspeed);
    printf("Lpwm=%d, Rpwm=%d\r\n", Lpwm, Rpwm);
    // 输出PWM到电机驱动
    right(Rpwm);
    left(Lpwm);
}




void posithon_w(int posithon)
{
		 pid_p=-35;pid_d=-120;pid_i=0;
		 pid_p1=35;pid_d1=120;pid_i1=0;
		 Velcity_Ki=10;Velcity_Kp=30;
		 Velcity_RKi=10;Velcity_RKp=30;
		 NowposithonL+=now_Lspeed;//位置环积累积累脉冲
		 NowposithonR+=now_Rspeed;//位置环积累积累脉冲
		 Lpwm=p_pid(NowposithonL,posithon);//积累脉冲(现在脉冲个数)，目标脉冲
		 Rpwm=p_pidR(NowposithonR,posithon);
		 //333(空载转速)/60=5.55转(1s)，每10ms获取的编码器数值5.55 * 30 * 4*11/100=73
		 //3599满载，3599/73=49，1编码器速度=49个pwm
		 
		 Lpwm=Velocity_Restrict(Lpwm,TargetVelocity);//限幅位置环输出的PWM TargetVelocity
		 Rpwm=Velocity_Restrict(Rpwm,TargetVelocity);//限幅位置环输出的PWM TargetVelocity

		 Lpwm=Lpwm/49;//PWM值转换为速度值 49为转换参数
		 Rpwm=Rpwm/49;//PWM值转换为速度值 49为转换参数
		
		 Lpwm=LVelocity_FeedbackControl(Lpwm,now_Lspeed);//左速度环,速度环闭环控制 相当于位置环的输出为速度环的输入，形成串级PID
		 Rpwm =RVelocity_FeedbackControl(Rpwm,now_Rspeed);//右速度环,速度环闭环控制 相当于位置环的输出为速度环的输入，形成串级PID
		 right(Rpwm);
		 left(Lpwm);
}
