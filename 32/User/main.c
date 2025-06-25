#include <string.h>
#include <stdio.h>
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "misc.h"           
#include "OLED.h"
#include "Timer.h"
#include "Motor.h"
#include "MotorRun.h"
#include "Encoder.h"
#include "delay.h"
#include "led.h"
#include "key.h"
#include "pid.h"
#include "Serial.h"
#include "UART2.h"

#include "PWM.h"
#include "sys.h"
#include "APP.h"
#include "mithon.h"
#include "REG.h"
#include "wit_c_sdk.h"
#include "stm32f10x.h"                  // Device header
#include "mithon.h"
#include "Servo.h"
#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80
uint8_t a[8]={1,1,1,1,1,1,1,1};
static volatile char s_cDataUpdate = 0, s_cCmd = 0xff;
const uint32_t c_uiBaud[10] = {0, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};
static void CmdProcess(void);
static void AutoScanSensor(void);
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);
static void wete_angle(void);
unsigned char Send_Count; //串口需要发送的数据个数
unsigned char j;          //计数变量
float zangle;
int zanglex;
int Gyroz;
int keynum;
int keyflag=5;
int A;

uint8_t rr;
int stopflag;
uint8_t Beep_flag=0;
uint8_t fuzhi=1;
int main(void)
{
	Stm32_Clock_Init(9);            //系统时钟设置
	delay_init(72);                 //延时初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置中断优先级分组为组2：2位抢占优先级，2位响应优先级
	uart4_init_DMA(115200);		                      //串口4初始化，陀螺仪
	Usart1Init(115200);
	OLED_Init();				//OLED初始化   PD6/PD7
	Key_Init();					//按键初始化   PE4  
	LED_Init();                  // PB5/PB1
	Timer_Init();				//定时器初始化 TIM4
	Motor_Init();				 //车轮引脚和PWM初始化            右PD0/PB13  左PB14/PB15
	Encoder_Init_Right();		//编码器右                PC6/PC7/TIM8
	Encoder_Init_Left();		//编码器左                 PA6/PA7/TIM3
	u5_Init(115200);			//蓝牙
	WitSerialWriteRegister(SensorUartSend);
	WitRegisterCallBack(SensorDataUpdata);
	WitDelayMsRegister(Delayms);

	Servo_Init();
//	printf("\r\n********************** wit-motion normal example  ************************\r\n");
	AutoScanSensor();

//	
    // OLED显示初始化
    OLED_ShowString(1, 1, "R:");  // 右编码器速度
    OLED_ShowString(2, 1, "L:");  // 左编码器速度
    OLED_ShowString(3, 1, "all:"); // 编码器速度和
    OLED_ShowString(4, 1, "err:"); // 偏差显示

	while (1)
	{    
			keyflag=fuzhi;
       		//屏幕数据
			OLED_ShowSignedNum(1, 3, now_Rspeed, 5);//右编码器速度
			OLED_ShowSignedNum(2, 3, now_Lspeed, 5);//左编码器速度
			OLED_ShowSignedNum(3, 5, ADD, 4); //转向环系数
			OLED_ShowSignedNum(4, 3,zangle , 3); //转向环系数

	}
}

uint8_t timeflag;uint8_t tt;
uint8_t uu;
int Lpwm1;
int Rpwm2;
uint8_t Time_flag=0;
void TIM4_IRQHandler(void)//1ms
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)//1ms
	{

			Servo_Tick_Handler();
			now_Rspeed = Encoder_Get_Right();//电机
			now_Lspeed =  -Encoder_Get_Left();//电机 
			wete_angle();//陀螺仪读取
			Time_flag=0;
			zhixian(1,0);
			


				
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	}
}

/************
陀螺仪函数
以下代码，不到万不得已，不动
************/
void wete_angle(void)
{
    float fAcc[3], fGyro[3], fAngle[3];
	int i;
	CmdProcess();
		if(s_cDataUpdate)
		{
			   for(i = 0; i < 3; i++)
			   {
			    	fAcc[i] = sReg[AX+i] / 32768.0f * 16.0f;
				    fGyro[i] = sReg[GX+i] / 32768.0f * 2000.0f;
			   	  	fAngle[i] = sReg[Roll+i] / 32768.0f * 180.0f;
			   }
			   if(s_cDataUpdate & ACC_UPDATE)
			   {
//				    printf("acc:%.3f %.3f %.3f\r\n", fAcc[0], fAcc[1], fAcc[2]);
				    s_cDataUpdate &= ~ACC_UPDATE;
			   }
			   if(s_cDataUpdate & GYRO_UPDATE)
			   {
					 zanglex=fGyro[2];
//			   	printf("gyro:%.3f %.3f %.3f\r\n", fGyro[0], fGyro[1], fGyro[2]);
                Gyroz = (int)(fGyro[2]);//陀螺仪z角速度
			   	s_cDataUpdate &= ~GYRO_UPDATE;
			   }
			   if(s_cDataUpdate & ANGLE_UPDATE)
			   {
                zangle=(fAngle[2]);//陀螺仪z轴角度
			   	//printf("angle:%.3f %.3f %.3f\r\n", fAngle[0], fAngle[1], fAngle[2]);//x,y,z 暂时注释掉陀螺仪数据输出
			   	s_cDataUpdate &= ~ANGLE_UPDATE;
					 
			   }
		}
}


void CopeCmdData(unsigned char ucData)
{
	 static unsigned char s_ucData[50], s_ucRxCnt = 0;
	
	 s_ucData[s_ucRxCnt++] = ucData;
	 if(s_ucRxCnt<3)return;										//Less than three data returned
	 if(s_ucRxCnt >= 50) s_ucRxCnt = 0;
	 if(s_ucRxCnt >= 3)
	 {
		 if((s_ucData[1] == '\r') && (s_ucData[2] == '\n'))
		 {
		  	s_cCmd = s_ucData[0];
			  memset(s_ucData,0,50);//
			  s_ucRxCnt = 0;
	   } 
		 else 
		 {
			 s_ucData[0] = s_ucData[1];
			 s_ucData[1] = s_ucData[2];
			 s_ucRxCnt = 2;
			}
	  }
}

static void ShowHelp(void)
{
	printf("\r\n************************	 WIT_SDK_DEMO	************************");
	printf("\r\n************************          HELP           ************************\r\n");
	printf("UART SEND:a\\r\\n   Acceleration calibration.\r\n");
	printf("UART SEND:m\\r\\n   Magnetic field calibration,After calibration send:   e\\r\\n   to indicate the end\r\n");
	printf("UART SEND:U\\r\\n   Bandwidth increase.\r\n");
	printf("UART SEND:u\\r\\n   Bandwidth reduction.\r\n");
	printf("UART SEND:B\\r\\n   Baud rate increased to 115200.\r\n");
	printf("UART SEND:b\\r\\n   Baud rate reduction to 9600.\r\n");
	printf("UART SEND:R\\r\\n   The return rate increases to 10Hz.\r\n");
	printf("UART SEND:r\\r\\n   The return rate reduction to 1Hz.\r\n");
	printf("UART SEND:C\\r\\n   Basic return content: acceleration, angular velocity, angle, magnetic field.\r\n");
	printf("UART SEND:c\\r\\n   Return content: acceleration.\r\n");
	printf("UART SEND:h\\r\\n   help.\r\n");
	printf("******************************************************************************\r\n");
}

static void CmdProcess(void)
{
	switch(s_cCmd)
	{
		case 'a':	
			if(WitStartAccCali() != WIT_HAL_OK) 
				printf("\r\nSet AccCali Error\r\n");
			break;
		case 'm':	
			if(WitStartMagCali() != WIT_HAL_OK) 
				printf("\r\nSet MagCali Error\r\n");
			break;
		case 'e':	
			if(WitStopMagCali() != WIT_HAL_OK)
				printf("\r\nSet MagCali Error\r\n");
			break;
		case 'u':	
			if(WitSetBandwidth(BANDWIDTH_5HZ) != WIT_HAL_OK) 
				printf("\r\nSet Bandwidth Error\r\n");
			break;
		case 'U':	
			if(WitSetBandwidth(BANDWIDTH_256HZ) != WIT_HAL_OK) 
				printf("\r\nSet Bandwidth Error\r\n");
			break;
		case 'B':	
			if(WitSetUartBaud(WIT_BAUD_115200) != WIT_HAL_OK) 
				printf("\r\nSet Baud Error\r\n");
			else 
				Usart4Init(c_uiBaud[WIT_BAUD_115200]);											
			break;
		case 'b':	
			if(WitSetUartBaud(WIT_BAUD_9600) != WIT_HAL_OK)
				printf("\r\nSet Baud Error\r\n");
			else 
				Usart4Init(c_uiBaud[WIT_BAUD_9600]);												
			break;
		case 'R':	
			if(WitSetOutputRate(RRATE_10HZ) != WIT_HAL_OK) 
				printf("\r\nSet Rate Error\r\n");
			break;
		case 'r':	
			if(WitSetOutputRate(RRATE_1HZ) != WIT_HAL_OK) 
				printf("\r\nSet Rate Error\r\n");
			break;
		case 'C':	
			if(WitSetContent(RSW_ACC|RSW_GYRO|RSW_ANGLE|RSW_MAG) != WIT_HAL_OK) 
				printf("\r\nSet RSW Error\r\n");
			break;
		case 'c':	
			if(WitSetContent(RSW_ACC) != WIT_HAL_OK) 
				printf("\r\nSet RSW Error\r\n");
			break;
		case 'h':
			ShowHelp();
			break;
	}
	s_cCmd = 0xff;
}

static void SensorUartSend(uint8_t *p_data, uint32_t uiSize)
{
	Uart4Send(p_data, uiSize);
}

static void Delayms(uint16_t ucMs)
{
	delay_ms(ucMs);
}

static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
{
	int i;
    for(i = 0; i < uiRegNum; i++)
    {
        switch(uiReg)
        {
//            case AX:
//            case AY:
            case AZ:
				s_cDataUpdate |= ACC_UPDATE;
            break;
//            case GX:
//            case GY:
            case GZ:
				s_cDataUpdate |= GYRO_UPDATE;
            break;
//            case HX:
//            case HY:
            case HZ:
				s_cDataUpdate |= MAG_UPDATE;
            break;
//            case Roll:
//            case Pitch:
            case Yaw:
				s_cDataUpdate |= ANGLE_UPDATE;
            break;
            default:
				s_cDataUpdate |= READ_UPDATE;
			break;
        }
		uiReg++;
    }
}

static void AutoScanSensor(void)
{
	int i, iRetry;
	
	for(i = 1; i < 10; i++)
	{
		Usart4Init(c_uiBaud[i]);
		iRetry = 2;
		do
		{
			s_cDataUpdate = 0;
			WitReadReg(AX, 3);
			delay_ms(100);
			if(s_cDataUpdate != 0)
			{
				printf("%d baud find sensor\r\n\r\n", c_uiBaud[i]);
				ShowHelp();
				return ;
			}
			iRetry--;
		}while(iRetry);		
	}
	printf("can not find sensor\r\n");
	printf("please check your connection\r\n");
}

