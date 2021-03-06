#include "sys.h"
#include "stm32f10x.h"
#include "sysconfig.h"
#include "system_stm32f10x.h"
#include "PID.h"
#include "PWM_output.h"
#include "Control_200Hz.h"
#include "MPU6050.h"
#include "delay.h"
#include "string.h"
#include "stdio.h"
#include "usart.h"
#include "spi.h"
#include "nrf24l01.h"
#include "myiic.h"
#include "led.h"
#include "math.h"
#include "drv_adc.h"
#include "usart2.h"
#include "show.h"
#include "oled.h"
#include "AT24cxx.h"
#include "iic_2.h"

/*
//版本：CoaxialDrone-V1
//日期：20180523
//心说明：
*/

uint8_t key2_flag = 0;
int8_t OLEDAlter_flag=1;

extern W_AND_ANGLE   w_and_angle;	//和sysconfig.h中的定义相对应，是外部变量
extern u16 Speed_FR;				 //位于Control_200Hz.h函数中的速度控制变量
extern __IO uint16_t ADCConvertedValue;
extern uint8_t Four_Axis_UNLOCK;     //一键解锁 起飞或者停止
uint8_t  system_launch_succeed = 0;
extern float Desire_angle_yaw_flag;
extern volatile uint8_t EN_I_Flag;
//extern int8_t MPU6050_First;

extern volatile uint8_t Counter_200Hz;
extern uint16_t key2,key2_pre,key3,key3_pre;
extern float Desire_angle_yaw;
extern uint16_t key1, key2, key3;
extern uint16_t pitch,roll,yaw,thr;

void My_System_Init(void);
void NVIC_Configuration(void);
void Cheak_yaw(void);
void ModeController(void);

int main(void)
{
//	int i;

	Four_Axis_UNLOCK  = 0;//一键解锁 起飞或者停止
	//MPU6050_First = 10;
	Speed_FR = 0;
	My_System_Init(); //对于各种外设的初始化
	LEDALL_OFF;
	Cheak_yaw();
	Four_Axis_UNLOCK = 1;
	LEDALL_OFF;
	ModeController();
}

void My_System_Init(void)
{
	uint8_t flag;
	NVIC_Configuration();
	delay_init(72);	   	 			//延时初始化

	LED_Init();
	LEDALL_ON;
//	uart_init(38400); 	    	//调试用串口初始化
	IIC_Init();								//MPU6050的IIC初始化
	MPU6050_Init();						//6050初始化
	MPU6050_Interrupt_Init();	//6050使用的外部中断的引脚初始化
	C02_IIC_Init();						//AT24C02的IIC初始化
	flag = AT24CXX_Check();		//检查AT24Cxx是否正常工作
	while(flag)
	{
			LED1_OFF;
			delay_Ms_Loop(200);
			LED1_ON;
			delay_Ms_Loop(200);
	}
	SPI1_INIT();							//SPI初始化，用于nRF模块
	flag = NRF_CHECK();				//检查NRF模块是否正常工作
	LEDALL_OFF;
	while(!flag)
	{
		LED2_OFF;
		delay_Ms_Loop(200);
		LED2_ON;
		delay_Ms_Loop(200);
	}
	NRF24L01_INIT();					//nRF初始化
	SetRX_Mode();							//设置为接收模式
	NRF24L01_INIT();					//nRF初始化
	NRF_GPIO_Interrupt_Init();//nRF使用的外部中断的引脚初始化
//	timer1_init();
	timer2_init();						//PWM输出定时器初始化
	timer3_init();						//PWM输出定时器初始化
	tim4_init();							//定时中断，作为系统的控制频率
	OLED_Init();
//	OLED_ShowString(1,16,"OLED Initial Finish");
//	OLED_Refresh_Gram();
	PID_Read();
}

void NVIC_Configuration(void)
{
	//定义用于配置中断的结构体变量
	NVIC_InitTypeDef    NVIC_InitStructure;

	/* #ifdef...#else...#endif结构的作用是根据预编译条件决定中断向量表起始地址*/
#ifdef  VECT_TAB_RAM
  /* 中断向量表起始地址从 0x20000000 开始 */
 	NVIC_SetVectorTable(NVIC_VectTab_RAM , 0x0);
#else
	/* VECT_TAB_FLASH */
  /* 中断向量表起始地址从 0x80000000 开始 */
  NVIC_SetVectorTable(NVIC_VectTab_FLASH , 0x0);
#endif
	//中断优先级分组  抢占式优先级别设置为2位；响应优先级占2位
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	/* 开启 TIM3 中断, 0级先占优先级，0级后占优先级，系统控制周期 */
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* 开启外部中断, 2级先占优先级，2级后占优先级，nRF信号接收中断 */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* 开启外部中断, 0级先占优先级，0级后占优先级，6050数据接收中断 */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}

void Cheak_yaw(void)
{
	float yaw_pre = 0, yaw_now = 0;
	
	OLED_ShowString(5,0,"Sys Init Finish");
	OLED_ShowString(5,16,"Waiting IMU");
	OLED_Refresh_Gram();
	Desire_angle_yaw_flag = w_and_angle.angle_yaw;
	//下面的外部中断会影响使用systick写的延迟函数，这是STM32的硬件缺陷�
	//目前没有找到解决方法。替换方法：使用delay_Ms_Loop
	delay_Ms_Loop(1000);
	//此处是除去MPU6050上电之后几分钟的yaw角度的波动
	while(1)
	{
		delay_Ms_Loop(200);
		LEDALL_ON;
		delay_Ms_Loop(200);
		LEDALL_OFF;
		//输出调试信息，分别是横滚，俯仰，偏航角度
//		printf("angle_roll:%f\r\n", w_and_angle.angle_roll);
//		printf("angle_pitch:%f\r\n", w_and_angle.angle_pitch);
//		printf("angle_yaw:%f\r\n", w_and_angle.angle_yaw);		
		OLED_ShowString(5,32,"Y");
		OLED_ShowNum_withsymbol(40,32,w_and_angle.angle_yaw,3,12);	
		OLED_Refresh_Gram();
		yaw_now = w_and_angle.angle_yaw;
		if( (fabs(yaw_now - yaw_pre) > 0.9) || (yaw_now == 0) )
		{
		}
		else
		{
			break;
		}
		if(yaw_now == 0)
		{			
		}
		yaw_pre = yaw_now;
	}
}

void ModeController(void)//这是主函数的while(1)
{
	OLED_ShowString(5,48,"All Ready");
	OLED_Refresh_Gram();
	while(1) //10ms循环一次
	{
//		printf("angle_roll:%f\r\n", w_and_angle.angle_roll);
//		printf("angle_pitch:%f\r\n", w_and_angle.angle_pitch);
//		printf("angle_yaw:%f\r\n", w_and_angle.angle_yaw);
		while(Counter_200Hz == 0)	//表示控制中断正常运行，否则在此循环
		{
			LEDALL_ON;
			delay_Ms_Loop(1000);
			LEDALL_OFF;
			delay_Ms_Loop(1000);
		}
		Counter_200Hz = 0;
		if(key3 == 0)	//调试模式
		{			
			//标志位，使PWM输出初始，电机最低，舵机零位
			if( key3 != key3_pre )
			{
				TIM_Cmd(TIM2, DISABLE);	
//				TIM_Cmd(TIM3, DISABLE);
//				OLEDAlter_flag=1;
			}
			OLEDAlter_flag=1;
			LEDALL_ON;
			delay_Ms_Loop(10);
			LEDALL_OFF;
			delay_Ms_Loop(10);
			if(key2 != key2_pre && key2==1)
			{
				key2_flag ++;
				OLED_Clear();
			}
			if (key2_flag == 3)
			{
				key2_flag = 0;
			}
			key2_pre = key2;
			
			if (key2_flag == 0)
			{	
				oledshow_Ifno();
			}else if (key2_flag == 1)
			{	
				oledshow_AlterPIDValub();
			}else
			{	
				oledshow_AlterIMUOffset();
			}
		}	
		else{
			//标志位，使PWM输出为PID控制
			OLEDAlter_flag=0;
			LEDALL_ON;
			delay_Ms_Loop(500); //飞行模式
//			TIM_Cmd(TIM1, ENABLE);	//重启PWM输出
//			TIM_Cmd(TIM2, ENABLE);
//			TIM_Cmd(TIM3, ENABLE);
//			OLED_Clear();
//			OLED_ShowString(5,0,"Ready for Fly");
//			OLED_Refresh_Gram();
			if(key3 != key3_pre )
			{
				OLED_Clear();
				OLED_ShowString(5,0,"Ready for Fly");
				OLED_Refresh_Gram();
				Desire_angle_yaw = w_and_angle.angle_yaw;	//使偏航通道的期望角为目前的测量值
				TIM_Cmd(TIM2, ENABLE);
				PID_Write();
			}	
		}
			//输出调试信息，分别是横滚，俯仰，偏航角度
//		printf("angle_roll:%f\r\n", w_and_angle.angle_roll);
//		printf("angle_pitch:%f\r\n", w_and_angle.angle_pitch);
//		printf("angle_yaw:%f\r\n", w_and_angle.angle_yaw);
		key3_pre = key3;
		LEDALL_ON;
	}// end of while(1)
}