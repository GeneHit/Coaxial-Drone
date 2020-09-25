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
//�汾��CoaxialDrone-V1
//���ڣ�20180523
//��˵����
*/

uint8_t key2_flag = 0;
int8_t OLEDAlter_flag=1;

extern W_AND_ANGLE   w_and_angle;	//��sysconfig.h�еĶ������Ӧ�����ⲿ����
extern u16 Speed_FR;				 //λ��Control_200Hz.h�����е��ٶȿ��Ʊ���
extern __IO uint16_t ADCConvertedValue;
extern uint8_t Four_Axis_UNLOCK;     //һ������ ��ɻ���ֹͣ
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

	Four_Axis_UNLOCK  = 0;//һ������ ��ɻ���ֹͣ
	//MPU6050_First = 10;
	Speed_FR = 0;
	My_System_Init(); //���ڸ�������ĳ�ʼ��
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
	delay_init(72);	   	 			//��ʱ��ʼ��

	LED_Init();
	LEDALL_ON;
//	uart_init(38400); 	    	//�����ô��ڳ�ʼ��
	IIC_Init();								//MPU6050��IIC��ʼ��
	MPU6050_Init();						//6050��ʼ��
	MPU6050_Interrupt_Init();	//6050ʹ�õ��ⲿ�жϵ����ų�ʼ��
	C02_IIC_Init();						//AT24C02��IIC��ʼ��
	flag = AT24CXX_Check();		//���AT24Cxx�Ƿ���������
	while(flag)
	{
			LED1_OFF;
			delay_Ms_Loop(200);
			LED1_ON;
			delay_Ms_Loop(200);
	}
	SPI1_INIT();							//SPI��ʼ��������nRFģ��
	flag = NRF_CHECK();				//���NRFģ���Ƿ���������
	LEDALL_OFF;
	while(!flag)
	{
		LED2_OFF;
		delay_Ms_Loop(200);
		LED2_ON;
		delay_Ms_Loop(200);
	}
	NRF24L01_INIT();					//nRF��ʼ��
	SetRX_Mode();							//����Ϊ����ģʽ
	NRF24L01_INIT();					//nRF��ʼ��
	NRF_GPIO_Interrupt_Init();//nRFʹ�õ��ⲿ�жϵ����ų�ʼ��
//	timer1_init();
	timer2_init();						//PWM�����ʱ����ʼ��
	timer3_init();						//PWM�����ʱ����ʼ��
	tim4_init();							//��ʱ�жϣ���Ϊϵͳ�Ŀ���Ƶ��
	OLED_Init();
//	OLED_ShowString(1,16,"OLED Initial Finish");
//	OLED_Refresh_Gram();
	PID_Read();
}

void NVIC_Configuration(void)
{
	//�������������жϵĽṹ�����
	NVIC_InitTypeDef    NVIC_InitStructure;

	/* #ifdef...#else...#endif�ṹ�������Ǹ���Ԥ�������������ж���������ʼ��ַ*/
#ifdef  VECT_TAB_RAM
  /* �ж���������ʼ��ַ�� 0x20000000 ��ʼ */
 	NVIC_SetVectorTable(NVIC_VectTab_RAM , 0x0);
#else
	/* VECT_TAB_FLASH */
  /* �ж���������ʼ��ַ�� 0x80000000 ��ʼ */
  NVIC_SetVectorTable(NVIC_VectTab_FLASH , 0x0);
#endif
	//�ж����ȼ�����  ��ռʽ���ȼ�������Ϊ2λ����Ӧ���ȼ�ռ2λ
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	/* ���� TIM3 �ж�, 0����ռ���ȼ���0����ռ���ȼ���ϵͳ�������� */
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* �����ⲿ�ж�, 2����ռ���ȼ���2����ռ���ȼ���nRF�źŽ����ж� */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* �����ⲿ�ж�, 0����ռ���ȼ���0����ռ���ȼ���6050���ݽ����ж� */
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
	//������ⲿ�жϻ�Ӱ��ʹ��systickд���ӳٺ���������STM32��Ӳ��ȱ�ݣ
	//Ŀǰû���ҵ�����������滻������ʹ��delay_Ms_Loop
	delay_Ms_Loop(1000);
	//�˴��ǳ�ȥMPU6050�ϵ�֮�󼸷��ӵ�yaw�ǶȵĲ���
	while(1)
	{
		delay_Ms_Loop(200);
		LEDALL_ON;
		delay_Ms_Loop(200);
		LEDALL_OFF;
		//���������Ϣ���ֱ��Ǻ����������ƫ���Ƕ�
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

void ModeController(void)//������������while(1)
{
	OLED_ShowString(5,48,"All Ready");
	OLED_Refresh_Gram();
	while(1) //10msѭ��һ��
	{
//		printf("angle_roll:%f\r\n", w_and_angle.angle_roll);
//		printf("angle_pitch:%f\r\n", w_and_angle.angle_pitch);
//		printf("angle_yaw:%f\r\n", w_and_angle.angle_yaw);
		while(Counter_200Hz == 0)	//��ʾ�����ж��������У������ڴ�ѭ��
		{
			LEDALL_ON;
			delay_Ms_Loop(1000);
			LEDALL_OFF;
			delay_Ms_Loop(1000);
		}
		Counter_200Hz = 0;
		if(key3 == 0)	//����ģʽ
		{			
			//��־λ��ʹPWM�����ʼ�������ͣ������λ
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
			//��־λ��ʹPWM���ΪPID����
			OLEDAlter_flag=0;
			LEDALL_ON;
			delay_Ms_Loop(500); //����ģʽ
//			TIM_Cmd(TIM1, ENABLE);	//����PWM���
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
				Desire_angle_yaw = w_and_angle.angle_yaw;	//ʹƫ��ͨ����������ΪĿǰ�Ĳ���ֵ
				TIM_Cmd(TIM2, ENABLE);
				PID_Write();
			}	
		}
			//���������Ϣ���ֱ��Ǻ����������ƫ���Ƕ�
//		printf("angle_roll:%f\r\n", w_and_angle.angle_roll);
//		printf("angle_pitch:%f\r\n", w_and_angle.angle_pitch);
//		printf("angle_yaw:%f\r\n", w_and_angle.angle_yaw);
		key3_pre = key3;
		LEDALL_ON;
	}// end of while(1)
}