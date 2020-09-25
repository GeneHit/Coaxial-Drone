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
//°æ±¾£ºCoaxialDrone-V1
//ÈÕÆÚ£º20180523
//ÐÄËµÃ÷£º
*/

uint8_t key2_flag = 0;
int8_t OLEDAlter_flag=1;

extern W_AND_ANGLE   w_and_angle;	//ºÍsysconfig.hÖÐµÄ¶¨ÒåÏà¶ÔÓ¦£¬ÊÇÍâ²¿±äÁ¿
extern u16 Speed_FR;				 //Î»ÓÚControl_200Hz.hº¯ÊýÖÐµÄËÙ¶È¿ØÖÆ±äÁ¿
extern __IO uint16_t ADCConvertedValue;
extern uint8_t Four_Axis_UNLOCK;     //Ò»¼ü½âËø Æð·É»òÕßÍ£Ö¹
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

	Four_Axis_UNLOCK  = 0;//Ò»¼ü½âËø Æð·É»òÕßÍ£Ö¹
	//MPU6050_First = 10;
	Speed_FR = 0;
	My_System_Init(); //¶ÔÓÚ¸÷ÖÖÍâÉèµÄ³õÊ¼»¯
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
	delay_init(72);	   	 			//ÑÓÊ±³õÊ¼»¯

	LED_Init();
	LEDALL_ON;
//	uart_init(38400); 	    	//µ÷ÊÔÓÃ´®¿Ú³õÊ¼»¯
	IIC_Init();								//MPU6050µÄIIC³õÊ¼»¯
	MPU6050_Init();						//6050³õÊ¼»¯
	MPU6050_Interrupt_Init();	//6050Ê¹ÓÃµÄÍâ²¿ÖÐ¶ÏµÄÒý½Å³õÊ¼»¯
	C02_IIC_Init();						//AT24C02µÄIIC³õÊ¼»¯
	flag = AT24CXX_Check();		//¼ì²éAT24CxxÊÇ·ñÕý³£¹¤×÷
	while(flag)
	{
			LED1_OFF;
			delay_Ms_Loop(200);
			LED1_ON;
			delay_Ms_Loop(200);
	}
	SPI1_INIT();							//SPI³õÊ¼»¯£¬ÓÃÓÚnRFÄ£¿é
	flag = NRF_CHECK();				//¼ì²éNRFÄ£¿éÊÇ·ñÕý³£¹¤×÷
	LEDALL_OFF;
	while(!flag)
	{
		LED2_OFF;
		delay_Ms_Loop(200);
		LED2_ON;
		delay_Ms_Loop(200);
	}
	NRF24L01_INIT();					//nRF³õÊ¼»¯
	SetRX_Mode();							//ÉèÖÃÎª½ÓÊÕÄ£Ê½
	NRF24L01_INIT();					//nRF³õÊ¼»¯
	NRF_GPIO_Interrupt_Init();//nRFÊ¹ÓÃµÄÍâ²¿ÖÐ¶ÏµÄÒý½Å³õÊ¼»¯
//	timer1_init();
	timer2_init();						//PWMÊä³ö¶¨Ê±Æ÷³õÊ¼»¯
	timer3_init();						//PWMÊä³ö¶¨Ê±Æ÷³õÊ¼»¯
	tim4_init();							//¶¨Ê±ÖÐ¶Ï£¬×÷ÎªÏµÍ³µÄ¿ØÖÆÆµÂÊ
	OLED_Init();
//	OLED_ShowString(1,16,"OLED Initial Finish");
//	OLED_Refresh_Gram();
	PID_Read();
}

void NVIC_Configuration(void)
{
	//¶¨ÒåÓÃÓÚÅäÖÃÖÐ¶ÏµÄ½á¹¹Ìå±äÁ¿
	NVIC_InitTypeDef    NVIC_InitStructure;

	/* #ifdef...#else...#endif½á¹¹µÄ×÷ÓÃÊÇ¸ù¾ÝÔ¤±àÒëÌõ¼þ¾ö¶¨ÖÐ¶ÏÏòÁ¿±íÆðÊ¼µØÖ·*/
#ifdef  VECT_TAB_RAM
  /* ÖÐ¶ÏÏòÁ¿±íÆðÊ¼µØÖ·´Ó 0x20000000 ¿ªÊ¼ */
 	NVIC_SetVectorTable(NVIC_VectTab_RAM , 0x0);
#else
	/* VECT_TAB_FLASH */
  /* ÖÐ¶ÏÏòÁ¿±íÆðÊ¼µØÖ·´Ó 0x80000000 ¿ªÊ¼ */
  NVIC_SetVectorTable(NVIC_VectTab_FLASH , 0x0);
#endif
	//ÖÐ¶ÏÓÅÏÈ¼¶·Ö×é  ÇÀÕ¼Ê½ÓÅÏÈ¼¶±ðÉèÖÃÎª2Î»£»ÏìÓ¦ÓÅÏÈ¼¶Õ¼2Î»
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	/* ¿ªÆô TIM3 ÖÐ¶Ï, 0¼¶ÏÈÕ¼ÓÅÏÈ¼¶£¬0¼¶ºóÕ¼ÓÅÏÈ¼¶£¬ÏµÍ³¿ØÖÆÖÜÆÚ */
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* ¿ªÆôÍâ²¿ÖÐ¶Ï, 2¼¶ÏÈÕ¼ÓÅÏÈ¼¶£¬2¼¶ºóÕ¼ÓÅÏÈ¼¶£¬nRFÐÅºÅ½ÓÊÕÖÐ¶Ï */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* ¿ªÆôÍâ²¿ÖÐ¶Ï, 0¼¶ÏÈÕ¼ÓÅÏÈ¼¶£¬0¼¶ºóÕ¼ÓÅÏÈ¼¶£¬6050Êý¾Ý½ÓÊÕÖÐ¶Ï */
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
	//ÏÂÃæµÄÍâ²¿ÖÐ¶Ï»áÓ°ÏìÊ¹ÓÃsystickÐ´µÄÑÓ³Ùº¯Êý£¬ÕâÊÇSTM32µÄÓ²¼þÈ±ÏÝ£
	//Ä¿Ç°Ã»ÓÐÕÒµ½½â¾ö·½·¨¡£Ìæ»»·½·¨£ºÊ¹ÓÃdelay_Ms_Loop
	delay_Ms_Loop(1000);
	//´Ë´¦ÊÇ³ýÈ¥MPU6050ÉÏµçÖ®ºó¼¸·ÖÖÓµÄyaw½Ç¶ÈµÄ²¨¶¯
	while(1)
	{
		delay_Ms_Loop(200);
		LEDALL_ON;
		delay_Ms_Loop(200);
		LEDALL_OFF;
		//Êä³öµ÷ÊÔÐÅÏ¢£¬·Ö±ðÊÇºá¹ö£¬¸©Ñö£¬Æ«º½½Ç¶È
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

void ModeController(void)//ÕâÊÇÖ÷º¯ÊýµÄwhile(1)
{
	OLED_ShowString(5,48,"All Ready");
	OLED_Refresh_Gram();
	while(1) //10msÑ­»·Ò»´Î
	{
//		printf("angle_roll:%f\r\n", w_and_angle.angle_roll);
//		printf("angle_pitch:%f\r\n", w_and_angle.angle_pitch);
//		printf("angle_yaw:%f\r\n", w_and_angle.angle_yaw);
		while(Counter_200Hz == 0)	//±íÊ¾¿ØÖÆÖÐ¶ÏÕý³£ÔËÐÐ£¬·ñÔòÔÚ´ËÑ­»·
		{
			LEDALL_ON;
			delay_Ms_Loop(1000);
			LEDALL_OFF;
			delay_Ms_Loop(1000);
		}
		Counter_200Hz = 0;
		if(key3 == 0)	//µ÷ÊÔÄ£Ê½
		{			
			//±êÖ¾Î»£¬Ê¹PWMÊä³ö³õÊ¼£¬µç»ú×îµÍ£¬¶æ»úÁãÎ»
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
			//±êÖ¾Î»£¬Ê¹PWMÊä³öÎªPID¿ØÖÆ
			OLEDAlter_flag=0;
			LEDALL_ON;
			delay_Ms_Loop(500); //·ÉÐÐÄ£Ê½
//			TIM_Cmd(TIM1, ENABLE);	//ÖØÆôPWMÊä³ö
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
				Desire_angle_yaw = w_and_angle.angle_yaw;	//Ê¹Æ«º½Í¨µÀµÄÆÚÍû½ÇÎªÄ¿Ç°µÄ²âÁ¿Öµ
				TIM_Cmd(TIM2, ENABLE);
				PID_Write();
			}	
		}
			//Êä³öµ÷ÊÔÐÅÏ¢£¬·Ö±ðÊÇºá¹ö£¬¸©Ñö£¬Æ«º½½Ç¶È
//		printf("angle_roll:%f\r\n", w_and_angle.angle_roll);
//		printf("angle_pitch:%f\r\n", w_and_angle.angle_pitch);
//		printf("angle_yaw:%f\r\n", w_and_angle.angle_yaw);
		key3_pre = key3;
		LEDALL_ON;
	}// end of while(1)
}