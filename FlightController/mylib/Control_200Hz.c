#include "Control_200Hz.h"
#include "stm32f10x_tim.h"
#include "sysconfig.h"
#include "MPU6050.h"
#include "PID.h"
#include "nrf24l01.h"
#include "led.h"
#include "spi.h"
#include "PWM_output.h"
#include "math.h"
#include "delay.h"

/*
//�汾��V1.0
//���ڣ�20180527
//�޸�˵����
*/

#define Speed_Respond_Threshold 1000 //��ֹ�����󴥷�  ��ֹң��ʧ��  ��ң���źŽ���ƽ������
extern volatile float PID_roll_out, PID_pitch_out, PID_yaw_out;
extern volatile float pitch_int, roll_int;
//ǰ��ƫ���С���ֵ���ƫ
extern float roll_angel_offset;
//���Ҽ�飬�Ӵ������ֵ������ƫ
extern float pitch_angel_offset;//��ʼʱ�̶�ƫ��
//vu16�ɶ���д���޷���16λ����
extern vu16 CCR1_Val;						/* ��ʼ������Ƚ�ͨ��1�������ڱ��� */
extern vu16 CCR2_Val;						/* ��ʼ������Ƚ�ͨ��2�������ڱ��� */
extern vu16 CCR3_Val;				    /* ��ʼ������Ƚ�ͨ��3�������ڱ��� */
extern vu16 CCR4_Val;						/* ��ʼ������Ƚ�ͨ��4�������ڱ��� */

extern uint8_t NRF24L01_RXDATA[RX_PLOAD_WIDTH];//nrf24l01���յ�������
extern uint8_t  system_launch_succeed;
uint32_t nrf_check = 0;
volatile uint16_t Speed_FR_Pre = 0;
volatile uint8_t Four_Axis_UNLOCK  = 0;     //һ������ ��ɻ���ֹͣ
volatile uint8_t Four_Axis_landing = 0;
volatile uint8_t High_Loc_En = 0; 					//ʹ�ܶ���
volatile uint16_t Speed_FR = 0;
volatile float Desire_angle_roll = 0,Desire_angle_pitch = 0,Desire_angle_yaw;
volatile uint32_t  nfr_Receive_Fail_Counter = 0;   //���մ����־��0.5sδ���ܵ���������Ϊ������ʧ��
volatile uint16_t  nfr_Receive_Succeed_Counter = 0;
volatile uint16_t  nfr_Receive_Succeed_Counter_Flag = 0;
volatile uint8_t 	Receive_KEY = 0;
volatile static uint8_t 	KEY_Press_Flag = 0;  //���ڰ������µļ���־��
volatile static uint16_t	KEY_Release_Counter = 0;	//�ڰ�������ʱʹ��
volatile float Desire_w_yaw = 0;
volatile uint32_t Sys_Time = 0;
static uint32_t my_tim4_Counter = 0;
volatile uint8_t Counter_200Hz = 0;
//ǰ��ƫ���С���ֵ���ƫ
//float NRFroll_angel_offset;
////���Ҽ�飬�Ӵ������ֵ������ƫ
//float NRFpitch_angel_offset;//NRF���õĳ�ʼʱ�̶�ƫ��

extern volatile float Desire_angle_roll_SUM,Desire_angle_pitch_SUM;

extern volatile int BT_Throttle, BT_Yaw, BT_Pitch, BT_Roll;
//�����������ʱ��ƫ��
float Desire_angle_roll_DIFF = 0;
float Desire_angle_pitch_DIFF = 0;
//for OLED
extern uint16_t pitch,roll,yaw,thr,key1, key2, key3;//ƵĻ���ң����ָ��
extern int8_t OLEDAlter_flag;
//�����λ��ʼֵ
volatile uint16_t Pitch_ServoInitialVal = 280;
volatile uint16_t Roll_ServoInitialVal = 287;


int int_constrain (int val, int low, int high)
{
	if(val > high)
	{
		return high;
	}
	else if(val < low)
	{
		return low;
	}
	else
	{
		return val;
	}
}

float float_constrain (float val, float low, float high)
{
	if(val > high)
	{
		return high;
	}
	else if(val < low)
	{
		return low;
	}
	else
	{
		return val;
	}
}

int int_remap(int val, int Origin_low, int Origin_high, int low, int high)
{
	return ( low + (int)((float)(high - low))/((float)(Origin_high - Origin_low)) * ((float)(val - Origin_low)) );
}

void KEY_SCAN()
{
	key1 = NRF24L01_RXDATA[10];
	key2 = NRF24L01_RXDATA[11];
	key3 = NRF24L01_RXDATA[12];	
}
//�������ϼ�����������������Ե��жϣ��Ӷ����Ƶ������ĸ���Ƶ�ʣ�
//��������Ϊ400Hz�ĸ���Ƶ�ʣ�����ȫ����Ҫ��ġ�
void tim4_init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 , ENABLE);
    TIM_DeInit(TIM4);
    TIM_TimeBaseStructure.TIM_Period=2500;		 								/* �Զ���װ�ؼĴ������ڵ�ֵ(����ֵ) */
    TIM_TimeBaseStructure.TIM_Prescaler= (72 - 1);				    /* ʱ��Ԥ��Ƶ�� 72M/72 */
    TIM_TimeBaseStructure.TIM_ClockDivision=0; 								/* ������Ƶ */
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; /* ���ϼ���ģʽ */
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    TIM_ClearFlag(TIM4, TIM_FLAG_Update);							    		/* �������жϱ�־ */
    TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
    TIM_Cmd(TIM4, ENABLE);																		/* ����ʱ�� */
}

void NRF_Data_Receive(void)
{
	uint8_t i;
	static uint16_t NRF_SPEED, NRF_YAW, NRF_ROLL, NRF_PITCH;
	if(1)			//!GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_2))
	{
		NRF_IRQ();	//���nRF���жϱ�־λ
		nrf_check = 0;
		for(i=0;i<30;i++)
		{
			nrf_check = nrf_check + NRF24L01_RXDATA[i];
		}		
		if((nrf_check < 250*30 ) && (nrf_check != 0 ) && ((NRF24L01_RXDATA[30] == (uint8_t)(nrf_check & 0x00ff)) && (NRF24L01_RXDATA[31] == (uint8_t)((nrf_check & 0xff00) >> 8))))
		{ 
			//�����ݽ���У�黹ԭ
			NRF_SPEED = (uint16_t)NRF24L01_RXDATA[3]*256 + (uint16_t)NRF24L01_RXDATA[2] + 1;
			NRF_YAW = (uint16_t)NRF24L01_RXDATA[5]*256 + (uint16_t)NRF24L01_RXDATA[4];
			NRF_ROLL = 1024 - ((uint16_t)NRF24L01_RXDATA[7]*256 + (uint16_t)NRF24L01_RXDATA[6]);
			NRF_PITCH = 1024 - ((uint16_t)NRF24L01_RXDATA[9]*256 + (uint16_t)NRF24L01_RXDATA[8]);
			
			//for OLEDƵĻ���ң����ԭʼָ��
			thr=NRF_SPEED;
			pitch=NRF_PITCH;
			roll=NRF_ROLL;
			yaw=NRF_YAW;
			
//			NRFroll_angel_offset = ((float)NRF24L01_RXDATA[20] - 128)/16;
//			NRFpitch_angel_offset = ((float)NRF24L01_RXDATA[21] - 128)/16;
			roll_angel_offset = ((float)NRF24L01_RXDATA[20] - 128)/16;
			pitch_angel_offset = ((float)NRF24L01_RXDATA[21] - 128)/16;
			//����Ϊ����PID����ʱʹ�ã����ڿ��Ժ���
//			rollP_IN = (float)NRF24L01_RXDATA[20]/30.1;
//			rollD_IN = (float)NRF24L01_RXDATA[21]/30.1*1.2;		
//			pitchP_IN = rollP_IN;
//			pitchD_IN = rollD_IN;
			
			nfr_Receive_Succeed_Counter_Flag ++;
			Four_Axis_landing = 0;
			if(!Four_Axis_landing)
			{
				if(NRF_SPEED != 0)
				{
					NRF_SPEED = NRF_SPEED/4;
					if(NRF_SPEED < 64)																//3089
					Speed_FR = 2001 + NRF_SPEED*17;
					else if((NRF_SPEED >= 64) && (NRF_SPEED<=192))		//3089-3729
					Speed_FR = 3089 + (NRF_SPEED-64)*5;
					else if( (NRF_SPEED >= 192) && (NRF_SPEED<=255) )	//3729-4200
					Speed_FR = 3729 + (NRF_SPEED-192)*7.5;
				}
			}
			//��ֹ���ŵ�ͻȻ�������½�
			if(Speed_FR_Pre == 0)
			{
				Speed_FR_Pre = Speed_FR;
			}
			if((Speed_FR < (Speed_FR_Pre + Speed_Respond_Threshold)))
				Speed_FR_Pre = Speed_FR;
			else
				Speed_FR = Speed_FR_Pre;

			nfr_Receive_Fail_Counter = 0;	//����ʧ�ܱ�־�����
			
			//��roll����п��ƣ���������������ֵ�������ָ�����
			NRF_ROLL = NRF_ROLL/4;
			if(fabs((float)NRF_ROLL-128.0f)<60)
				Desire_angle_roll = ((float)NRF_ROLL-128.0f)*20.0f/128.0f + Desire_angle_roll_DIFF;		//������+-60��֮��
			else
			{
				if(((float)NRF_ROLL-128.0f) <= -60)
					Desire_angle_roll = -9.3 + ((float)NRF_ROLL-128.0f+60.0f)*30.0f/128.0f + Desire_angle_roll_DIFF;
				else
					Desire_angle_roll = 9.3 + ((float)NRF_ROLL-128.0f-60.0f)*30.0f/128.0f + Desire_angle_roll_DIFF;
			}
			
			//��pitch����п��ƣ���������������ֵ�������ָ�����
			NRF_PITCH = NRF_PITCH/4;
			if(fabs((float)NRF_PITCH-128.0f)<60)
				Desire_angle_pitch = -((float)NRF_PITCH-128.0f)*20.0f/128.0f + Desire_angle_pitch_DIFF;	//������+-60��֮��
			else
			{
				if(((float)NRF_PITCH-128.0f) <= -60)
					Desire_angle_pitch = 9.3 - ((float)NRF_PITCH-128.0f+60.0f)*30.0f/128.0f + Desire_angle_pitch_DIFF;
				else
					Desire_angle_pitch = -9.3 - ((float)NRF_PITCH-128.0f-60.0f)*30.0f/128.0f + Desire_angle_pitch_DIFF;	
			}
			
			//��yaw����п���
			NRF_YAW = NRF_YAW/4;
			Desire_w_yaw = -((float)NRF_YAW - 128.0f)*40.0f/128.0f;	//������0-50��ÿ��֮��
			KEY_SCAN();
		}
		else
		{
			nfr_Receive_Fail_Counter ++;
		}
	}
	Speed_FR = int_constrain (Speed_FR, 2250, 4500);
//	Speed_FR = int_constrain (Speed_FR, 2250, 3000);
}

void PID_UPdata(void)        
{
	static volatile uint32_t Accel_FR_EXC = 0;
	static volatile float PID_K_ACCEL = 0;
		
	//�˴�Ϊ������ǲ���
//	if(fabs(w_and_angle.angle_roll) >= fabs(w_and_angle.angle_pitch))
//	Speed_FR_EXC = fabs(w_and_angle.angle_roll)*5;
//	else
//	Speed_FR_EXC = fabs(w_and_angle.angle_pitch)*5;
	PID_K_ACCEL = 1;
	Accel_FR_EXC = 0;
		
	//������ת�ٿ���
	CCR1_Val = (Speed_FR - PID_yaw_out * PID_K_ACCEL  + Accel_FR_EXC)*4;
//	CCR1_Val = (Speed_FR+ Speed_FR_EXC)*4;
	//����ת�ٳ��������ٶ���ֵ��Χ
	CCR1_Val=int_constrain (CCR1_Val,9000,18000);
//	CCR1_Val=int_remap(CCR1_Val, 9000,18000, 160, 400);
	CCR1_Val=int_remap(CCR1_Val, 9000,18000, 160, 320);
	
	//������ת�ٿ���
	CCR2_Val = (Speed_FR + PID_yaw_out * PID_K_ACCEL  + Accel_FR_EXC)*4;
//	CCR2_Val = (Speed_FR)*4;
	//����ת�ٳ��������ٶ���ֵ��Χ
	CCR2_Val=int_constrain (CCR2_Val,9000,18000);
//	CCR2_Val=int_remap(CCR2_Val, 9000,18000, 160, 400);
	CCR2_Val=int_remap(CCR2_Val, 9000,18000, 160, 320);

	//���ö���������ƽǶ���ֵ��Χ
	CCR3_Val=Pitch_ServoInitialVal - int_remap(  int_constrain( (int)(- PID_pitch_out *PID_K_ACCEL *6),-1200,1200), -1200, 1200, -40, 40);
	

	//���ö���������ƽǶ���ֵ��Χ
	CCR4_Val=Roll_ServoInitialVal + int_remap(int_constrain ((int) ( - PID_roll_out *PID_K_ACCEL *6),-1200,1200), -1200, 1200, -40, 40);
	
	
	if(!Four_Axis_UNLOCK || Speed_FR<2300 || OLEDAlter_flag==1) //����ǰ�������ţ��������ͣת
	{
		TIM2->CCR3= 160;
		TIM2->CCR4= 160;
		TIM3->CCR3= Pitch_ServoInitialVal;
		TIM3->CCR4= Roll_ServoInitialVal;
//		TIM1->CCR3= Pitch_ServoInitialVal;
//		TIM1->CCR2= Roll_ServoInitialVal;
	}
	else		//�Ĵ�����ֵ	
	{
		TIM2->CCR3= CCR1_Val;
		TIM2->CCR4= CCR2_Val;
//		TIM2->CCR3= 80;
//		TIM2->CCR4= 80;
		TIM3->CCR3= CCR3_Val;
		TIM3->CCR4= CCR4_Val;
//		TIM3->CCR3= Pitch_ServoInitialVal;
//		TIM3->CCR4= Roll_ServoInitialVal;
//		TIM1->CCR3= CCR3_Val;
//		TIM1->CCR2= CCR4_Val;
	}
}

//timer4������ʱ�����������ڣ���ʱ�趨Ϊÿ5ms����һ�Σ���ÿ5ms����һ���жϣ�
//���ж��У��������ݵĴ�������PWMռ�ձȵĸ��£��Ӷ��ı���ת�٣�������������̬
void my_tim4_IRQHandler(void)   // ÿ2.5ms����һ��--------------400Hz
{
	if ( TIM_GetITStatus(TIM4 , TIM_IT_Update) != RESET ) 
	{	
		TIM_ClearITPendingBit(TIM4 , TIM_FLAG_Update);
		Sys_Time ++;
		if(nfr_Receive_Fail_Counter > 300)
		{
			//NRF24L01_INIT();
			NRF_Write_Reg(0x27, 0xff);//���nrf���жϱ�־λ
			nfr_Receive_Fail_Counter = 0;
			Speed_FR = 0;
			// ң�����źŽ��մ��󣬹ر�������
			//�˴����Ժ�����Ż�
			LEDALL_OFF;
		}
		if(Sys_Time == 400*60*60*24)//�ɻ�����һ��
		{
			Sys_Time = 0;
		}
		if(Sys_Time%400 == 0)
		{
			nfr_Receive_Succeed_Counter = nfr_Receive_Succeed_Counter_Flag;
			nfr_Receive_Succeed_Counter_Flag = 0;
		}
		if(my_tim4_Counter%2 == 0)
		{
			Counter_200Hz = 1;
			PID_calculate();
			PID_UPdata();
//			pitch_int = Desire_angle_pitch_SUM - w_and_angle.angle_pitch; //pitch���ƫ�����
//			roll_int  = Desire_angle_roll_SUM - w_and_angle.angle_roll;	//roll���ƫ�����
		}
		else if(my_tim4_Counter%40 == 0)
		{	
		}
		if(my_tim4_Counter >= 1000)
		{
			my_tim4_Counter = 0;
		}
		my_tim4_Counter ++;
	}	
}





