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
//版本：V1.0
//日期：20180527
//修改说明：
*/

#define Speed_Respond_Threshold 1000 //防止油门误触发  防止遥控失灵  对遥控信号进行平滑处理
extern volatile float PID_roll_out, PID_pitch_out, PID_yaw_out;
extern volatile float pitch_int, roll_int;
//前后偏差，减小这个值向后偏
extern float roll_angel_offset;
//左右检查，加大下面的值会向左偏
extern float pitch_angel_offset;//初始时固定偏移
//vu16可读可写的无符号16位数据
extern vu16 CCR1_Val;						/* 初始化输出比较通道1计数周期变量 */
extern vu16 CCR2_Val;						/* 初始化输出比较通道2计数周期变量 */
extern vu16 CCR3_Val;				    /* 初始化输出比较通道3计数周期变量 */
extern vu16 CCR4_Val;						/* 初始化输出比较通道4计数周期变量 */

extern uint8_t NRF24L01_RXDATA[RX_PLOAD_WIDTH];//nrf24l01接收到的数据
extern uint8_t  system_launch_succeed;
uint32_t nrf_check = 0;
volatile uint16_t Speed_FR_Pre = 0;
volatile uint8_t Four_Axis_UNLOCK  = 0;     //一键解锁 起飞或者停止
volatile uint8_t Four_Axis_landing = 0;
volatile uint8_t High_Loc_En = 0; 					//使能定高
volatile uint16_t Speed_FR = 0;
volatile float Desire_angle_roll = 0,Desire_angle_pitch = 0,Desire_angle_yaw;
volatile uint32_t  nfr_Receive_Fail_Counter = 0;   //接收错误标志，0.5s未接受到数据则认为飞行器失控
volatile uint16_t  nfr_Receive_Succeed_Counter = 0;
volatile uint16_t  nfr_Receive_Succeed_Counter_Flag = 0;
volatile uint8_t 	Receive_KEY = 0;
volatile static uint8_t 	KEY_Press_Flag = 0;  //对于按键按下的检测标志量
volatile static uint16_t	KEY_Release_Counter = 0;	//在按键消抖时使用
volatile float Desire_w_yaw = 0;
volatile uint32_t Sys_Time = 0;
static uint32_t my_tim4_Counter = 0;
volatile uint8_t Counter_200Hz = 0;
//前后偏差，减小这个值向后偏
//float NRFroll_angel_offset;
////左右检查，加大下面的值会向左偏
//float NRFpitch_angel_offset;//NRF设置的初始时固定偏移

extern volatile float Desire_angle_roll_SUM,Desire_angle_pitch_SUM;

extern volatile int BT_Throttle, BT_Yaw, BT_Pitch, BT_Roll;
//用来矫正起飞时的偏差
float Desire_angle_roll_DIFF = 0;
float Desire_angle_pitch_DIFF = 0;
//for OLED
extern uint16_t pitch,roll,yaw,thr,key1, key2, key3;//频幕输出遥控器指令
extern int8_t OLEDAlter_flag;
//舵机中位初始值
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
//采用向上计数溢出来产生周期性的中断，从而控制电机脉宽的更新频率，
//现在设置为400Hz的更新频率，是完全满足要求的。
void tim4_init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 , ENABLE);
    TIM_DeInit(TIM4);
    TIM_TimeBaseStructure.TIM_Period=2500;		 								/* 自动重装载寄存器周期的值(计数值) */
    TIM_TimeBaseStructure.TIM_Prescaler= (72 - 1);				    /* 时钟预分频数 72M/72 */
    TIM_TimeBaseStructure.TIM_ClockDivision=0; 								/* 采样分频 */
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; /* 向上计数模式 */
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    TIM_ClearFlag(TIM4, TIM_FLAG_Update);							    		/* 清除溢出中断标志 */
    TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
    TIM_Cmd(TIM4, ENABLE);																		/* 开启时钟 */
}

void NRF_Data_Receive(void)
{
	uint8_t i;
	static uint16_t NRF_SPEED, NRF_YAW, NRF_ROLL, NRF_PITCH;
	if(1)			//!GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_2))
	{
		NRF_IRQ();	//清除nRF的中断标志位
		nrf_check = 0;
		for(i=0;i<30;i++)
		{
			nrf_check = nrf_check + NRF24L01_RXDATA[i];
		}		
		if((nrf_check < 250*30 ) && (nrf_check != 0 ) && ((NRF24L01_RXDATA[30] == (uint8_t)(nrf_check & 0x00ff)) && (NRF24L01_RXDATA[31] == (uint8_t)((nrf_check & 0xff00) >> 8))))
		{ 
			//对数据进行校验还原
			NRF_SPEED = (uint16_t)NRF24L01_RXDATA[3]*256 + (uint16_t)NRF24L01_RXDATA[2] + 1;
			NRF_YAW = (uint16_t)NRF24L01_RXDATA[5]*256 + (uint16_t)NRF24L01_RXDATA[4];
			NRF_ROLL = 1024 - ((uint16_t)NRF24L01_RXDATA[7]*256 + (uint16_t)NRF24L01_RXDATA[6]);
			NRF_PITCH = 1024 - ((uint16_t)NRF24L01_RXDATA[9]*256 + (uint16_t)NRF24L01_RXDATA[8]);
			
			//for OLED频幕输出遥控器原始指令
			thr=NRF_SPEED;
			pitch=NRF_PITCH;
			roll=NRF_ROLL;
			yaw=NRF_YAW;
			
//			NRFroll_angel_offset = ((float)NRF24L01_RXDATA[20] - 128)/16;
//			NRFpitch_angel_offset = ((float)NRF24L01_RXDATA[21] - 128)/16;
			roll_angel_offset = ((float)NRF24L01_RXDATA[20] - 128)/16;
			pitch_angel_offset = ((float)NRF24L01_RXDATA[21] - 128)/16;
			//以下为调试PID参数时使用，现在可以忽略
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
			//防止油门的突然上升和下降
			if(Speed_FR_Pre == 0)
			{
				Speed_FR_Pre = Speed_FR;
			}
			if((Speed_FR < (Speed_FR_Pre + Speed_Respond_Threshold)))
				Speed_FR_Pre = Speed_FR;
			else
				Speed_FR = Speed_FR_Pre;

			nfr_Receive_Fail_Counter = 0;	//接收失败标志量清除
			
			//对roll轴进行控制，其中我设置了阈值，操作手感提升
			NRF_ROLL = NRF_ROLL/4;
			if(fabs((float)NRF_ROLL-128.0f)<60)
				Desire_angle_roll = ((float)NRF_ROLL-128.0f)*20.0f/128.0f + Desire_angle_roll_DIFF;		//限制在+-60度之内
			else
			{
				if(((float)NRF_ROLL-128.0f) <= -60)
					Desire_angle_roll = -9.3 + ((float)NRF_ROLL-128.0f+60.0f)*30.0f/128.0f + Desire_angle_roll_DIFF;
				else
					Desire_angle_roll = 9.3 + ((float)NRF_ROLL-128.0f-60.0f)*30.0f/128.0f + Desire_angle_roll_DIFF;
			}
			
			//对pitch轴进行控制，其中我设置了阈值，操作手感提升
			NRF_PITCH = NRF_PITCH/4;
			if(fabs((float)NRF_PITCH-128.0f)<60)
				Desire_angle_pitch = -((float)NRF_PITCH-128.0f)*20.0f/128.0f + Desire_angle_pitch_DIFF;	//限制在+-60度之内
			else
			{
				if(((float)NRF_PITCH-128.0f) <= -60)
					Desire_angle_pitch = 9.3 - ((float)NRF_PITCH-128.0f+60.0f)*30.0f/128.0f + Desire_angle_pitch_DIFF;
				else
					Desire_angle_pitch = -9.3 - ((float)NRF_PITCH-128.0f-60.0f)*30.0f/128.0f + Desire_angle_pitch_DIFF;	
			}
			
			//对yaw轴进行控制
			NRF_YAW = NRF_YAW/4;
			Desire_w_yaw = -((float)NRF_YAW - 128.0f)*40.0f/128.0f;	//限制在0-50度每秒之内
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
		
	//此处为进行倾角补偿
//	if(fabs(w_and_angle.angle_roll) >= fabs(w_and_angle.angle_pitch))
//	Speed_FR_EXC = fabs(w_and_angle.angle_roll)*5;
//	else
//	Speed_FR_EXC = fabs(w_and_angle.angle_pitch)*5;
	PID_K_ACCEL = 1;
	Accel_FR_EXC = 0;
		
	//下旋翼转速控制
	CCR1_Val = (Speed_FR - PID_yaw_out * PID_K_ACCEL  + Accel_FR_EXC)*4;
//	CCR1_Val = (Speed_FR+ Speed_FR_EXC)*4;
	//不让转速超出控制速度阈值范围
	CCR1_Val=int_constrain (CCR1_Val,9000,18000);
//	CCR1_Val=int_remap(CCR1_Val, 9000,18000, 160, 400);
	CCR1_Val=int_remap(CCR1_Val, 9000,18000, 160, 320);
	
	//上旋翼转速控制
	CCR2_Val = (Speed_FR + PID_yaw_out * PID_K_ACCEL  + Accel_FR_EXC)*4;
//	CCR2_Val = (Speed_FR)*4;
	//不让转速超出控制速度阈值范围
	CCR2_Val=int_constrain (CCR2_Val,9000,18000);
//	CCR2_Val=int_remap(CCR2_Val, 9000,18000, 160, 400);
	CCR2_Val=int_remap(CCR2_Val, 9000,18000, 160, 320);

	//不让舵机超出控制角度阈值范围
	CCR3_Val=Pitch_ServoInitialVal - int_remap(  int_constrain( (int)(- PID_pitch_out *PID_K_ACCEL *6),-1200,1200), -1200, 1200, -40, 40);
	

	//不让舵机超出控制角度阈值范围
	CCR4_Val=Roll_ServoInitialVal + int_remap(int_constrain ((int) ( - PID_roll_out *PID_K_ACCEL *6),-1200,1200), -1200, 1200, -40, 40);
	
	
	if(!Four_Axis_UNLOCK || Speed_FR<2300 || OLEDAlter_flag==1) //解锁前锁定油门，设置马达停转
	{
		TIM2->CCR3= 160;
		TIM2->CCR4= 160;
		TIM3->CCR3= Pitch_ServoInitialVal;
		TIM3->CCR4= Roll_ServoInitialVal;
//		TIM1->CCR3= Pitch_ServoInitialVal;
//		TIM1->CCR2= Roll_ServoInitialVal;
	}
	else		//寄存器赋值	
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

//timer4用来定时产生控制周期，暂时设定为每5ms控制一次，即每5ms进入一次中断，
//在中断中，进行数据的处理和输出PWM占空比的更新，从而改变电机转速，控制四旋翼姿态
void my_tim4_IRQHandler(void)   // 每2.5ms进来一次--------------400Hz
{
	if ( TIM_GetITStatus(TIM4 , TIM_IT_Update) != RESET ) 
	{	
		TIM_ClearITPendingBit(TIM4 , TIM_FLAG_Update);
		Sys_Time ++;
		if(nfr_Receive_Fail_Counter > 300)
		{
			//NRF24L01_INIT();
			NRF_Write_Reg(0x27, 0xff);//清除nrf的中断标志位
			nfr_Receive_Fail_Counter = 0;
			Speed_FR = 0;
			// 遥控器信号接收错误，关闭马达输出
			//此处在以后进行优化
			LEDALL_OFF;
		}
		if(Sys_Time == 400*60*60*24)//飞机飞了一天
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
//			pitch_int = Desire_angle_pitch_SUM - w_and_angle.angle_pitch; //pitch轴的偏差积分
//			roll_int  = Desire_angle_roll_SUM - w_and_angle.angle_roll;	//roll轴的偏差积分
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





