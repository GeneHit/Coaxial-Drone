
//计算PID的输出值
#include "PID.h"
#include "led.h"
#include "sysconfig.h"
#include "math.h"
#include "nrf24l01.h"
#include "AT24cxx.h"
#include "Control_200Hz.h"

/*
//版本：V1.0
//日期：20180527
//修改说明：
*/

float rollP_EXC=-10.5,rollI_EXC=-0.05,rollD_EXC=0,rollP_IN=0.90,rollI_IN=0,rollD_IN=0;
float pitchP_EXC=-10,pitchI_EXC=-0.06,pitchD_EXC=0,pitchP_IN=1.0,pitchI_IN=0,pitchD_IN=0;
float yawP_EXC=4,yawI_EXC=0.1,yawD_EXC=0.0,yawP_IN=1,yawI_IN=0.0,yawD_IN=0.0;

extern float pitch_angel_offset, roll_angel_offset;
extern	volatile uint16_t Pitch_ServoInitialVal;
extern volatile uint16_t Roll_ServoInitialVal;
extern W_AND_ANGLE w_and_angle;	//和sysconfig中的定义相对应，是外部变量,此变量存在于MPU6050.c中								
extern float Desire_angle_roll,Desire_angle_pitch,Desire_angle_yaw,Desire_w_yaw;
extern uint8_t Four_Axis_UNLOCK;     //一键解锁 起飞或者停止
extern uint16_t roll;

volatile float Desire_angle_yaw_flag = 3.0;	
	
volatile float Desire_angle_roll_SUM = 0;
volatile float Now_Angle_roll_Err=0,Pre1_Angle_roll_Err=0.0,Pre2_Angle_roll_Err=0.0;
volatile float Desire_W_roll = 0,D_Desire_W_roll=0.0,Pre_Desire_W_roll=0.0;
volatile float Now_W_roll_Err=0.0,Pre1_W_roll_Err=0.0,Pre2_W_roll_Err=0.0;
volatile float PID_roll_out = 0.0,D_PID_roll_out=0.0,Pre_PID_roll_out=0.0;
volatile float MaxRoll=200,MinRoll=-200;

volatile float Desire_angle_pitch_SUM = 0;
volatile float Now_Angle_pitch_Err=0,Pre1_Angle_pitch_Err=0.0,Pre2_Angle_pitch_Err=0.0;
volatile float Desire_W_pitch = 0,D_Desire_W_pitch=0.0,Pre_Desire_W_pitch=0.0;
volatile float Now_W_pitch_Err = 0,Pre1_W_pitch_Err=0.0,Pre2_W_pitch_Err=0.0;
volatile float PID_pitch_out = 0.0,D_PID_pitch_out=0.0,Pre_PID_pitch_out=0.0;
volatile float MaxPitch=200,MinPitch=-200;

volatile float Pre_Desire_w_yaw=0.0;
volatile float Now_angle_yaw = 0,Now_w_yaw=0;
volatile float Now_Angle_yaw_Err=0.0,Pre1_Angle_yaw_Err,Pre2_Angle_yaw_Err;
volatile float D_Desire_W_yaw=0.0,Desire_w_yawCal=0.0,Pre_Desire_w_yawCal=0.0;
volatile float Now_W_yaw_Err=0,Pre1_W_yaw_Err=0.0,Pre2_W_yaw_Err=0.0;
volatile float PID_yaw_out = 0.0,D_PID_yaw_out=0.0,Pre_PID_yaw_out=0.0;
volatile float MaxYaw=2000,MinYaw=-2000;


void PID_calculate(void)	
{
//roll的PID计算************************
	//对期望偏角进行限制
	//-40*0.45度~40*0.45度
	Desire_angle_roll_SUM  = float_constrain(Desire_angle_roll, -15,15);
	//当前角度误差
	Now_Angle_roll_Err  = (Desire_angle_roll_SUM  - w_and_angle.angle_roll);
	D_Desire_W_roll= (Now_Angle_roll_Err-Pre1_Angle_roll_Err)*rollP_EXC + Now_Angle_roll_Err*rollI_EXC +(Now_Angle_roll_Err - 2*Pre1_Angle_roll_Err+Pre2_Angle_roll_Err)*rollD_EXC;
	Pre1_Angle_roll_Err = Now_Angle_roll_Err;
	Pre2_Angle_roll_Err = Pre1_Angle_roll_Err;
	Desire_W_roll=Pre_Desire_W_roll+D_Desire_W_roll;
	Pre_Desire_W_roll=Desire_W_roll;
	Now_W_roll_Err= Desire_W_roll- w_and_angle.w_roll;
	D_PID_roll_out=-(Now_W_roll_Err-Pre1_W_roll_Err)*rollP_IN +Now_W_roll_Err*rollI_IN -(Now_W_roll_Err - 2*Pre1_W_roll_Err+Pre2_W_roll_Err)*rollD_IN;
	Pre1_W_roll_Err = Now_W_roll_Err;
	Pre2_W_roll_Err = Pre1_W_roll_Err;
	PID_roll_out =Pre_PID_roll_out+D_PID_roll_out;
	if(PID_roll_out>MaxRoll)
		PID_roll_out = MaxRoll;
	if(PID_roll_out<MinRoll)
		PID_roll_out = MinRoll;
	Pre_PID_roll_out=PID_roll_out;
//**************************************
//pitch的PID计算************************
	Desire_angle_pitch_SUM = float_constrain(Desire_angle_pitch,-15,15);
	Now_Angle_pitch_Err = (Desire_angle_pitch_SUM - w_and_angle.angle_pitch);
	D_Desire_W_pitch = (Now_Angle_pitch_Err-Pre1_Angle_pitch_Err)*pitchP_EXC +Now_Angle_pitch_Err*pitchI_EXC +(Now_Angle_pitch_Err-2*Pre1_Angle_pitch_Err+Pre2_Angle_pitch_Err)*pitchD_EXC;
	Pre1_Angle_pitch_Err = Now_Angle_pitch_Err;
	Pre2_Angle_pitch_Err = Pre1_Angle_pitch_Err;
	Desire_W_pitch=Pre_Desire_W_pitch+D_Desire_W_pitch;
	Pre_Desire_W_pitch=Desire_W_pitch;
	Now_W_pitch_Err = Desire_W_pitch - w_and_angle.w_pitch;
	D_PID_pitch_out=(-Now_W_pitch_Err-Pre1_W_pitch_Err)*pitchP_IN +Now_W_pitch_Err*pitchI_IN -(Now_W_pitch_Err - 2*Pre1_W_pitch_Err+Pre2_W_pitch_Err)*pitchD_IN;
	Pre1_W_pitch_Err = Now_W_pitch_Err;
	Pre2_W_pitch_Err = Pre1_W_pitch_Err;
	PID_pitch_out =Pre_PID_pitch_out+D_PID_pitch_out;
	if(PID_pitch_out>MaxPitch)
		PID_pitch_out = MaxPitch;
	if(PID_pitch_out<MinPitch)
		PID_pitch_out = MinPitch;
	Pre_PID_pitch_out=PID_pitch_out;
//**************************************
//yaw的PID计算**************************
//对yaw角度的处理，yaw是一个角速度和角度的闭环稳定系统
	Now_angle_yaw = w_and_angle.angle_yaw; 
	Now_w_yaw = w_and_angle.w_yaw; 
	if((fabs(Desire_w_yaw) < 10.0))	//控制量为角度
	{
		if( fabs(Pre_Desire_w_yaw)> 10.0f ){
			Desire_angle_yaw=Now_angle_yaw;
		}
		if(Desire_angle_yaw >= 180) { 
			if(  ( Desire_angle_yaw-Now_angle_yaw ) >=180  ) {
				Now_Angle_yaw_Err = Desire_angle_yaw-Now_angle_yaw-360;
			} 
			else { 
				Now_Angle_yaw_Err = Desire_angle_yaw-Now_angle_yaw ; 
			} 
		}
		else{ 
			if( ( Now_angle_yaw - Desire_angle_yaw ) >= 180 ) {
				Now_Angle_yaw_Err = Desire_angle_yaw - Now_angle_yaw+360; 
			} else {
				Now_Angle_yaw_Err = Desire_angle_yaw-Now_angle_yaw ; 
			} 
		}
		D_Desire_W_yaw = (Now_Angle_yaw_Err-Pre1_Angle_yaw_Err)*yawP_EXC+Now_Angle_yaw_Err*yawI_EXC +(Now_Angle_yaw_Err - 2*Pre1_Angle_yaw_Err+Pre2_Angle_yaw_Err)*yawD_EXC;
		Pre2_Angle_yaw_Err=Pre1_Angle_yaw_Err;
		Pre1_Angle_yaw_Err=Now_Angle_yaw_Err;
		Desire_w_yawCal=Pre_Desire_w_yawCal+D_Desire_W_yaw;
	}else{
		Desire_w_yawCal=Desire_w_yaw;
		Desire_angle_yaw=Now_angle_yaw;
	}
	Now_W_yaw_Err= Desire_w_yawCal- w_and_angle.w_yaw;
	D_PID_yaw_out=(Now_W_yaw_Err-Pre1_W_yaw_Err)*yawP_IN +Now_W_yaw_Err*yawI_IN +(Now_W_yaw_Err - 2*Pre1_W_yaw_Err+Pre2_W_yaw_Err)*yawD_IN;
	PID_yaw_out =Pre_PID_yaw_out+D_PID_yaw_out;
	if(PID_yaw_out>MaxYaw)
		PID_yaw_out = MaxYaw;
	if(PID_yaw_out<MinYaw)
		PID_yaw_out = MinYaw;
	Pre1_W_yaw_Err = Now_W_yaw_Err;
	Pre2_W_yaw_Err = Pre1_W_yaw_Err;
	Pre_PID_yaw_out=PID_yaw_out;
	Pre_Desire_w_yaw=Desire_w_yaw;
	Pre_Desire_w_yawCal=Desire_w_yawCal;
//**************************************
	if(!Four_Axis_UNLOCK )
	{
		Desire_angle_yaw_flag = w_and_angle.angle_yaw;
		PID_yaw_out = 0;
		PID_pitch_out = 0;
		PID_roll_out = 0;
	}
	
}

void PID_Tune (float *Param)
{
	if(roll > 560)
	{
		if(roll < 700)
		{
			*Param = *Param - 0.001;
		}
		else if(roll < 1000)
		{
			*Param = *Param - 0.01;
		}
		else
		{
			*Param = *Param - 0.1;
		}
	}
	if(roll < 450)
	{
		if(roll > 300)
		{
			*Param = *Param + 0.001;
		}
		else if(roll > 150)
		{
			*Param = *Param + 0.01;
		}
		else
		{
			*Param = *Param + 0.1;
		}
	}
}

void INT_Tune (int *Param)
{
	
		if(roll > 650)
	{
			*Param = *Param - 1;
	}
	if(roll < 350)
	{
			*Param = *Param + 1;
	}
	
}


void PID_Write (void)
{
//	float P_ServoIniValFlag=0,R_ServoIniValFlag=0;
		
	AT24CXX_Write_Float(0,rollP_IN);
	AT24CXX_Write_Float(3,rollI_IN);
	AT24CXX_Write_Float(6,rollD_IN);
	AT24CXX_Write_Float(9,rollP_EXC);
	AT24CXX_Write_Float(12,rollI_EXC);
	AT24CXX_Write_Float(15,rollD_EXC);
	AT24CXX_Write_Float(18,pitchP_IN);
	AT24CXX_Write_Float(21,pitchI_IN);
	AT24CXX_Write_Float(24,pitchD_IN);
	AT24CXX_Write_Float(27,pitchP_EXC);
	AT24CXX_Write_Float(30,pitchI_EXC);
	AT24CXX_Write_Float(33,pitchD_EXC);
	AT24CXX_Write_Float(36,yawP_IN);
	AT24CXX_Write_Float(39,yawI_IN);
	AT24CXX_Write_Float(42,yawD_IN);
	AT24CXX_Write_Float(45,yawP_EXC);
	AT24CXX_Write_Float(48,yawI_EXC);
	AT24CXX_Write_Float(51,yawD_EXC);
	AT24CXX_Write_Float(54,pitch_angel_offset);
	AT24CXX_Write_Float(57,roll_angel_offset);	
	
//	P_ServoIniValFlag=Pitch_ServoInitialVal;
//	R_ServoIniValFlag=Roll_ServoInitialVal;
//	AT24CXX_Write_Float(60,P_ServoIniValFlag);
//	AT24CXX_Write_Float(63,R_ServoIniValFlag);	
}	

void PID_Read (void)
{
//	float P_ServoIniValFlag=0,R_ServoIniValFlag=0;
	
	AT24CXX_Read_Float(0,&rollP_IN);
	AT24CXX_Read_Float(3,&rollI_IN);
	AT24CXX_Read_Float(6,&rollD_IN);
	AT24CXX_Read_Float(9,&rollP_EXC);
	AT24CXX_Read_Float(12,&rollI_EXC);
	AT24CXX_Read_Float(15,&rollD_EXC);
	AT24CXX_Read_Float(18,&pitchP_IN);
	AT24CXX_Read_Float(21,&pitchI_IN);
	AT24CXX_Read_Float(24,&pitchD_IN);
	AT24CXX_Read_Float(27,&pitchP_EXC);
	AT24CXX_Read_Float(30,&pitchI_EXC);
	AT24CXX_Read_Float(33,&pitchD_EXC);
	AT24CXX_Read_Float(36,&yawP_IN);
	AT24CXX_Read_Float(39,&yawI_IN);
	AT24CXX_Read_Float(42,&yawD_IN);
	AT24CXX_Read_Float(45,&yawP_EXC);
	AT24CXX_Read_Float(48,&yawI_EXC);
	AT24CXX_Read_Float(51,&yawD_EXC);
	AT24CXX_Read_Float(54,&pitch_angel_offset);
	AT24CXX_Read_Float(57,&roll_angel_offset);
//	AT24CXX_Read_Float(60,&P_ServoIniValFlag);
//	AT24CXX_Read_Float(63,&R_ServoIniValFlag);
//	Pitch_ServoInitialVal=P_ServoIniValFlag;
//	Roll_ServoInitialVal=R_ServoIniValFlag;
}	




