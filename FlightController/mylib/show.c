#include "show.h"
#include "PID.h"
#include "AT24cxx.h"
#include "sysconfig.h"
#include "mpu6050.h"

float mpitch=0,mroll=0;

int8_t key1_flag = 0;
uint8_t Buffer = 0;
uint16_t pitch=0,roll=0,yaw=0,thr=0;
uint16_t key1=0, key2=0, key3=0, key1_pre=0, key2_pre=0, key3_pre=0;

extern W_AND_ANGLE  w_and_angle;
extern int CCR1_Val, CCR2_Val, CCR3_Val, CCR4_Val;	
extern float rollP_IN, rollI_IN, rollD_IN;
extern float rollP_EXC, rollI_EXC, rollD_EXC;
extern float pitchP_EXC, pitchI_EXC, pitchD_EXC;
extern float pitchP_IN, pitchI_IN, pitchD_IN;
extern float yawP_EXC,yawI_EXC,yawD_EXC,yawP_IN,yawI_IN,yawD_IN;
//extern float w_yawP, w_yawI, w_yawD;
//extern float yawP, yawI, yawD;
extern float pitch_angel_offset, roll_angel_offset;
extern volatile uint16_t Speed_FR;
extern volatile float PID_roll_out, PID_pitch_out, PID_yaw_out;// Error_Angle_yaw
extern volatile float Desire_angle_roll, Desire_angle_pitch, Desire_angle_yaw, Desire_w_yaw;
extern volatile float Now_Angle_yaw_Err;
extern volatile uint16_t Pitch_ServoInitialVal,Roll_ServoInitialVal;
//for test
void oledshow(void)
{
	
//	if(w_and_angle.angle_pitch<0)
//	{	
//	OLED_ShowString(5,0, "mp -");
//	mpitch=-w_and_angle.angle_pitch;
//	OLED_ShowNum(35,0,mpitch,3,12);
//	}
//	else
//	{
//		OLED_ShowString(5,0, "mp +");
//		OLED_ShowNum(35,0,w_and_angle.angle_pitch,3,12);
//	}
	
//	if(w_and_angle.angle_roll<0)
//	{	
//	OLED_ShowString(68,0, "mr -");
//	mroll=-w_and_angle.angle_roll;
//	OLED_ShowNum(98,0,mroll,3,12);
//	}
//	else
//	{
//	OLED_ShowString(68,0, "mr +");
//	OLED_ShowNum(98,0,w_and_angle.angle_roll,3,12);
//	}

	OLED_ShowString(5,0, "K1");
	OLED_ShowNum(35,0,key1,3,12);

	OLED_ShowString(68,0, "K2");
	OLED_ShowNum(98,0,key2,3,12);
	
	OLED_ShowString(5,16, "K3");
	OLED_ShowNum(35,16,key3,3,12);
		
// 	OLED_ShowString(10,16,"NRF SUCCESSFUL"); 

	OLED_ShowString(5,32,"thr");
	OLED_ShowNum(35,32,thr,3,12);
	
	OLED_ShowString(68,32,"pit");
	OLED_ShowNum(98,32,pitch,3,12);
	
	OLED_ShowString(5,48,"yaw");
	OLED_ShowNum(35,48,yaw,3,12);
	
	OLED_ShowString(68,48,"rol");
	OLED_ShowNum(98,48,roll,3,12);
	
	//刷新
	OLED_Refresh_Gram();

}

void oledshow_Ifno(void)
{
//	float temp;
	if(key1 != key1_pre&& key1==1)
	{
		key1_flag ++;
		OLED_Clear();
	}
	if(key1_flag >= 7)
	{
		key1_flag = 1;
	}
	key1_pre = key1;

	if (key1_flag == 1)
	{	
		OLED_ShowString(5,0,"System Info 1/6");
		OLED_ShowString(5,16,"Output of IMU");

		OLED_ShowString(5,32,"P");
		OLED_ShowFloat(20,32,w_and_angle.angle_pitch,3,12);
		
		OLED_ShowString(65,32,"R");
		OLED_ShowFloat(80,32,w_and_angle.angle_roll,3,12);
		
		OLED_ShowString(5,48,"Y");
		OLED_ShowFloat(20,48,w_and_angle.angle_yaw,3,12);
		
		OLED_ShowString(65,48,"EY");
		OLED_ShowFloat(80,48,Now_Angle_yaw_Err,3,12);
		
//		OLED_ShowString(68,48,"Buff");
//		OLED_ShowNum_withsymbol(103,48,AT24CXX_ReadOneByte(32),3,12);
//		AT24CXX_Read_Float(33,&temp);
//		OLED_ShowNum_withsymbol(103,48,temp,3,12);	//AT24读写测试
		
//		//输出调试信息，分别是横滚，俯仰，偏航角度
//		printf("angle_roll:%f\r\n", w_and_angle.angle_roll);
//		printf("angle_pitch:%f\r\n", w_and_angle.angle_pitch);
//		printf("angle_yaw:%f\r\n", w_and_angle.angle_yaw);
		
	}
	else if (key1_flag == 2)
	{
		OLED_ShowString(5,0,"System Info 2/6");
		OLED_ShowString(5,16,"Output of IMU");

		OLED_ShowString(5,32,"WP");
		OLED_ShowNum_withsymbol(40,32,w_and_angle.w_pitch,3,12);
		
		OLED_ShowString(68,32,"WR");
		OLED_ShowNum_withsymbol(103,32,w_and_angle.w_roll,3,12);
		
		OLED_ShowString(5,48,"WY");
		OLED_ShowNum_withsymbol(40,48,w_and_angle.w_yaw,3,12);
	}
	else if (key1_flag == 3)
	{
		OLED_ShowString(5,0,"System Info 3/6");
		OLED_ShowString(5,16,"Input of R-C");
		
		OLED_ShowString(5,32,"Thr");
		OLED_ShowNum(35,32,thr,4,12);
		
		OLED_ShowString(68,32,"Pit");
		OLED_ShowNum(103,32,pitch,3,12);
		
		OLED_ShowString(5,48,"Yaw");
		OLED_ShowNum(40,48,yaw,3,12);
		
		OLED_ShowString(68,48,"Rol");
		OLED_ShowNum(103,48,roll,3,12);
	}
	else if (key1_flag == 4)
	{
		OLED_ShowString(5,0,"System Info 4/6");
		OLED_ShowString(5,16,"Desired Input");

		OLED_ShowString(5,32,"Thr");
		OLED_ShowNum(35,32,Speed_FR,4,12);
		
		OLED_ShowString(68,32,"AofP");
		OLED_ShowNum_withsymbol(103,32,Desire_angle_pitch,3,12);
		
		OLED_ShowString(5,48,"WofY");
		OLED_ShowNum_withsymbol(40,48,Desire_w_yaw,3,12);
		
		OLED_ShowString(68,48,"A0fR");
		OLED_ShowNum_withsymbol(103,48,Desire_angle_roll,3,12);
		
	}
	else if (key1_flag == 5)
	{
		OLED_ShowString(5,0,"System Info 5/6");
		OLED_ShowString(5,16,"Output of PID");
		
		OLED_ShowString(5,32,"Ch-R");
		OLED_ShowNum_withsymbol(43,32,PID_roll_out,3,12);
		
		OLED_ShowString(68,32,"Ch-P");
		OLED_ShowNum_withsymbol(106,32,PID_pitch_out,3,12);
				
		OLED_ShowString(5,48,"Ch-Y");
		OLED_ShowNum_withsymbol(43,48,PID_yaw_out,4,12);
	}
	else
	{
		OLED_ShowString(5,0,"System Info 6/6");
		OLED_ShowString(5,16,"Output of PWM");
		
		OLED_ShowString(5,32,"Up");
		OLED_ShowNum_withsymbol(30,32,TIM2->CCR3,4,12);
		
		OLED_ShowString(68,32,"Low");
		OLED_ShowNum_withsymbol(93,32,TIM2->CCR3,4,12);
				
		OLED_ShowString(5,48,"PSer");
		OLED_ShowNum_withsymbol(30,48,TIM3->CCR3,4,12);
		
		OLED_ShowString(68,48,"RSer");
		OLED_ShowNum_withsymbol(93,48,TIM3->CCR4,4,12);
	}
	//刷新
	OLED_Refresh_Gram();
}

void oledshow_AlterPIDValub(void)
{
	if(key1 != key1_pre&& key1==1)
	{
		key1_flag ++;
		OLED_Clear();
	}
	if (key1_flag >= 19)
	{
		key1_flag = 1;
	}
	key1_pre = key1;

	if (key1_flag == 1)
	{
		OLED_ShowString(5,0,"PID Param 1/18");
		OLED_ShowString(5,16, "In-P of Pitch");
		OLED_ShowFloat(10,32,pitchP_IN,3,12);
		PID_Tune(&pitchP_IN);
	}	
	else if (key1_flag == 2)
	{
		OLED_ShowString(5,0,"PID Param 2/18");
		OLED_ShowString(5,16, "In-I of Pitch");
		OLED_ShowFloat(10,32,pitchI_IN,3,12);
		PID_Tune(&pitchI_IN);
	}
	else if (key1_flag == 3)
	{
		OLED_ShowString(5,0,"PID Param 3/18");
		OLED_ShowString(5,16, "In-D of Pitch");
		OLED_ShowFloat(10,32,pitchD_IN,3,12);
		PID_Tune(&pitchD_IN);
	}
	else if (key1_flag == 4)
	{
		OLED_ShowString(5,0,"PID Param 4/18");
		OLED_ShowString(5,16, "Ex-P of Pitch");
		OLED_ShowFloat(10,32,pitchP_EXC,3,12);
		PID_Tune(&pitchP_EXC);
	}
	else if (key1_flag == 5)
	{
		OLED_ShowString(5,0,"PID Param 5/18");
		OLED_ShowString(5,16, "Ex-I of Pitch");
		OLED_ShowFloat(10,32,pitchI_EXC,3,12);
		PID_Tune(&pitchI_EXC);
	}
	else if (key1_flag == 6)
	{
		OLED_ShowString(5,0,"PID Param 6/18");
		OLED_ShowString(5,16, "Ex-D of Pitch");
		OLED_ShowFloat(10,32,pitchD_EXC,3,12);
		PID_Tune(&pitchD_EXC);
	}
	else if (key1_flag == 7)
	{
		OLED_ShowString(5,0,"PID Param 7/18");
		OLED_ShowString(5,16, "In-P of Roll");
		OLED_ShowFloat(10,32,rollP_IN,3,12);
		OLED_ShowString(5,48, " ");
		PID_Tune(&rollP_IN);
	}	
	else if (key1_flag == 8)
	{
		OLED_ShowString(5,0,"PID Param 8/18");
		OLED_ShowString(5,16, "In-I of Roll");
		OLED_ShowFloat(10,32,rollI_IN,3,12);
		PID_Tune(&rollI_IN);
	}
	else if (key1_flag == 9)
	{
		OLED_ShowString(5,0,"PID Param 9/18");
		OLED_ShowString(5,16, "In-D of Roll");
		OLED_ShowFloat(10,32,rollD_IN,3,12);
		PID_Tune(&rollD_IN);
	}
	else if (key1_flag == 10)
	{
		OLED_ShowString(5,0,"PID Param 10/18");
		OLED_ShowString(5,16, "Ex-P of Roll");
		OLED_ShowFloat(10,32,(rollP_EXC),3,12);
		PID_Tune(&rollP_EXC);
	}
	else if (key1_flag == 11)
	{
		OLED_ShowString(5,0,"PID Param 11/18");
		OLED_ShowString(5,16, "Ex-I of Roll");
		OLED_ShowFloat(10,32,(rollI_EXC),3,12);
		PID_Tune(&rollI_EXC);
	}
	else if (key1_flag == 12)
	{
		OLED_ShowString(5,0,"PID Param 12/18");
		OLED_ShowString(5,16, "Ex-D of Roll");
		OLED_ShowFloat(10,32,(rollD_EXC),3,12);
		PID_Tune(&rollD_EXC);
	}
	else if (key1_flag == 13)
	{
		OLED_ShowString(5,0,"PID Param 13/18");
		OLED_ShowString(5,16, "Ex-P of Yaw");
		OLED_ShowFloat(10,32,yawP_EXC,3,12);
		PID_Tune(&yawP_EXC);
	}	
	else if (key1_flag == 14)
	{
		OLED_ShowString(5,0,"PID Param 14/18");
		OLED_ShowString(5,16, "Ex-I of Yaw");
		OLED_ShowFloat(10,32,yawI_EXC,3,12);
		PID_Tune(&yawI_EXC);
	}
	else if (key1_flag == 15)
	{
		OLED_ShowString(5,0,"PID Param 15/18");
		OLED_ShowString(5,16, "Ex-D of Yaw");
		OLED_ShowFloat(10,32,yawD_EXC,3,12);
		PID_Tune(&yawD_EXC);
	}
	else if (key1_flag == 16)
	{
		OLED_ShowString(5,0,"PID Param 16/18");
		OLED_ShowString(5,16, "In-P of Yaw");
		OLED_ShowFloat(10,32,yawP_IN,3,12);
		PID_Tune(&yawP_IN);
	}
	else if (key1_flag == 17)
	{
		OLED_ShowString(5,0,"PID Param 17/18");
		OLED_ShowString(5,16, "In-I of Yaw");
		OLED_ShowFloat(10,32,yawI_IN,3,12);
		PID_Tune(&yawI_IN);
	}
	else
	{
		OLED_ShowString(5,0,"PID Param 18/18");
		OLED_ShowString(5,16, "In-D of Yaw");
		OLED_ShowFloat(10,32,yawD_IN,3,12);
		PID_Tune(&yawD_IN);
	}	
	//刷新
	OLED_Refresh_Gram();
}

void oledshow_AlterIMUOffset(void)
{
//	float P_angel_offsetFlag=0,R_angel_offsetFlag=0;
	
	if(key1 != key1_pre&& key1==1)
	{
		key1_flag ++;
		OLED_Clear();
	}
	if (key1_flag >= 5)
	{
		key1_flag = 1;
	}
	key1_pre = key1;

	if (key1_flag == 1)
	{
		OLED_ShowString(5,0,"IMU_Offset 1/4");
		OLED_ShowString(5,16, "Pitch Angel");
		PID_Tune(&pitch_angel_offset);
		OLED_ShowFloat(10,32,pitch_angel_offset,3,12);
	}	
	else if (key1_flag == 2)
	{
		OLED_ShowString(5,0,"IMU_Offset 2/4");
		OLED_ShowString(5,16, "Roll Angel");
		PID_Tune(&roll_angel_offset);
		OLED_ShowFloat(10,32,roll_angel_offset,3,12);
	}
	if (key1_flag == 3)
	{
		OLED_ShowString(5,0,"Servo_Offset 3/4");
		OLED_ShowString(5,16, "PB0 Pitch Val");
		if(roll > 600)
		{
			Pitch_ServoInitialVal = Pitch_ServoInitialVal - 1;
		}
		if(roll < 400)
		{
			Pitch_ServoInitialVal = Pitch_ServoInitialVal+ 1;
		}
		OLED_ShowFloat(10,32,Pitch_ServoInitialVal,3,12);
	}	
	if (key1_flag == 4)
	{
		OLED_ShowString(5,0,"Servo_Offset 4/4");
		OLED_ShowString(5,16, "PB1 Roll Val");
		if(roll > 600)
		{
			Roll_ServoInitialVal = Roll_ServoInitialVal - 1;
		}
		if(roll < 400)
		{
			Roll_ServoInitialVal = Roll_ServoInitialVal+ 1;
		}
		OLED_ShowFloat(10,32,Roll_ServoInitialVal,3,12);
	}
	//刷新
	OLED_Refresh_Gram();
}

