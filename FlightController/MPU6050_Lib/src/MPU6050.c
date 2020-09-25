#include "mpu6050.h"
#include "myiic.h"
#include "inv_mpu.h" //st.chip_cfg.sample_rate & DEFAULT_MPU_HZ�����ڴ˴��������趨
#include "inv_mpu_dmp_motion_driver.h"
#include "usart.h"
#include "math.h"
#include "sysconfig.h"
#include "led.h"
#include "delay.h"

/*
//�汾��V1.0
//���ڣ�20180527
//�޸�˵����
*/

//���岻ͬ������Χ�Ŀ̶�����
#define Gyro_250_Scale_Factor   131.0f
#define Gyro_500_Scale_Factor   65.5f
#define Gyro_1000_Scale_Factor  32.8f
#define Gyro_2000_Scale_Factor  16.4f
#define Accel_2_Scale_Factor    16384.0f
#define Accel_4_Scale_Factor    8192.0f
#define Accel_8_Scale_Factor    4096.0f
#define Accel_16_Scale_Factor   2048.0f
#define DEFAULT_MPU_HZ 200

//����������������Ҫ��ǰ��ã����ã�
//����ƫ���С���ֵ����ƫ
#define roll_ang_off_ByCoaxial  1.4
//ǰ���飬�Ӵ������ֵ����ǰƫ
#define pitch_ang_off_ByCoaxial  0.3//��ʼʱ�̶�ƫ��

//����ң���ϵ����׵�λ����APP�ϵĵ��Ծ�ƫ�Ĺ��������Ծ�ƫ�ı���
float roll_angel_offset = 0;
float pitch_angel_offset = 0;
static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};//roll��pitch���Ƿ��ģ�yaw������
float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
float Pitch,Roll,Yaw;
extern float mpitch,mroll;//for LED
float Pitch_Pre,Roll_Pre,Yaw_Pre;
float W_Roll, W_Pitch, W_Yaw;	//���ٶ�										   
float W_Roll_Pre, W_Pitch_Pre, W_Yaw_Pre;
float Accel_Filter[5];
volatile float Accel_High_Now;
unsigned long sensor_timestamp;
short gyro[3], accel[3], sensors;
unsigned char more;
long quat[4];
float W_Roll_Filter[5],W_Pitch_Filter[5],W_Yaw_Filter[5];
float ACC0_Filter[8],ACC1_Filter[8],ACC2_Filter[8];
W_AND_ANGLE w_and_angle;

volatile float Velocity_X,Velocity_Y,Velocity_Z;																				
volatile float Distance_X,Distance_Y,Distance_Z;																					 
//extern int magX, magY, magZ;	//hmc��ԭʼ����
const float Accel_Ground_Diff = 16734.000000;
const float Accel_X_Diff = 30.000000;	
const float Accel_Y_Diff = 132.000000;																						 
int8_t MPU6050_First = -1;

//float magOffset[3] = { (MAG0MAX + MAG0MIN) / 2, (MAG1MAX + MAG1MIN) / 2, (MAG2MAX + MAG2MIN) / 2 };
static  unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static  unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

static void run_self_test(void)
{
	
    int result;
    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);
    if (result == 0x7) {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
		//printf("setting bias succesfully ......\r\n");
    }
	else
	{
		//printf("bias has not been modified ......\r\n");
	}

}
//���Ƕ���6050�����ݵĶ�ȡ���õ��жϵ���ʽ
void MPU6050_Interrupt_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource4);
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line4;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	EXTI_ClearITPendingBit(EXTI_Line4);
}
//����MPU6050�ĳ�ʼ��
void MPU6050_Init(void)
{
	int result=1;
	while(result)
	{
		result=mpu_init(); //��ʼ��MPU6050
		//st.chip_cfg.sample_rate�����ڴ˴��������趨
		//st.chip_cfg�ṹ���ڴ˴������˳�ʼ��
		
	}

	if(result == 0)
	{
	}
	
	if(!result)
	{	 		 
	
		PrintChar("mpu initialization complete......\n ");		//mpu initialization complete	 	  

		if(!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))		//mpu_set_sensor
			PrintChar("mpu_set_sensor complete ......\n");
		else
			PrintChar("mpu_set_sensor come across error ......\n");

		if(!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))	//mpu_configure_fifo
			PrintChar("mpu_configure_fifo complete ......\n");
		else
			PrintChar("mpu_configure_fifo come across error ......\n");

		if(!mpu_set_sample_rate(DEFAULT_MPU_HZ))	   	  		//mpu_set_sample_rate
		//st.chip_cfg.sample_rate�����ڴ˴������˸ı䡣
		 PrintChar("mpu_set_sample_rate complete ......\n");
		else
		 	PrintChar("mpu_set_sample_rate error ......\n");

		if(!dmp_load_motion_driver_firmware())   	  			//dmp_load_motion_driver_firmvare
			PrintChar("dmp_load_motion_driver_firmware complete ......\n");
		else
			PrintChar("dmp_load_motion_driver_firmware come across error ......\n");

		if(!dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation))) 	  //dmp_set_orientation
		 	PrintChar("dmp_set_orientation complete ......\n");
		else
		 	PrintChar("dmp_set_orientation come across error ......\n");

		if(!dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
		    DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
		    DMP_FEATURE_GYRO_CAL))		   	 					 //dmp_enable_feature
		 	PrintChar("dmp_enable_feature complete ......\n");
		else
		 	PrintChar("dmp_enable_feature come across error ......\n");

		if(!dmp_set_fifo_rate(DEFAULT_MPU_HZ))   	 			 //dmp_set_fifo_rate
		 	PrintChar("dmp_set_fifo_rate complete ......\n");
		else
		 	PrintChar("dmp_set_fifo_rate come across error ......\n");

		run_self_test();		//�Լ�

		if(!mpu_set_dmp_state(1))
		 	PrintChar("mpu_set_dmp_state complete ......\n");
		else
		 	PrintChar("mpu_set_dmp_state come across error ......\n");
	}
}
//�˺������Ƕ�ȡ6050���ݵĲ�������
void MPU6050_Pose(void)
{
//	float temp;
	float w_SUM;
	uint8_t i;
	double Gyrox,Gyroy,Gyroz;	
	
	dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);	 
	/* Gyro and accel data are written to the FIFO by the DMP in chip frame and hardware units.
	 * This behavior is convenient because it keeps the gyro and accel outputs of dmp_read_fifo and mpu_read_fifo consistent.
	**/
	/*if (sensors & INV_XYZ_GYRO )
	send_packet(PACKET_TYPE_GYRO, gyro);
	if (sensors & INV_XYZ_ACCEL)
	send_packet(PACKET_TYPE_ACCEL, accel); */
	/* Unlike gyro and accel, quaternions are written to the FIFO in the body frame, q30.
	 * The orientation is set by the scalar passed to dmp_set_orientation during initialization. 
	**/
	if(sensors & INV_WXYZ_QUAT )
	{
	    //�淶����ȷ�𣿣�������
		q0 = (float)quat[0] / q30;	
		q1 = (float)quat[1] / q30;
		q2 = (float)quat[2] / q30;
		q3 = (float)quat[3] / q30;
    /*
		��20150329�о��˴��Ĵ����������⣬Ҳ����asin atan2������
		��ʾ�����鿴����ʱ���ֽ��ٶ�û�����⣬���Ƕ�������
		��ȡ��ʵ�鷽���ǵ�������PID�����ã�Ҳ����������P I D �ֱ𵥶����ã�
		�ٲ鿴PWM���ŵĲ���ͼ��������45�Ⱥ�60��λ����һ��ͻ�䣬Ҳ���ǲ���
		��ռ�ձ���ԭ����ռ�ձ�һ����Խ�䵽10%���Ҳ��������ڽǶ����ݵ�ͻ����ɵģ�
		һ�����ƣ�Ӧ���Ǵ˴������������Ǻ����������⣺
		1.�������ƽǶ�������ͻ��
		2.���ò��Һ����������д���
		*/
		//if(fabs(2 * q1 * q3 - 2 * q0* q2)<=1)
		Pitch = asin(2 * q1 * q3 - 2 * q0* q2)* RAD_TO_DEG;	// pitch,��λ����
		if(fabs(Pitch)>90)
			Pitch = Pitch_Pre;
		else
			Pitch_Pre = Pitch;
		
		Roll  = -atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* RAD_TO_DEG;	// roll
		if(fabs(Roll)>90)
			Roll = Roll_Pre;
		else
			Roll_Pre = Roll;
		
		Yaw   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * RAD_TO_DEG;	//yaw	,��λ����
		if(fabs(Yaw)>180)
			Yaw = Yaw_Pre;
		else
			Yaw_Pre = Yaw;
		
	  w_and_angle.angle_roll = Roll+roll_angel_offset+(float)roll_ang_off_ByCoaxial; 		//����Ƕ�
		w_and_angle.angle_pitch = Pitch+pitch_angel_offset+(float)pitch_ang_off_ByCoaxial;	//�����Ƕ�
		w_and_angle.angle_yaw = Yaw + 180; // ת��Ϊ0~360��	//ƫ���Ƕ�
		//W_Roll_Filter[5],W_Pitch_Filter[5],W_Yaw_Filter[5];
		Gyrox = gyro[0]/(double)Gyro_2000_Scale_Factor;
		Gyroy = gyro[1]/(double)Gyro_2000_Scale_Factor;
		Gyroz = gyro[2]/(double)Gyro_2000_Scale_Factor;
		
/*
�˴����ڽ��ٶȺͼ��ٶȵ����ݽ����˻���ƽ��ֵ�˲�����
���ǽ�����˲�������ģͣ����ϣ��ﵽ�������յ�Ҫ��
Ҳ��˵��������Ľ���ʱ�����ˣģͣУ�����ƽ��ֵ�˲��ķ���
��ȵ�����DMP�͵�����Kalman�ں��˲������޷�������ŵ�
*/
		//����5��ֵ�Ļ����˲�����
		w_SUM = 0;
		for(i=4;i>0;i--)
		{
			W_Roll_Filter[i] = W_Roll_Filter[i-1];
			w_SUM += W_Roll_Filter[i];
		}
		W_Roll_Filter[0]	= Gyrox;//gyro[0]/Gyro_2000_Scale_Factor;
		w_SUM += W_Roll_Filter[0];
		W_Roll  = w_SUM/5.0f;
		
		w_SUM = 0;
		for(i=4;i>0;i--)
		{
			W_Pitch_Filter[i] = W_Pitch_Filter[i-1];
			w_SUM += W_Pitch_Filter[i];
		}
		W_Pitch_Filter[0]	= Gyroy;//gyro[1]/Gyro_2000_Scale_Factor;
		w_SUM += W_Pitch_Filter[0];
		W_Pitch  = w_SUM/5.0f;
		
		w_SUM = 0;
		for(i=4;i>0;i--)
		{
			W_Yaw_Filter[i] = W_Yaw_Filter[i-1];
			w_SUM += W_Yaw_Filter[i];
		}
		W_Yaw_Filter[0]	= Gyroz;//gyro[2]/Gyro_2000_Scale_Factor;
		w_SUM += W_Yaw_Filter[0];
		W_Yaw  = w_SUM/5.0f;
		
		
		
		//���ٶȽ���8��ֵ�Ļ����˲�����
		w_SUM = 0;
		for(i=7;i>0;i--)
		{
			ACC0_Filter[i] = ACC0_Filter[i-1];
			w_SUM += ACC0_Filter[i];
		}
		ACC0_Filter[0]	= accel[0];//gyro[0]/Gyro_2000_Scale_Factor;
		w_SUM += ACC0_Filter[0];
		accel[0]  = w_SUM/8.0f;
		
		w_SUM = 0;
		for(i=7;i>0;i--)
		{
			ACC1_Filter[i] = ACC1_Filter[i-1];
			w_SUM += ACC1_Filter[i];
		}
		ACC1_Filter[0]	= accel[1];//gyro[0]/Gyro_2000_Scale_Factor;
		w_SUM += ACC1_Filter[0];
		accel[1]  = w_SUM/8.0f;
		
		w_SUM = 0;
		for(i=7;i>0;i--)
		{
			ACC2_Filter[i] = ACC2_Filter[i-1];
			w_SUM += ACC2_Filter[i];
		}
		ACC2_Filter[0]	= accel[2];//gyro[0]/Gyro_2000_Scale_Factor;
		w_SUM += ACC2_Filter[0];
		accel[2]  = w_SUM/8.0f;
		
														//��ʱת��Ϊ��ÿ��
		if(fabs(W_Roll)>1000)
			W_Roll = W_Roll_Pre;
		else
			W_Roll_Pre = W_Roll;
		
		if(fabs(W_Pitch)>1000)
			W_Pitch = W_Pitch_Pre;
		else
			W_Pitch_Pre = W_Pitch;
		
		if(fabs(W_Yaw)>1000)
			W_Yaw = W_Yaw_Pre;
		else
			W_Yaw_Pre = W_Yaw;
		
		w_and_angle.w_roll = W_Roll;  	//������ٶ�
		w_and_angle.w_pitch = W_Pitch;	//�������ٶ�
		w_and_angle.w_yaw = W_Yaw;			//ƫ�����ٶ�
		
		//�˴��ǶԼ��ٶȽ��е��˲�����
		accel[0] -= Accel_X_Diff;
		accel[1] -= Accel_Y_Diff;
		}	
}

