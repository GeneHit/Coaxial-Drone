//��timer2��timer3��ʱ�����в���������PID������ı�ȽϼĴ���CCR��ֵ���Ӷ��ı�PWM����ߵ�ƽʱ��
#include "stm32f10x_tim.h"
#include "PWM_output.h"
//#include "myiic.h" 	//���ͷ�ļ�������PAout(n)�Ķ���������
/*
//�汾��V1.0
//���ڣ�20180527
//�޸�˵����
*/

volatile uint16_t CCR1_Val = 160;						/* ��ʼ������Ƚ�ͨ��1�������ڱ��� */
volatile uint16_t CCR2_Val = 160;						/* ��ʼ������Ƚ�ͨ��2�������ڱ��� */
volatile uint16_t CCR3_Val = 280;				   		/* ��ʼ������Ƚ�ͨ��3�������ڱ��� */
volatile uint16_t CCR4_Val = 280;						/* ��ʼ������Ƚ�ͨ��4�������ڱ��� */


//��ΪPWM�������ʱ���˿�
void timer1_init(void)
{
 	GPIO_InitTypeDef	GPIO_InitStructure;
	TIM_OCInitTypeDef	TIM_OCInitStructure;
	TIM_TimeBaseInitTypeDef	TIM_TimeBaseStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE);	/* �� TIM1 ʱ�� */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	TIM_DeInit(TIM1);
	
	//***�ڴ�����PWM���������
	/* ���� GPIOA �ϵ� TIM1 1,4ͨ����Ӧ���� PA.8,11Ϊ�ڶ������������ */
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_11;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* ���� GPIOA �ϵ� TIM1 2N,3Nͨ����Ӧ���� PA.14,15Ϊ�ڶ������������ */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//�ڴ�����PWM���������
	/* 	����timer1
	*  	��������ֵΪ2000
	*  	Ԥ��ƵֵΪ720
	*  	ʱ�ӷָ�0
	*  	���ϼ���ģʽ
	*		�������PWM�ź�Ƶ��Ϊ50Hz(20ms)
	*/
	TIM_TimeBaseStructure.TIM_Period = (2000-1);
	TIM_TimeBaseStructure.TIM_Prescaler = (720-1);	//Fruqency= 72M/(Period+1)*(Prescaler+1)
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);		
	
	/* 	����timer1�� 1,2N,3N,4 ͨ��
	*  	����ģʽΪ PWM ���ģʽ
	*  	ʹ�ܱȽ�ƥ���������
	*  	ʱ�ӷָ�0
	*  	���ϼ���ģʽ
	*		��ʼֵ����Ϊ150����Ӧ1.5ms 
	*/
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	//���ö�ʱ��ģʽ
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	//ʹ������Ƚ�״̬
//	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;	//ʹ�ܻ������״̬
//	TIM_OCInitStructure.TIM_Pulse = (80-1);	//���ó�ʼֵ
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;	//���ü���
//	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;	
	
	
	TIM_OCInitStructure.TIM_Pulse = CCR3_Val;	
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	
	TIM_OCInitStructure.TIM_Pulse = CCR4_Val;	
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	
//	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
//	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
//	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
//	TIM_OC4Init(TIM1, &TIM_OCInitStructure);
	
	/* ʹ��Ԥװ�ؼĴ��� */
//	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);	
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);	
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);	
//	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);	
	TIM_ARRPreloadConfig(TIM1, ENABLE);	
	
	/* ���� TIM ���� */
	TIM_CtrlPWMOutputs(TIM1,ENABLE);
	TIM_Cmd(TIM1, ENABLE);	
}


void timer2_init(void)
{
 	GPIO_InitTypeDef   GPIO_InitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE); /* �� TIM2 ʱ�� */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	/* ���� GPIOA �ϵ� TIM2 1��2ͨ����Ӧ���� PA.0,PA.1Ϊ�ڶ������������ */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* 	����timer2
	*  	��������ֵΪ1000
	*  	Ԥ��ƵֵΪ3
	*  	ʱ�ӷָ�0
	*  	���ϼ���ģʽ
	*	�������PWM�ź�Ƶ��Ϊ24KHz��ռ�ձ�ΪCCRx_Val/1000
	*/
	TIM_TimeBaseStructure.TIM_Period = (4000-1);
	TIM_TimeBaseStructure.TIM_Prescaler = (360-1);
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	
	TIM_TimeBaseInit(TIM2 , &TIM_TimeBaseStructure);		
	/* 	����timer2�� OC1,OC2ͨ��
	*  	����ģʽΪ PWM ���ģʽ
	*  	ʹ�ܱȽ�ƥ���������
	*  	ʱ�ӷָ�0
	*  	���ϼ���ģʽ
	*	���ø�ƥ��ֵ�ֱ�Ϊ CCR1_Val, CCR2_Val
	*/
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OCInitStructure.TIM_Pulse = CCR1_Val;	
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
		
	TIM_OCInitStructure.TIM_Pulse = CCR2_Val;	
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);
	
	/* ʹ��Ԥװ�ؼĴ��� */
	TIM_OC3PreloadConfig(TIM2 , TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM2 , TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM2, ENABLE);
		
	/* ���� TIM ���� */
	TIM_Cmd(TIM2 , ENABLE);		
}
void timer3_init(void)
{
 	GPIO_InitTypeDef   GPIO_InitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE); /* �� TIM3 ʱ�� */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
		TIM_DeInit(TIM3);
	
	/* ���� GPIOA �ϵ� TIM3  3��4ͨ����Ӧ���� PB.0,PB.1Ϊ�ڶ������������ */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* 	����timer3
	*  	��������ֵΪ2000
	*  	Ԥ��ƵֵΪ720
	*  	ʱ�ӷָ�0
	*  	���ϼ���ģʽ
	*	�������PWM�ź�Ƶ��Ϊ50Hz��ռ�ձ�ΪCCRx_Val/2000
	*/
	TIM_TimeBaseStructure.TIM_Period = (4000-1);
	TIM_TimeBaseStructure.TIM_Prescaler = (360-1);
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	
	TIM_TimeBaseInit(TIM3 , &TIM_TimeBaseStructure);		
	/* 	����timer3��OC3,OC4 ͨ��
	*  	����ģʽΪ PWM ���ģʽ
	*  	ʹ�ܱȽ�ƥ���������
	*  	ʱ�ӷָ�0
	*  	���ϼ���ģʽ
	*	���ø�ƥ��ֵ�ֱ�Ϊ CCR3_Val, CCR4_Val
	*/
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
	TIM_OCInitStructure.TIM_Pulse = CCR3_Val;	
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
		
	TIM_OCInitStructure.TIM_Pulse = CCR4_Val;	
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);
	
	/* ʹ��Ԥװ�ؼĴ��� */
	TIM_OC3PreloadConfig(TIM3 , TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM3 , TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM3, ENABLE);
		
	/* ���� TIM ���� */
	TIM_CtrlPWMOutputs(TIM3,ENABLE);
	TIM_Cmd(TIM3 , ENABLE);		
}

