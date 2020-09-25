//对timer2、timer3定时器进行操作，利用PID的输出改变比较寄存器CCR的值，从而改变PWM输出高电平时间
#include "stm32f10x_tim.h"
#include "PWM_output.h"
//#include "myiic.h" 	//这个头文件包含了PAout(n)的定义与声明
/*
//版本：V1.0
//日期：20180527
//修改说明：
*/

volatile uint16_t CCR1_Val = 160;						/* 初始化输出比较通道1计数周期变量 */
volatile uint16_t CCR2_Val = 160;						/* 初始化输出比较通道2计数周期变量 */
volatile uint16_t CCR3_Val = 280;				   		/* 初始化输出比较通道3计数周期变量 */
volatile uint16_t CCR4_Val = 280;						/* 初始化输出比较通道4计数周期变量 */


//此为PWM的输出定时器端口
void timer1_init(void)
{
 	GPIO_InitTypeDef	GPIO_InitStructure;
	TIM_OCInitTypeDef	TIM_OCInitStructure;
	TIM_TimeBaseInitTypeDef	TIM_TimeBaseStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE);	/* 打开 TIM1 时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	TIM_DeInit(TIM1);
	
	//***在此设置PWM的输出引脚
	/* 设置 GPIOA 上的 TIM1 1,4通道对应引脚 PA.8,11为第二功能推挽输出 */
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_11;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* 设置 GPIOA 上的 TIM1 2N,3N通道对应引脚 PA.14,15为第二功能推挽输出 */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//在此设置PWM的输出参数
	/* 	设置timer1
	*  	计数重载值为2000
	*  	预分频值为720
	*  	时钟分割0
	*  	向上计数模式
	*		则产生的PWM信号频率为50Hz(20ms)
	*/
	TIM_TimeBaseStructure.TIM_Period = (2000-1);
	TIM_TimeBaseStructure.TIM_Prescaler = (720-1);	//Fruqency= 72M/(Period+1)*(Prescaler+1)
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);		
	
	/* 	设置timer1的 1,2N,3N,4 通道
	*  	工作模式为 PWM 输出模式
	*  	使能比较匹配输出极性
	*  	时钟分割0
	*  	向上计数模式
	*		初始值设置为150，对应1.5ms 
	*/
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	//设置定时器模式
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	//使能输出比较状态
//	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;	//使能互补输出状态
//	TIM_OCInitStructure.TIM_Pulse = (80-1);	//设置初始值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;	//设置极性
//	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;	
	
	
	TIM_OCInitStructure.TIM_Pulse = CCR3_Val;	
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	
	TIM_OCInitStructure.TIM_Pulse = CCR4_Val;	
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	
//	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
//	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
//	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
//	TIM_OC4Init(TIM1, &TIM_OCInitStructure);
	
	/* 使能预装载寄存器 */
//	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);	
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);	
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);	
//	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);	
	TIM_ARRPreloadConfig(TIM1, ENABLE);	
	
	/* 启动 TIM 计数 */
	TIM_CtrlPWMOutputs(TIM1,ENABLE);
	TIM_Cmd(TIM1, ENABLE);	
}


void timer2_init(void)
{
 	GPIO_InitTypeDef   GPIO_InitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE); /* 打开 TIM2 时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	/* 设置 GPIOA 上的 TIM2 1，2通道对应引脚 PA.0,PA.1为第二功能推挽输出 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* 	设置timer2
	*  	计数重载值为1000
	*  	预分频值为3
	*  	时钟分割0
	*  	向上计数模式
	*	则产生的PWM信号频率为24KHz，占空比为CCRx_Val/1000
	*/
	TIM_TimeBaseStructure.TIM_Period = (4000-1);
	TIM_TimeBaseStructure.TIM_Prescaler = (360-1);
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	
	TIM_TimeBaseInit(TIM2 , &TIM_TimeBaseStructure);		
	/* 	设置timer2的 OC1,OC2通道
	*  	工作模式为 PWM 输出模式
	*  	使能比较匹配输出极性
	*  	时钟分割0
	*  	向上计数模式
	*	设置各匹配值分别为 CCR1_Val, CCR2_Val
	*/
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OCInitStructure.TIM_Pulse = CCR1_Val;	
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
		
	TIM_OCInitStructure.TIM_Pulse = CCR2_Val;	
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);
	
	/* 使能预装载寄存器 */
	TIM_OC3PreloadConfig(TIM2 , TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM2 , TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM2, ENABLE);
		
	/* 启动 TIM 计数 */
	TIM_Cmd(TIM2 , ENABLE);		
}
void timer3_init(void)
{
 	GPIO_InitTypeDef   GPIO_InitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE); /* 打开 TIM3 时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
		TIM_DeInit(TIM3);
	
	/* 设置 GPIOA 上的 TIM3  3，4通道对应引脚 PB.0,PB.1为第二功能推挽输出 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* 	设置timer3
	*  	计数重载值为2000
	*  	预分频值为720
	*  	时钟分割0
	*  	向上计数模式
	*	则产生的PWM信号频率为50Hz，占空比为CCRx_Val/2000
	*/
	TIM_TimeBaseStructure.TIM_Period = (4000-1);
	TIM_TimeBaseStructure.TIM_Prescaler = (360-1);
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	
	TIM_TimeBaseInit(TIM3 , &TIM_TimeBaseStructure);		
	/* 	设置timer3的OC3,OC4 通道
	*  	工作模式为 PWM 输出模式
	*  	使能比较匹配输出极性
	*  	时钟分割0
	*  	向上计数模式
	*	设置各匹配值分别为 CCR3_Val, CCR4_Val
	*/
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
	TIM_OCInitStructure.TIM_Pulse = CCR3_Val;	
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
		
	TIM_OCInitStructure.TIM_Pulse = CCR4_Val;	
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);
	
	/* 使能预装载寄存器 */
	TIM_OC3PreloadConfig(TIM3 , TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM3 , TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM3, ENABLE);
		
	/* 启动 TIM 计数 */
	TIM_CtrlPWMOutputs(TIM3,ENABLE);
	TIM_Cmd(TIM3 , ENABLE);		
}

