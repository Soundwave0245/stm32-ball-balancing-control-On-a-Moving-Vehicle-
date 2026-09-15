#include "stm32f10x.h"                  // Device header
#include "PWM.h"

void Motor_Init()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	PWM_Init();
	
}

void Motor1_Setspeed(int16_t Speed) //左电机
{
	if(Speed >= 0)
	{
		GPIO_SetBits(GPIOA,GPIO_Pin_4);
		GPIO_ResetBits(GPIOA,GPIO_Pin_5);
		PWM2_SetCompare3(Speed);
	}
	else
	{
		GPIO_SetBits(GPIOA,GPIO_Pin_5);
		GPIO_ResetBits(GPIOA,GPIO_Pin_4);
		PWM2_SetCompare3(-Speed);
	}
}

void Motor2_Setspeed(int16_t Speed)  //右电机
{
	if(Speed >= 0)
	{
		GPIO_SetBits(GPIOA,GPIO_Pin_8);
		GPIO_ResetBits(GPIOA,GPIO_Pin_11);
		PWM2_SetCompare2(Speed);
	}
	else
	{
		GPIO_SetBits(GPIOA,GPIO_Pin_11);
		GPIO_ResetBits(GPIOA,GPIO_Pin_8);
		PWM2_SetCompare2(-Speed);
	}
}

void Motor_Stop(void)
{
	Motor1_Setspeed(0);
	Motor2_Setspeed(0);
}

void Motor_Left(void)
{
	Motor1_Setspeed(-60);
	Motor2_Setspeed(60);
}

void Motor_Right(void)
{
	Motor1_Setspeed(60);
	Motor2_Setspeed(-60);
}
