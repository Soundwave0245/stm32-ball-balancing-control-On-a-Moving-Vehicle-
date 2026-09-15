#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Timer.h"
#include "Motor.h"
#include "PID.h"
#include "Tracker.h"
#include "Serial.h"  //HC05
#include "Kalman.h"
#include "Key.h"

uint16_t Time = 0;
uint8_t KeyNum;
int16_t Speed1;
int16_t Speed2;
int16_t SpeedUp;
uint16_t BaseSpeed = 40;        //一些PID参数

                               //PID结构体

TrackerStruct Tracker;        //循迹模块结构体

extern uint16_t Num_of_Cross;



extern uint8_t Serial_TxPacket[];    //串口发送数据，大小在Serial.c里修改
extern uint8_t Serial_RxPacket[];    //串口接收数据，大小在Serial.c里修改，记得改状态机里面对应的接收数据个数

/*卡尔曼滤波器定义*/
#define CHANNEL_NUM 4                   //需要处理几个数据，这里的设计是用KalmanFilter对串口接收到的数组滤波，串口一次进来四个数，CHANNEL_NUM给4

Kalman1D_t kalman_filter[CHANNEL_NUM];    //卡尔曼滤波器结构体定义

float filtered_float[CHANNEL_NUM];  //计算出来的数据是浮点数，初步用这个存储

uint16_t filtered_int[CHANNEL_NUM];  // 数据都是整数，取整后输出



/*————————————————————Utils各种滤波器初始化驱动配置函数定义————————————————————————*/
	
	
/*————————————————————1. 卡尔曼滤波器初始化驱动——————————————————————*/
//卡尔曼滤波器赋初值和初始化
void KalmanFilter_Init(void)
{
	float init_x = 0.0f;   // 初始估计值，需要根据实际情况调整！！！
	float init_P = 100.0f;     // 初始协方差，表示对初始值的不确定度
	float Q = 0.1f;           // 过程噪声，值越小滤波越平滑，响应越慢
	float R = 10.0f;          // 测量噪声，值越大对测量值信任越低（噪声大时增大）
	uint8_t max_invalid = 5;  // 连续5次无效则复位                                             //这些参数可能需要调整！！！
	

	for (int i = 0; i < CHANNEL_NUM; i++) 
	{
		Kalman1D_Init(&kalman_filter[i], init_x, init_P, Q, R, max_invalid);
		filtered_float[i] = init_x;
		/*将每个通道的 filtered_values[i] 设为init_x，是为了给数组一个确定的初始值，避免未初始化导致随机值，同时与卡尔曼滤波器给值一致，对齐*/
		filtered_int[i] = (uint16_t)(init_x + 0.5f); // 取整, +0.5f给小数点后的数据四舍五入
	}

}

// 在接收中断/主循环中使用
void KalmanFilter_ReceivedDataProcess(void)
{
    if (Serial_GetRxFlag()) 
	{
        for (int i = 0; i < CHANNEL_NUM; i++) 
		{
            uint16_t raw = Serial_RxPacket[i];
            // 有效范围 0~1000（根据你的实际量程调整）
            filtered_float[i] = Kalman1D_UpdateWithCheck(&kalman_filter[i], (float)raw, 0.0f, 1000.0f);
            // 转为整型（四舍五入）
            filtered_int[i] = (uint16_t)(filtered_float[i] + 0.5f);
        }
       
    }
}




/*————————————————————模式化编程——————————————————————————*/

uint16_t CurrMode = 0;
uint16_t NextMode = 0;



/*————————————————————模式参数——————————————————————————*/
uint8_t Mode1_Running = 0;
uint8_t Mode2_Running = 0;

/*___________________________Mode 0____________________________*/
void Mode0_Init(void)
{
	Tracker.TrackMode = 0;
	OLED_Clear();
	OLED_ShowString(1,1,"[Mode0]");
	Num_of_Cross = 0;
	
}
void Mode0_Loop(void)
{
	
}
void Mode0_Exit(void)
{
	Tracker.TrackMode = 0;
	Num_of_Cross = 0;
}



/*_________________________Mode 1_________________________________*/
void Mode1_Init(void)
{
	
	OLED_Clear();
	OLED_ShowString(1,1,"[Mode1]");
	OLED_ShowString(2,1,"K2>>");
	
	Tracker.TrackMode = 0;  //别让他跑了
	Num_of_Cross = 0;
}



void Mode1_Loop(void)
{
	if(KeyNum == 2 && Mode1_Running == 0)
	{
		OLED_ShowString(1,1,"                    ");
		OLED_ShowString(1,1,"Time:   .");
		OLED_ShowString(2,1,"Running...");
		
		Tracker.TrackMode = 1;
		Time = 0;
		
		Mode1_Running = 1;
	}
	
	
	
	
	if(Mode1_Running)
	{
		
		Signal_Handler(&Tracker);
			
		OLED_ShowNum(1,6,(Time / 1000),3);
		OLED_ShowNum(1,10,(Time % 1000),3);
		
		OLED_ShowNum(3,1,Num_of_Cross,3);
	}
}



void Mode1_Exit(void)
{
	Mode1_Running = 0;
	Tracker.TrackMode = 0;  //别让他跑了
	Num_of_Cross = 0;
}



/*模式2缓速启动专用加速参数*/
int Acc = 40 / 6;                                          //60是BaseSpeed的原来的数字，5秒加速完成

uint16_t Time_for_Acc = 0;
uint16_t Time_Acc_Flag = 0;

/*________________________Mode 2______________________________________*/
void Mode2_Init(void)
{
	Time_for_Acc = 0;
	Time_Acc_Flag = 0;
	
	Mode2_Running = 0;
	Tracker.TrackMode = 0;  //别让他跑了
	Num_of_Cross = 0;
	BaseSpeed = 0;
	
	OLED_Clear();
	OLED_ShowString(1,1,"[Mode2]");
	OLED_ShowString(2,1,"K2>>");
	
}


void Mode2_Loop(void)
{
	if(KeyNum == 2 && Mode2_Running == 0)
	{
		OLED_ShowString(1,1,"                    ");
		OLED_ShowString(1,1,"Time:   .");
		OLED_ShowString(2,1,"Running...");
		
		Tracker.TrackMode = 1;
		Time = 0;
		
		Mode2_Running = 1;
	}
	
	if(Mode2_Running)
	{
		BaseSpeed = (float)Time_for_Acc / 1000 * Acc;
		
		Signal_Handler(&Tracker);
			
		OLED_ShowNum(1,6,(Time / 1000),3);
		OLED_ShowNum(1,10,(Time % 1000),3);
		
		OLED_ShowNum(3,1,Num_of_Cross,3);
	}
}


void Mode2_Exit(void)
{
	Mode2_Running = 0;
	Tracker.TrackMode = 0;  //别让他跑了
	Num_of_Cross = 0;
	BaseSpeed = 40;             //改回来免得影响Mode1或者其他功能的演示
	
	Time_for_Acc = 0;
	Time_Acc_Flag = 0;
}








	
int main (void)
{
	/*——————————————————————硬件单元初始化————————————————————————*/
	OLED_Init();     //OLED初始化
	OLED_ShowString(1,1,"A");  // 先确认OLED可用
	Delay_ms(100);
	Motor_Init();    //电机初始化
	

	
	Tracker_Init();  //循迹模块初始化
	OLED_ShowString(1,1,"B");  
	Delay_ms(100);
	
	Timer_Init();	 //1ms定时器初始化
	OLED_ShowString(1,1,"C");  
	Delay_ms(100);
	
	OLED_ShowString(1,1,"D");  
	Delay_ms(100);
	
	KEY_Init();
	OLED_ShowString(1,1,"E");  
	Delay_ms(100);
	
	OLED_ShowString(1,1,"OK");
	Delay_ms(500);               // 短暂停留，让您看到OK
	Mode0_Init();
	/*————————————————————Utils各种滤波器初始化————————————————————————*/
//	KalmanFilter_Init();
	
	/*——————————————————主函数正式开始——————————————————————*/
	
	Tracker.TrackMode = 0;
	
//	OLED_ShowString(1,1,"Speed1:");
//	OLED_ShowString(2,1,"Speed2:");
//	OLED_ShowString(3,1,"Tdata:  TDev:");
//	OLED_ShowString(4,1,"TMode: ");
	
	CurrMode = 0;
	
	while(1)
		{
			KeyNum = Key_GetNum();
			
			if(KeyNum == 1)
			{
				NextMode++;
				
				if(NextMode > 2)
				{
					NextMode = 0;
				}
			}
			
			if(CurrMode == NextMode)
			{
				switch(CurrMode)
				{
					case 0: Mode0_Loop(); break;
					
					case 1: Mode1_Loop(); break;
					
					case 2: Mode2_Loop(); break;
				}
			}
			
			else
			{
				switch(CurrMode)
				{
					case 0: Mode0_Exit(); break;
					
					case 1: Mode1_Exit(); break;
					
					case 2: Mode2_Exit(); break;
				}
				
				switch(NextMode)
				{
					case 0: Mode0_Init(); break;
					
					case 1: Mode1_Init(); break;
					
					case 2: Mode2_Init(); break;
				}
				
				CurrMode = NextMode;
			}
			
			
//			Signal_Handler(&Tracker);
//			
//			OLED_ShowNum(1,6,(Time / 1000),3);
//			OLED_ShowNum(1,10,(Time % 1000),3);
//			
//			OLED_ShowNum(2,1,Num_of_Cross,3);
			
			
//			OLED_ShowSignedNum(1,8,Speed1,3);
//			OLED_ShowSignedNum(2,8,Speed2,3);
//			OLED_ShowSignedNum(3,7,Tracker.TrackData,2);
//			OLED_ShowSignedNum(3,14,Tracker.TrackCurrentDev,2);
//			OLED_ShowSignedNum(4,8,Tracker.TrackMode,1);
		}
}



void TIM4_IRQHandler(void)  //TIM4定时中断，累计40次1ms中断更新一次状态值
{
	static uint16_t Count;
	
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
		
		if(CurrMode != 0)
		{
			Count++;
			if(Tracker.TrackMode != 0)
			{	
				Time ++;
			}
			if(Num_of_Cross)
			{
				uint8_t is_protected = 0;
                if(CurrMode == 1 && Mode1_Running && Time < 6000) is_protected = 1;
                if(CurrMode == 2 && Mode2_Running && Time < 8000) is_protected = 1;
                if(!is_protected)   // 非保护期才停车
                {
                    Tracker.TrackMode = 0;
                }
			}
			
			if(Count >= 40)
			{
				
				if (Tracker.TrackData == 0x00) 
				{   // 完全脱线
					SpeedUp = 0;
				} 

				if(Tracker.TrackMode == 0)
				{
					Motor_Stop();
				}
				else if(Tracker.TrackMode == 1)
				{
					SpeedUp = Dev_SpeedPIDUpdate(Tracker.TrackCurrentDev);//转速差，这里面有一个PID控制器，可以过去调参
					
					Speed1 = BaseSpeed + SpeedUp;    //如果出现越左越左、越右越右，把这两个SpeedUp前面的正负号对调！
					Motor1_Setspeed(Speed1);
					Speed2 = BaseSpeed - SpeedUp;
					Motor2_Setspeed(Speed2);	
				}
				else if(Tracker.TrackMode == 2)
				{
					Motor_Stop();
					Motor_Left();
				}
				else if(Tracker.TrackMode == 3)
				{
					Motor_Stop();
					Motor_Right();
				}
				
				Count = 0;
			
			}
			
			if(CurrMode == 2 && Mode2_Running && Time_Acc_Flag == 0)
			{
				Time_for_Acc++;
				if(Time_for_Acc >= 6000)
				{
					Time_for_Acc = 6000;
					
					Time_Acc_Flag = 1;
				}
				
			}
			
			if(CurrMode == 2 && Mode2_Running && Time >= 12000 && Time_Acc_Flag == 1)
			{
				Time_for_Acc--;
				if(Time_for_Acc <= 2500)
				{
					Time_for_Acc = 2500;
					
					Time_Acc_Flag = 2;
				}
			}
			
			if(CurrMode == 2 && Mode2_Running && Time >= 22500 && Time_Acc_Flag == 2)
			{
				Time_for_Acc--;
				if(Time_for_Acc <= 1500)
				{
					Time_for_Acc = 1500;
				}
			}
			
//			TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
			
		}
			
		
	
	}
}




