#include "stm32f10x.h"                  // Device header
#include "Tracker.h"
#include "Delay.h"

#define Tracker_Port GPIOB
#define Tracker_L1_Pin GPIO_Pin_0
#define Tracker_L0_Pin GPIO_Pin_1
#define Tracker_M_Pin GPIO_Pin_12
#define Tracker_R0_Pin GPIO_Pin_13
#define Tracker_R1_Pin GPIO_Pin_14

//从左到右分别是L1 L0 M R0 R1

uint16_t Num_of_Cross = 0;

void Tracker_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = Tracker_L1_Pin | Tracker_L0_Pin | Tracker_M_Pin | Tracker_R0_Pin | Tracker_R1_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(Tracker_Port,&GPIO_InitStructure);
	
}

void Tracker_ReadData(TrackerStruct *Tracker)//五路巡线传感器读取线路信息
{
	if(GPIO_ReadInputDataBit(Tracker_Port,Tracker_L1_Pin))
	{
		Tracker->TrackData |= 0x20;   //读取到黑线时该位置1
	}
	else
	{
		Tracker->TrackData &= ~0x20;   //没有黑线时该位置0
	}
	
	if(GPIO_ReadInputDataBit(Tracker_Port,Tracker_L0_Pin))
	{
		Tracker->TrackData |= 0x10;
	}
	else
	{
		Tracker->TrackData &= ~0x10;
	}
	
	if(GPIO_ReadInputDataBit(Tracker_Port,Tracker_M_Pin))
	{
		Tracker->TrackData |= 0x08;
	}
	else
	{
		Tracker->TrackData &= ~0x08;
	}
	
	if(GPIO_ReadInputDataBit(Tracker_Port,Tracker_R0_Pin))
	{
		Tracker->TrackData |= 0x04;
	}
	else
	{
		Tracker->TrackData &= ~0x04;
	}
	
	if(GPIO_ReadInputDataBit(Tracker_Port,Tracker_R1_Pin))
	{
		Tracker->TrackData |= 0x02;
	}
	else
	{
		Tracker->TrackData &= ~0x02;
	}
}



void Signal_Handler(TrackerStruct *Tracker)  //信号处理，处理传感器得到的信号，分析偏差值给Dev_PID，Dev_PID返回需要的差速Speed值，这个值的大小可以通过调参调节
{
	static uint8_t OverRun_Time = 0;  //允许小车脱离线的时间，
	
	Tracker_ReadData(Tracker);
	
	// ========== 新增：十字路口多种组合检测 ==========
    uint16_t data = Tracker->TrackData;
    // 列出所有触发十字路口计数的组合（按位：L1=0x20, L0=0x10, M=0x08, R0=0x04, R1=0x02）
    // 原有: R0ML0 -> 0x1C (L0,M,R0)
    // 新增: R1L0 -> 0x12 (L0,R1)
    //       R0L1 -> 0x24 (L1,R0)
    //       L1R1 -> 0x22 (L1,R1)
    //       以及原来的 0x3e (L0,M,R0,R1)
    if (data == 0x3e || data == 0x1C || data == 0x12 || data == 0x24 || data == 0x22)
    {
        Num_of_Cross++;
        // 检测到十字路口，直接返回，不进行后续的偏差计算和转弯处理
        return;
    }
    // =============================================
    
	
	
    // 以下为原有逻辑（已移除 case 0x3e，因为已被上面处理）
	if(Tracker->TrackData != 0x00) //检测到线，脱线时长清零
	{
		OverRun_Time = 0;
	}
	
	if(Tracker->TrackData &0x22 && Tracker->TrackData & 0x08)//直角、锐角标记，0x22（00100010）用于过滤中间的三个传感器状态
			/*Tracker->TrackData & 0x08是为了保证只有中间也接检测到黑线的时候才会记录TrackLastData，防止手遮挡导致原地打转*/
	{
		Tracker->TrackLastData = Tracker->TrackData;
	}	

	switch(Tracker->TrackData)
	{
		case 0x00:     //脱离线路，可能是直角锐角也可能是循迹失败
			if(Tracker->TrackLastData & 0x20)//0x20:刚刚左侧直角锐角
			{
				uint16_t Count1 = 0;
				Tracker->TrackMode = LEFT;//右电机正转左电机反转
				while(1)                  //直到循迹模块再次检测到线
				{
					Count1++;
					if(Count1 >= 60000)
					{
						Tracker->TrackMode = STOP;
						Tracker->TrackMode = 1;                            //回到循迹模式，注意这段程序很危险！！！可能导致乱跑，如果他乱跑了就赶紧把这段代码删了！
						Tracker->TrackLastData = 0x00;
						break;
					}                             //原设计是超时停车退出防卡死，加上上面两句代码之后就是让他超市之后继续循迹了，这是一个雷点，记得注意
					Tracker_ReadData(Tracker);
					if(Tracker->TrackData & 0x3e)
					{
						Tracker->TrackMode = STOP;
						Tracker->TrackLastData = 0x00; 
						Delay_ms(50);
						break;
					}
				}
			}
			else if(Tracker->TrackLastData & 0x02)//0x02:刚刚右侧直角锐角
			{
				uint16_t Count2 = 0;
				Tracker->TrackMode = RIGHT;//左电机正转右电机反转
				while(1)                   //可能会阻塞主循环。导致遥控信息数据回传等过程受阻，到时候如果有这个问题，可以改uint32_t Num;Num++;来计时
				{
					Count2++;
					if(Count2 >= 60000)             //等待超时
					{
						Tracker->TrackMode = STOP;
						Tracker->TrackMode = 1;                            //回到循迹模式，注意这段程序很危险！！！可能导致乱跑，如果他乱跑了就赶紧把这段代码删了！
						Tracker->TrackLastData = 0x00;
						break;
					}								//原设计是超时停车退出防卡死，加上上面两句代码之后就是让他超市之后继续循迹了，这是一个雷点，记得注意
					
					Tracker_ReadData(Tracker);
					if(Tracker->TrackData & 0x3e)
					{
						Tracker->TrackMode = STOP;
						Tracker->TrackLastData = 0x00; 
						Delay_ms(50);
						break;
					}
				}
			}
			else
			{
				OverRun_Time++;
				if(OverRun_Time == 250)//脱轨时间上限，超过这个视为循迹失败立刻停车
				{
					OverRun_Time = 0;
					Tracker->TrackMode = STOP;
				}
			}
		break;
			
		case 0x08: //中间
			Tracker->TrackCurrentDev = 0;
			break;
		
		case 0x18: //偏右-1
			Tracker->TrackCurrentDev = -1;
			break;
		
		case 0x0c: //偏左1
			Tracker->TrackCurrentDev = 1;
			break;
		
		case 0x10: //偏右-2
			Tracker->TrackCurrentDev = -2;
			break;
		
		case 0x04: //偏左2
			Tracker->TrackCurrentDev = 2;
			break;
		
		case 0x30: //偏右-3
			Tracker->TrackCurrentDev = -4;
			break;
		
		case 0x06: //偏左3
			Tracker->TrackCurrentDev = 4;
			break;
		
		case 0x20: //偏右-4
			Tracker->TrackCurrentDev = -7;
			break;
		
		case 0x02: //偏左4
			Tracker->TrackCurrentDev = 7;
			break;
		
//		case 0x3e: //预留的识别交叉路口的代码
//			Num_of_Cross++;
//			break; 
	}
}

int16_t Dev_SpeedPIDUpdate(int16_t Empower)//Empower指权重，这个值就是上文中的Tracker->TrackCurrentDev
{
	static int8_t Target = 0;
//	float Kp = 6.25,Ki = 0.00;     
//	float Kd = 0.65;
	
	float Kp = 4.5,Ki = 0.00;     
	float Kd = 1.4;
	
	/*这套参数的调节方式如下：起始Kp = 2.0，之后0.5步距地加，选择两个最好的点之间步距0.1地测试细化，
	
	到小车可以正常转弯并且在直线上左右摆动的时候开始引入Kd，Kd通常小于Kp，从0.1开始给，步距0.1，消除小车摇摆的同时，不要过大
	
	除非弯道有长期偏差，比如总是偏向于一侧，否则不加Ki，加的话也很小，0.01-0.05之间并记得加限幅*/
	
	
	static int16_t ErrorInt = 0;
	static int16_t Error1 = 0;
	
	int16_t Error0;
	int16_t Out;
	
	 if (Empower == 0) //偏差为0时，无差速需求，彻底清零并返回0
	{
        ErrorInt = 0;
        Error1 = 0;
        return 0;
    }
	
	
	Error0 = Empower - Target;
	ErrorInt += Error0;
	
	// 积分限幅（防止饱和）
    if (ErrorInt > 50) ErrorInt = 50;
    if (ErrorInt < -50) ErrorInt = -50;
	
	Out = Kp * Error0 + Ki * ErrorInt + Kd * (Error0 - Error1);
	Error1 = Error0;
	
	// 输出限幅（防止过大）
    if (Out > 100) Out = 100;
    if (Out < -100) Out = -100;
	
    return Out;
}                              //这个PID控制器里面的数据类型乱七八糟，于是暂时不使用PID_t的固有结构体以免祸害他人


