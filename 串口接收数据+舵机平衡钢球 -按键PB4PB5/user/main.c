#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Serial.h"
#include "Key.h"
#include "Kalman.h"
#include "Buzzer.h"
#include "Servo.h"
#include "PID.h"
#include <math.h>
#include "Encoder.h"
#include "Timer.h"

extern uint8_t Serial_TxPacket[];    //串口发送数据，大小在Serial.c里修改
extern uint8_t Serial_RxPacket[];    //串口接收数据，大小在Serial.c里修改，记得改状态机里面对应的接收数据个数

volatile int16_t Encoder_Left = 0;
volatile int16_t Encoder_Right = 0;

PID_t ServoPID;//PID结构体定义

uint16_t Time = 0;

uint8_t KeyNum = 0;


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
	float R = 5.0f;          // 测量噪声，值越大对测量值信任越低（噪声大时增大）
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
            // 有效范围 0~640
            filtered_float[i] = Kalman1D_UpdateWithCheck(&kalman_filter[i], (float)raw, 0.0f, 640.0f);
            // 转为整型（四舍五入）
            filtered_int[i] = (uint16_t)(filtered_float[i] + 0.5f);
        }
       
    }
}





/*————————和视觉模块配合接口————————————*/

#define Data_Max 640.0f;
#define Data_Min 0.0f;

#define Serial_Length_of_Packet 4

#define Length 25.0f
#define Center 12.5f

#define Step 5.0f

#define Pow Data_Max/Length

#define Epsilon_for_Vision 0.1f // 浮点数直接比大小很危险，需要这个和fabs函数搭配
#define Epsilon 0.25f // 位置偏差值

float Coordinate_X_Raw = 0.0f;
uint16_t Coordinate_X = 0;








// 将低8位(low)和高8位(high)拼接为16位值
uint16_t Combine_Bytes(uint8_t Low, uint8_t High) 
{
    return ((uint16_t)High << 8) | Low;
}


/*—————————————————串口数据处理函数—————————————————————*/
void Coordinate_Handler(void)
{
	if(Serial_GetRxFlag())
	{
		
		if(Serial_RxPacket[0] == 0xAA && Serial_RxPacket[3] == 0xAB)      //这是两个固定数据校验位，需要做视觉的同学告诉你他定义的是什么，需要到时候修改！！！！
		{
			if((Combine_Bytes(Serial_RxPacket[2],Serial_RxPacket[1]) - 640.0) <=  Epsilon_for_Vision && (Combine_Bytes(Serial_RxPacket[2],Serial_RxPacket[1]) - 0.0) >=  -Epsilon_for_Vision)//数据没有超出限度
			{
				Coordinate_X_Raw = Combine_Bytes(Serial_RxPacket[2],Serial_RxPacket[1]) / 640.0 * 25;
				Coordinate_X = (uint16_t)(Coordinate_X_Raw + 0.5f);    //四舍五入
			}
		}
	}
}


/*————————————舵机相关参数————————————*/

#define Servo_BaseAngle 90



/*针对后面的行车过程中的钢球平衡，车启动和转弯的时候都会造成不小的干扰，所以加入速度前馈，包括加速度前馈和转弯补偿*/

/*这里使用一阶低通滤波实现融合了加速度和角度偏移的速度补偿并实现平滑过渡*/

/*------------------------------速度前馈相关变量-------------------------------------------*/
// 速度、加速度、前馈变量
static int16_t Last_Speed = 0;		//保留本次和上一次变量，为一阶低通滤波做准备
static float Accel_Filtered = 0.0f;
float Speed_Feedforward = 0.0f;      // 加速度前馈（角度偏移）
float Turn_Compensation = 0.0f;      // 转弯补偿

#define K_ACCEL  0.2f    // 需调参标定！！！！！！
#define K_TURN   0.05f   // 需调参标定！！！！！！

uint16_t Time_for_Moving = 0;


/*————————按键切换模式，3小问为Mode1，4-5小问为Mode2，6问为Mode3————————————*/

//定义模式参数
uint16_t CurrMode = 0;
uint16_t NextMode = 0;


uint8_t Mode1_Running = 0; //模式1运行参数
uint8_t Mode2_Running = 0; //模式2运行参数
uint8_t Mode3_Running = 0; //模式3运行参数


//模式1的状态机
uint16_t Status = 0;

/*___________________Mode0__________________*/
void Mode0_Init(void)
{
	OLED_Clear();
	Servo_SetAngle(Servo_BaseAngle);//调整为中间状态
	OLED_ShowString(1,1,"[Mode0]");
}

void Mode0_Loop(void)
{
	
}
void Mode0_Exit(void)
{
	
}


/*——————————Mode1需要的标志位————————————*/
uint8_t Flag_Mode1Complete = 2;//模式1任务完成标志位 


/*___________________Mode1__________________*/
void Mode1_Init(void)
{
	OLED_Clear();
	
	Servo_SetAngle(Servo_BaseAngle);//调整为中间状态
	 
	OLED_ShowString(1,1,"[Mode1]");
	
	OLED_ShowString(2,1,"K2>>");
	
	Mode1_Running = 0;

	Status = 0;
}




/*——状态机处理函数——*/


void Mode1_StatusHandler(void)
{
	if(Status == 0)
	{
		ServoPID.Target = 17.5;
		ServoPID.Actual = Coordinate_X_Raw;
		PID_Update(&ServoPID);
		Servo_SetAngle(Servo_BaseAngle + ServoPID.Out);
		
		if (fabs((Coordinate_X_Raw) - 17.5) <= Epsilon)    //这个17.5，12.5这种数可以根据实际情况给小一点或者大一点，就是提前一点或者滞后一点切换下一个模式！！！！！
		{
			Status = 1;
		}
	}
	
	else if(Status == 1)
	{
		ServoPID.Target = 12.5;
		ServoPID.Actual = Coordinate_X_Raw;
		PID_Update(&ServoPID);
		Servo_SetAngle(Servo_BaseAngle + ServoPID.Out);
		
		if (fabs((Coordinate_X_Raw) - 12.5) <= Epsilon)
		{
			Status = 2;
		}
	}
	else if(Status == 2)
	{
		ServoPID.Target = 7.5;
		ServoPID.Actual = Coordinate_X_Raw;
		PID_Update(&ServoPID);
		Servo_SetAngle(Servo_BaseAngle + ServoPID.Out);
		
		if (fabs((Coordinate_X_Raw) - 7.5) <= Epsilon)
		{
			Status = 3;
		}
	}
	else if(Status == 3)
	{
		ServoPID.Target = 12.5;
		ServoPID.Actual = Coordinate_X_Raw;
		PID_Update(&ServoPID);
		Servo_SetAngle(Servo_BaseAngle + ServoPID.Out);
		
		if (fabs((Coordinate_X_Raw) - 12.5) <= Epsilon)
		{
			Status = 4;
		}
	}
	else if(Status == 4)
	{
		Servo_SetAngle(Servo_BaseAngle);
		Flag_Mode1Complete = 1;                   //任务完成状态位置1，停止计时，状态机停在Status = 4的状态
	}
}


void Mode1_Loop(void)
{
	/*这个非阻塞检测是优秀的经验，应当记录一下，之前用Init函数里用while(1)循环，导致不运行Mode1不能往后切，体验很不好*/
	if (KeyNum == 2 && Mode1_Running == 0) //检测启动按键（非阻塞）
	{
        Mode1_Running = 1;
        OLED_ShowString(2,1,"Running...");
		OLED_ShowString(3,1,"Time:   .");
    }
		
	if (Mode1_Running)     //只有启动后才执行逻辑
	{
			
		Flag_Mode1Complete = 0;     //这里给0才开始计时，否则刚到Mode1的界面就开始计时了	
		
		/*——坐标处理——*/
		Coordinate_Handler();
		
		/*——时间显示——*/
		OLED_ShowNum(3,6,(Time / 1000),3);
		OLED_ShowNum(3,10,(Time % 1000),3);
		
		/*——状态机Mode1_StatusHandler()在定时中断里面——*/
	}
}


void Mode1_Exit(void)
{
	Servo_SetAngle(Servo_BaseAngle);
	
	Time = 0;	
	
	Status = 0;//状态机状态清零以便于下一次使用
	
	//我总感觉下面这个串口数据包清零操作形同虚设，可以删掉
	for(int i = 0;i < Serial_Length_of_Packet;i++)
	{
		Serial_RxPacket[i] = 00;
	}
	
	Flag_Mode1Complete = 2;                         //把他拉回2以便下一次使用
}



/*___________________Mode2__________________*/
void Mode2_Init(void)
{
	OLED_Clear();
	
	Servo_SetAngle(Servo_BaseAngle);
	OLED_ShowString(1,1,"[Mode2]");
	OLED_ShowString(2,1,"K2>>");
	
	
	Mode2_Running = 0;
	
//	while(1)
//	{
//		KeyNum = Key_GetNum();
//		if(KeyNum == 2)
//		{
//			OLED_ShowString(2,1,"Running...");
//			break;
//		}
//	}
}

#define Alpha 0.15f                              //一阶低通滤波系数

//实际测试的时候只是状态机+PID_UpDate就足够让OLED闪屏了，这里为了保留算力，省略了误差较小一步到位的一阶低通滤波的判断逻辑

//float Cauculate_Speed_Feedforward_and_Turn_Compensation(float Alpha_for_Filter,int16_t Encoder_Left,int16_t Encoder_Right,
//															float K_ACCEL_for_Speed_Feedforward, float K_TURN_for_Turn_Compensation)
//{
//		// 计算平均速度（单位：脉冲/10ms）
//		int16_t Speed_Avg = (Encoder_Left + Encoder_Right) / 2;
//		
//		// 加速度 = (当前 - 上次) / 0.01s
//		float accel_raw = (float)(Speed_Avg - Last_Speed) / 0.01f;
//		Last_Speed = Speed_Avg;                                          //保留本次和上一次变量，为一阶低通滤波做准备
//		
//		// 一阶低通滤波
//		Accel_Filtered = (1 - Alpha) * Accel_Filtered + Alpha * accel_raw;
//		
//		// 前馈量
//		Speed_Feedforward = K_ACCEL * Accel_Filtered;
//		
//		// 转弯补偿（左右差）
//		int16_t Speed_Diff = Encoder_Left - Encoder_Right;
//		
//		Turn_Compensation = K_TURN * Speed_Diff;
//	
//		float Full_Compensation = Speed_Feedforward + Turn_Compensation;
//	
//		return Full_Compensation;
//}
//把计算补偿的过程封装成函数了，但是总觉得有点为了封装而封装了，调用起来并不简洁，所以还是直接写两遍

void Mode2_Loop(void)
{
	if (KeyNum == 2 && Mode2_Running == 0) //检测启动按键（非阻塞）
	{
        Mode2_Running = 1;
        OLED_ShowString(2,1,"Running...");
		ServoPID.Target = 12.5;
    }
	
	if (Mode2_Running)
	{
		if(Time_for_Moving >= 10)
		{
			 // 计算平均速度（单位：脉冲/10ms）
			int16_t Speed_Avg = (Encoder_Left + Encoder_Right) / 2;
			
			// 加速度 = (当前 - 上次) / 0.01s
			float Accel_Raw = (float)(Speed_Avg - Last_Speed) / 0.01f;
			Last_Speed = Speed_Avg;                                          //保留本次和上一次变量，为一阶低通滤波做准备
			
			// 一阶低通滤波
			Accel_Filtered = (1 - Alpha) * Accel_Filtered + Alpha * Accel_Raw;
			
			// 前馈量
			Speed_Feedforward = K_ACCEL * Accel_Filtered;
			
			// 转弯补偿（左右差）
			int16_t Speed_Diff = Encoder_Left - Encoder_Right;
			
			Turn_Compensation = K_TURN * Speed_Diff;
			
			Time_for_Moving = 0;
		}
		
		
		Coordinate_Handler();   // 更新球位置
        
        float Base_Target = 12.5f;   // 中心
		
		
        // 叠加前馈：加速时球后滚，目标减小（舵机前倾）
        float Final_Target = Base_Target - Speed_Feedforward + Turn_Compensation;
        // 限幅
        if (Final_Target < 7.5f) Final_Target = 7.5f;
        if (Final_Target > 17.5f) Final_Target = 17.5f;
        
        ServoPID.Target = Final_Target;
        ServoPID.Actual = Coordinate_X_Raw;
		
		//积分分离逻辑（核心防过冲）
        float Current_Error = fabs(ServoPID.Target - ServoPID.Actual);
        if (Current_Error > 0.5f)                                           //这个极限距离0.5也可以调的，但是不建议，如果调了记得把Mode3的也关注一下
		{  
            // 误差较大（>0.5cm）时，我们认为球还在追赶虚拟目标，禁用积分，防止积分饱和
            ServoPID.Ki = 0.0f;   
        }
		else 
		{
            // 误差较小（接近目标）时，恢复你原来设定好的Ki值（比如0.01）
            ServoPID.Ki = 0.01f;                                                    // 这里要改成main函数里同款Ki
        }
		
        // PID 更新和舵机输出在中断中统一执行（无需在这里调用）
		
		
	}
}

void Mode2_Exit(void)
{
	// 1. 停止运行标志（防止退出后中断或Loop还在偷偷调舵机）
	Mode2_Running = 0;
	
	// 2. 舵机回中（让摆杆回到水平位置，防止钢球在停车后还憋着劲，也方便测试员观察）
	Servo_SetAngle(Servo_BaseAngle);
	
	// 3. 清空PID积分项（防止下次进入Mode2时，积分还残留着上次的累计值，导致启动瞬间剧烈抖动）
	ServoPID.ErrorInt = 0;
	
	// 4. 清零时间累加器（防止下次进入时，Time_for_Moving带着旧值导致立即触发速度计算）
	Time_for_Moving = 0;
}


/*留给未来可能抽风的自己：

你可能会想，要不要把 Last_Speed（上次速度）和 Accel_Filtered（滤波后的加速度）也清零？

千万不要！

如果在 Exit 里把它们强行置为 0，当你下次再次进入 Mode2 启动时，编码器读出当前速度是 100，而 Last_Speed 是 0，

算出来的 accel_raw 会瞬间变成 (100 - 0)/0.01 = 10000，这会导致舵机猛地抽动一下（假加速冲击）。

正确做法是：保留它们的旧值，让滤波器在下次启动时从当前真实速度开始自然过渡，这样起步才足够丝滑。
*/


/*___________________Mode3__________________*/


float Target_Any = 12.5f; // 存储用户指定的任意目标

void Mode3_Init(void)
{
	OLED_Clear();
	
	Servo_SetAngle(Servo_BaseAngle);
	OLED_ShowString(1,1,"[Mode3]");
	OLED_ShowString(2,1,"Pos:");
	OLED_ShowString(3,1,"K2: Set&Run");
	
	Mode3_Running = 0;
	Target_Any = 12.5f; // 复位
//	while(1)
//	{
//		KeyNum = Key_GetNum();
//		if(KeyNum == 2)
//		{
//			OLED_ShowString(2,1,"Running...");
//			break;
//		}
//	}
}

void Mode3_Loop(void)
{
	if (!Mode3_Running) //未运行时，实时显示当前球的位置，方便测试员决定在哪个位置按启动
	{
		Coordinate_Handler();
		OLED_ShowNum(2,5, (uint16_t)(Coordinate_X_Raw * 10), 3); // 显示比如 125 代表 12.5cm             //可以优化一下显示，不优化的话单位就是毫米，也没事
	}
	
	if (KeyNum == 2 && Mode3_Running == 0) //检测启动按键（非阻塞）
	{
        // 先更新一次坐标，确保记录的是最新的位置
		Coordinate_Handler(); 
		Target_Any = Coordinate_X_Raw; // 记录当前位置（单位 cm）
		
		Mode3_Running = 1;
		ServoPID.ErrorInt = 0; // 清积分
		OLED_ShowString(3,1,"                 ");
		OLED_ShowString(3,1,"Running...");
		
		ServoPID.Target = Target_Any; //更新当前位置为目标位置
		
    }
	
	if (Mode3_Running)
	{
		// ---------- 速度、加速度、前馈计算（和Mode2一模一样的代码） ----------
		
			if(Time_for_Moving >= 10) // 10ms周期计算一次
			{
				int16_t Speed_Avg = (Encoder_Left + Encoder_Right) / 2;
				float Accel_Raw = (float)(Speed_Avg - Last_Speed) / 0.01f;
				Last_Speed = Speed_Avg;
				Accel_Filtered = (1 - Alpha) * Accel_Filtered + Alpha * Accel_Raw;
				Speed_Feedforward = K_ACCEL * Accel_Filtered;
				
				int16_t Speed_Diff = Encoder_Left - Encoder_Right;
				Turn_Compensation = K_TURN * Speed_Diff;
				
				Time_for_Moving = 0;
			}
			
			Coordinate_Handler();   // 更新球位置
			
			// 【唯一与Mode2不同的地方】：基准是用户指定的 Target_Any
			float Base_Target = Target_Any; 
			float Final_Target = Base_Target - Speed_Feedforward + Turn_Compensation;
			
			// 限幅
			if (Final_Target < 7.5f) Final_Target = 7.5f;
			if (Final_Target > 17.5f) Final_Target = 17.5f;
			
			ServoPID.Target = Final_Target;
			ServoPID.Actual = Coordinate_X_Raw;
			
			// 积分分离逻辑（与Mode2完全相同）
			float Current_Error = fabs(ServoPID.Target - ServoPID.Actual);
			if (Current_Error > 0.5f) 												//这个极限距离0.5也可以调的，但是不建议，如果调了记得把Mode2的也关注一下
			{  
				ServoPID.Ki = 0.0f;   
			}
			else 
			{
				ServoPID.Ki = 0.01f;                                                    // 这里要改成main函数里同款Ki   
			}	
	}
}


void Mode3_Exit(void)
{
	// 1. 停止运行标志
	Mode3_Running = 0;
	
	// 2. 舵机回中（防止停车后舵机还憋着劲）
	Servo_SetAngle(Servo_BaseAngle);
	
	// 3. 清空PID积分项（防止下次启动时残留积分导致抖动）
	ServoPID.ErrorInt = 0;   // 注意你的变量名是 ErrorInt，不是 Integral
	
	// 4. 清零时间累加器
	Time_for_Moving = 0;
	
	// 注意：千万不要清零 Last_Speed 和 Accel_Filtered，原因同 Mode2
	//Target_Any不需要清零，它会在下次启动时被覆盖
}



/**********************************************主函数在这里***********************************************/

int main(void)
{
	
	/*——————————————————————硬件单元初始化————————————————————————*/
	
	OLED_Init();
	Serial_Init(); 
	BUZZER_Init();
	Servo_Init();
	Key_Init();
	Encoder1_Init();
	Encoder2_Init();
	Timer_Init();
	
	Mode0_Init();                     //先给一个空闲状态
	Servo_SetAngle(Servo_BaseAngle);  //回中
	
	/*————————————————————Utils各种滤波器初始化————————————————————————*/
	KalmanFilter_Init();
	
	
	
	/*———————————————————PID参数在这里！！！———————————————————————————*/
	ServoPID.Kp = 0.4;
	ServoPID.Ki = 0.0;  //调节这个之后要在Mode2和Mode3的Loop函数里改一下对应的Ki
	ServoPID.Kd = 0.04;
	
	ServoPID.OutMax = 10;
	ServoPID.OutMin = -10;
	
	
	/*————————————主循环——————————————*/
    while(1)
    {
		KeyNum = Key_GetNum();
		
		if(KeyNum == 1)
		{
			NextMode++;
			if(NextMode > 3){NextMode = 0;}
		}
		
		
       if(CurrMode == NextMode)
	   {
			switch (CurrMode)
			{
				case 0:Mode0_Loop(); break;
				
				case 1:Mode1_Loop(); break;
				
				case 2:Mode2_Loop(); break;
				
				case 3:Mode3_Loop(); break;
			}
	   }
	   
	   else
	   {
			switch(CurrMode)
			{
				case 0:Mode0_Exit(); break;
				
				case 1:Mode1_Exit(); break;
				
				case 2:Mode2_Exit(); break;
				
				case 3:Mode3_Exit(); break;
			}
			
			switch(NextMode)
			{
				case 0:Mode0_Init(); break;
				
				case 1:Mode1_Init(); break;
				
				case 2:Mode2_Init(); break;
				
				case 3:Mode3_Init(); break;
			}
			
			CurrMode = NextMode;
	   }
		
		
    }
}

#define PID_Update_Time 20

void TIM4_IRQHandler(void)
{
	static uint16_t Count = 0;
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
	{
		Encoder_Left = Encoder1_Get();
		Encoder_Right = Encoder2_Get();
		
		Time_for_Moving++;                            //这东西也是十秒钟调节一次，为了防止占用中断资源或者在主循环里用提不用他都计算抢占CPU资源，他的运算放在对应Mode2/3函数里进行
		
		if(CurrMode == 1 && Flag_Mode1Complete == 0)  //这里检测的是CurrMode，还好，如果检测的是NextMode就会出现无论是否Key2进入模式运行都会调节钢球了，那就是个Bug了
		{
			Time++;
			Count++;
			
			if(Count >= PID_Update_Time)                                  
			{
				Mode1_StatusHandler();//状态机
				
				/*根据需要改调节时间，有一种风险是PID计算消耗时间超过1ms会阻塞定时中断造成卡顿
				
				但是目前没测试会不会出现这个问题也没有想到太好的解决办法
				
				不过按照倒立摆的经验来看好像不太会，双环倒立摆都不会*/
			}
		}
	
		
		else if(CurrMode == 2 || CurrMode == 3)      //给速度前馈补偿预留的空间
		{
			Count++;
			
			if(Count >= PID_Update_Time)                                  
			{
				PID_Update(&ServoPID);
				Servo_SetAngle(Servo_BaseAngle + ServoPID.Out);
			}
		}
		
		
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	}
}

