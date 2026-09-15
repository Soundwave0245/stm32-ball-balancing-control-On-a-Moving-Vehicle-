#include "stm32f10x.h"                  // Device header
#include "Kalman.h"

// 初始化滤波器
void Kalman1D_Init(Kalman1D_t *kf, float init_x, float init_P, float Q, float R, uint8_t max_invalid)
{
    kf->x = init_x;
    kf->P = init_P;
    kf->Q = Q;
    kf->R = R;
    kf->consecutive_invalid = 0;
    kf->max_invalid = max_invalid;
}

// 带异常检测和防发散的滤波更新函数
// 输入：测量值 z，有效范围 [min_valid, max_valid]
// 输出：滤波后的估计值（若无效则保持原值，连续无效超限则复位）
float Kalman1D_UpdateWithCheck(Kalman1D_t *kf, float z, float min_valid, float max_valid)
{
    // ---- 1. 有效性判断 ----
    if (z >= min_valid && z <= max_valid)
	{
        // 有效测量：重置无效计数器
        kf->consecutive_invalid = 0;

        // ---- 2. 标准卡尔曼更新 ----
        // 预测
        kf->P = kf->P + kf->Q;
        // 更新
        float K = kf->P / (kf->P + kf->R);
        kf->x = kf->x + K * (z - kf->x);
        kf->P = (1.0f - K) * kf->P;
    } 
	else
	{
        // ---- 3. 无效测量处理 ----
        kf->consecutive_invalid++;

        // 若连续无效次数达到阈值，复位滤波器防止发散
        if (kf->consecutive_invalid >= kf->max_invalid)
		{
            // 复位策略：重置状态为0，协方差恢复初始大值（表示高度不确定）
            kf->x = 0.0f;           // 也可设为上一次有效值，此处简单归零
            kf->P = 100.0f;         // 初始协方差，根据实际情况调整
            kf->consecutive_invalid = 0; // 计数清零
            // 注意：复位后不返回新值，x已被重置，后续有效测量会快速收敛
        }
        // 若未达阈值，保持x不变，P也不变（不更新）
    }

    return kf->x;
}





/*————————————卡尔曼滤波器使用示例（以接收一维数组为例）————————————*/
/*

	while (1) 
	{
		if (Serial_GetRxFlag()) 
		{
			// 对四个通道分别滤波
			for (int i = 0; i < CHANNEL_NUM; i++) 
			{
				uint16_t raw = Serial_RxPacket[i];

				// 调用带检查的更新函数，设定有效范围（例如0~1000）,这个需要调整！！！
				filtered_float[i] = Kalman1D_UpdateWithCheck(&kf[i], (float)raw, 0.0f, 1000.0f);
			}

			// 将浮点滤波结果转为整型（四舍五入）并显示
			uint16_t filtered_int[4];
			for (int i = 0; i < 4; i++)
			{
				filtered_int[i] = (uint16_t)(filtered_float[i] + 0.5f);
			}

			// 显示四个通道的滤波后整型值（可根据实际 OLED 布局调整）
			OLED_ShowNum(1, 1, filtered_int[0], 4);   // 通道0
			OLED_ShowNum(2, 1, filtered_int[1], 4);   // 通道1
			OLED_ShowNum(3, 1, filtered_int[2], 4);   // 通道2
			OLED_ShowNum(4, 1, filtered_int[3], 4);   // 通道3
		}
	}



*/



/*———————————————————调参技巧————————————————————*/

/*
1.参数微调

如果滤波后响应太慢（滞后明显），尝试 增大 Q 或 减小 R（使滤波器更信任测量值）。

如果仍有毛刺，适当 增大 R 或 减小 Q（更信任估计值）。

建议先固定 Q，调节 R，找到噪声抑制与响应速度的平衡点。

2.限幅阈值

根据你的实际物理量（如速度 0~100、角度 0~360）设置合理的上下限，彻底杜绝 255/256 这种非法值进入滤波器。

3.异常值处理

在连续多次收到异常值时，将滤波器状态 重置（重新初始化协方差 P 和状态 x），避免滤波器因异常值发散。
*/







/*—————————————————永 远 怀 念———————————————————*/


// 初始化滤波器，提供初始估计值和噪声参数
/*这个是很简单的版本的滤波器，但是缺少自校准能力，累计过多的无效值会导致滤波器因异常值发散*/
//void Kalman1D_Init(Kalman1D_t *kf, float init_x, float init_P, float Q, float R)
//{
//    kf->x = init_x;
//    kf->P = init_P;
//    kf->Q = Q;
//    kf->R = R;
//}

// 输入：测量值 z
// 输出：滤波后的估计值
//float Kalman1D_Update(Kalman1D_t *kf, float z)
//{
//    // ---- 预测 ----
//    // 状态预测：x_hat = x_hat （常数模型）
//    // 协方差预测：P = P + Q
//    kf->P = kf->P + kf->Q;

//    // ---- 更新 ----
//    // 卡尔曼增益
//    float K = kf->P / (kf->P + kf->R);
//    // 状态更新
//    kf->x = kf->x + K * (z - kf->x);
//    // 协方差更新
//    kf->P = (1.0f - K) * kf->P;

//    return kf->x;
//}


/*————————————卡尔曼滤波器（无阈值复位的旧版）使用示例（以接收一维数组为例）————————————*/
/*

                                           【主函数里】
		在最前面定义：
		#define CHANNEL_NUM 4                   //需要处理几个数据，这里的设计是用KalmanFilter对串口接收到的数组滤波，串口一次进来四个数，CHANNEL_NUM给4
		Kalman1D_t kalman_filters[CHANNEL_NUM];    //卡尔曼滤波器结构体定义
		
		
		主循环之前的初始化部分：
		
		//卡尔曼滤波器赋初值和初始化
		float init_value = 0.0f;   // 初始估计值，可根据情况调整
		float init_P = 100.0f;     // 初始协方差，表示对初始值的不确定度
		float Q = 0.1f;           // 过程噪声，值越小滤波越平滑，响应越慢
		float R = 10.0f;          // 测量噪声，值越大对测量值信任越低（噪声大时增大）

		for (int i = 0; i < CHANNEL_NUM; i++) 
		{
			Kalman1D_Init(&kalman_filters[i], init_value, init_P, Q, R);     //卡尔曼滤波器初始化
		}
		
		
		while(1)
		{
			if (Serial_GetRxFlag())   // 收到新数据包
			{
				// 对每个通道进行滤波，同时剔除明显异常值（如255/256）
				for (int i = 0; i < 4; i++) 
					{
					uint16_t raw = Serial_RxPacket[i];
					// 先限幅，过滤乱码（假设正常值范围 0~1000，可根据实际情况调整），数值合法性检查
					if (raw >= 0 && raw <= 1000)
					{
						// 转换为浮点（如果原始数据是整型，需按实际量纲转换）
						float measurement = (float)raw;
						float filtered = Kalman1D_Update(&kalman_filters[i], measurement);

						// 如果需要回存为整型，可四舍五入
						
						// Serial_RxPacket[i] = (uint16_t)(filtered + 0.5f);
						
						// 但一般用单独的滤波结果数组
						filtered_values[i] = filtered;  // 定义 float filtered_values[4]
					} 
					
					else 
					{
						// 异常值：不更新滤波器，保持上一次估计值
						// 也可以选择将上一次估计值作为当前值
					}
				}
				// 显示或使用滤波后的值
				OLED_ShowNum(1, 1, (uint16_t)(filtered_values[0] + 0.5f), 4);
			}
		}
*/


