#include "stm32f10x.h"                  // Device header
#include <stdint.h>
#include <math.h>
#include "imu.h"

/*用于存储姿态的四元数*/
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;


void IMU_InitQuaternion(float ax, float ay, float az)//四元数初始化，ax ay az 加速度值
{
	/*使用加速度值，计算得到横滚角和俯仰角*/
	float theta = atan2(ay, az);
	float phi = atan2(-ax, sqrt(az * az + ay * ay));
	float psi = 0;
	
	float a = psi * 0.5f, b = phi * 0.5f, g = theta * 0.5f;
	float ca = cosf(a), sa = sinf(a);
	float cb = cosf(b), sb = sinf(b);
	float cg = cosf(g), sg = sinf(g);
	
	/*初始化四元数*/
	q0 = ca*cb*cg + sa*sb*sg;
	q1 = ca*cb*sg - sa*sb*cg;
	q2 = ca*sb*cg + sa*cb*sg;
	q3 = sa*cb*cg - ca*sb*sg;
	
	/*四元数归一化*/
	float norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 /= norm;
	q1 /= norm;
	q2 /= norm;
	q3 /= norm;
}

/**
  * 函    数：3轴姿态解算（仅使用加速度计）
  * 参    数：ax ay az 加速度值，单位：任意
  * 参    数：roll pitch 解算出来的横滚角和俯仰角，输出参数，单位：度
  * 返 回 值：无
  * 说    明：只在静止状态下准确，运动时会受运动加速度影响
  *           加速度计无法解算出偏航角
  */
void IMU_3Axis_Acc(float ax, float ay, float az, float *roll, float *pitch)
{
	/*使用加速度值，计算得到横滚角和俯仰角*/
	*roll = atan2(ay, az) * RAD_TO_DEG;
	*pitch = atan2(-ax, sqrt(az * az + ay * ay)) * RAD_TO_DEG;
}

/**
  * 函    数：3轴姿态解算（仅使用陀螺仪）
  * 参    数：gx gy gz 陀螺仪角速度值，单位：度每秒
  * 参    数：roll pitch yaw 解算出来的横滚角、俯仰角和偏航角，输出参数，单位：度
  * 参    数：dt 调用此函数的周期，单位：秒
  * 返 回 值：无
  * 说    明：使用陀螺仪更新全局变量的姿态四元数，输出此四元数对应的欧拉角
  *           输出的欧拉角会因为陀螺仪零偏而逐渐漂移
  */
void IMU_3Axis_Gyro(float gx, float gy, float gz, float *roll, float *pitch, float *yaw, float dt)
{
	/*角度制转弧度制*/
	/*若输入参数已经是弧度制，则不需要此转换*/
	gx *= DEG_TO_RAD;
	gy *= DEG_TO_RAD;
	gz *= DEG_TO_RAD;
	
	/*四元数更新*/
    q0 += 0.5f * (-q1*gx - q2*gy - q3*gz) * dt;
    q1 += 0.5f * ( q0*gx - q3*gy + q2*gz) * dt;
    q2 += 0.5f * ( q3*gx + q0*gy - q1*gz) * dt;
    q3 += 0.5f * (-q2*gx + q1*gy + q0*gz) * dt;
    
	/*四元数归一化*/
    float norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 /= norm;
    q1 /= norm;
    q2 /= norm;
    q3 /= norm;
	
	/*四元数转欧拉角*/
    *roll = atan2f(2.0f*(q2*q3 + q0*q1), 1.0f - 2.0f*(q1*q1 + q2*q2)) * RAD_TO_DEG;
	*pitch = asinf(2.0f * (q0*q2 - q1*q3)) * RAD_TO_DEG;
    *yaw = atan2f(2.0f*(q1*q2 + q0*q3), 1.0f - 2.0f*(q2*q2 + q3*q3)) * RAD_TO_DEG;
}

/**
  * 函    数：6轴姿态解算（使用加速度计和陀螺仪互补滤波融合）
  * 参    数：ax ay az 加速度值，单位：任意
  * 参    数：gx gy gz 陀螺仪角速度值，单位：度每秒
  * 参    数：roll pitch yaw 解算出来的横滚角、俯仰角和偏航角，输出参数，单位：度
  * 参    数：dt 调用此函数的周期，单位：秒
  * 返 回 值：无
  * 说    明：使用加速度计和陀螺仪互补滤波融合，更新全局变量的姿态四元数，输出此四元数对应的欧拉角
  *           输出的横滚角和俯仰角准确无漂移，但偏航角会因为陀螺仪零偏而逐渐漂移
  */
void IMU_6Axis_Gyro_Acc(float ax, float ay, float az, 
						float gx, float gy, float gz, 
						float *roll, float *pitch, float *yaw, float dt)
{
	/*角度制转弧度制*/
	/*若输入参数已经是弧度制，则不需要此转换*/
	gx *= DEG_TO_RAD;
	gy *= DEG_TO_RAD;
	gz *= DEG_TO_RAD;
	
	/*如果加速度全为0，则退出此函数，避免加速度归一化时产生除零错误*/
	if((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)) return;
	
	/*加速度归一化，对应实际重力方向[ax, ay, az]*/
    float norm = sqrtf(ax*ax + ay*ay + az*az);
	ax /= norm;
	ay /= norm;
	az /= norm;
	
	/*提取重力分量*/
	/*四元数提取的重力分量，对应理论重力方向[vx, vy, vz]*/
	float vx = 2.0f*(q1*q3 - q0*q2);
	float vy = 2.0f*(q0*q1 + q2*q3);
	float vz = 1.0f - 2.0f*(q1*q1 + q2*q2);
	
	/*实际重力方向和理论重力方向，叉乘得到误差*/
	/*e的方向表示a到v旋转的轴，即减小误差的方向*/
	float ex = ay * vz - az * vy;
	float ey = az * vx - ax * vz;
	float ez = ax * vy - ay * vx;
	
	/*误差补偿*/
	float Kp = 0.2f;		//PI互补滤波器的Kp系数，决定了四元数姿态贴近加速度计姿态的快慢
	float Ki = 0.1f;		//PI互补滤波器的Ki系数，用于消除稳态误差，此值过大可能导致不稳定
	static float exInt, eyInt, ezInt;	//积分变量
	
	/*误差逐渐累加到积分变量中，进行误差积分*/
	exInt += ex * dt;
	eyInt += ey * dt;
	ezInt += ez * dt;
	
	/*误差[ex, ey, ez]经过PI互补滤波器，补偿给陀螺仪角速度，进而使四元数姿态逐渐贴近加速度计姿态*/
	gx += Kp * ex + Ki * exInt;
	gy += Kp * ey + Ki * eyInt;
	gz += Kp * ez + Ki * ezInt;
	
	/*四元数更新*/
	/*此时的[gx, gy, gz]，包含了真实转动和补偿转动，一起用于姿态更新*/
    q0 += 0.5f * (-q1*gx - q2*gy - q3*gz) * dt;
    q1 += 0.5f * ( q0*gx - q3*gy + q2*gz) * dt;
    q2 += 0.5f * ( q3*gx + q0*gy - q1*gz) * dt;
    q3 += 0.5f * (-q2*gx + q1*gy + q0*gz) * dt;
    
	/*四元数归一化*/
    norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 /= norm;
    q1 /= norm;
    q2 /= norm;
    q3 /= norm;
	
	/*四元数转欧拉角*/
    *roll = atan2f(2.0f*(q2*q3 + q0*q1), 1.0f - 2.0f*(q1*q1 + q2*q2)) * RAD_TO_DEG;
	*pitch = asinf(2.0f * (q0*q2 - q1*q3)) * RAD_TO_DEG;
    *yaw = atan2f(2.0f*(q1*q2 + q0*q3), 1.0f - 2.0f*(q2*q2 + q3*q3)) * RAD_TO_DEG;
}

/**
  * 函    数：9轴姿态解算（使用加速度计、陀螺仪和磁力计互补滤波融合）
  * 参    数：ax ay az 加速度值，单位：任意
  * 参    数：gx gy gz 陀螺仪角速度值，单位：度每秒
  * 参    数：mx my mz 校准后的磁力值，单位：任意
  * 参    数：roll pitch yaw 解算出来的横滚角、俯仰角和偏航角，输出参数，单位：度
  * 参    数：dt 调用此函数的周期，单位：秒
  * 返 回 值：无
  * 说    明：使用加速度计、陀螺仪和磁力计互补滤波融合，更新全局变量的姿态四元数，输出此四元数对应的欧拉角
  *           输出的横滚角、俯仰角和偏航角均准确无漂移，但要注意避免磁力计被干扰
  */
void IMU_9Axis_Gyro_Acc_Mag(float ax, float ay, float az, 
							float gx, float gy, float gz, 
							float mx, float my, float mz, 
							float *roll, float *pitch, float *yaw, float dt)
{
	float q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	
	/*角度制转弧度制*/
	/*若输入参数已经是弧度制，则不需要此转换*/
	gx *= DEG_TO_RAD;
	gy *= DEG_TO_RAD;
	gz *= DEG_TO_RAD;
	
	/*加速度归一化，对应实际重力方向[ax, ay, az]*/
    float norm = sqrtf(ax*ax + ay*ay + az*az);
	ax /= norm;
	ay /= norm;
	az /= norm;
	
	/*磁力计归一化，对应实际磁力方向[mx, my, mz]*/
    norm = sqrtf(mx*mx + my*my + mz*mz);
	mx /= norm;
	my /= norm;
	mz /= norm;
	
	/*重复的计算放在前面统一执行*/
//	q0q0 = q0 * q0;
	q0q1 = q0 * q1;
	q0q2 = q0 * q2;
	q0q3 = q0 * q3;
	q1q1 = q1 * q1;
	q1q2 = q1 * q2;
	q1q3 = q1 * q3;
	q2q2 = q2 * q2;
	q2q3 = q2 * q3;
	q3q3 = q3 * q3; 
	
	/*地球磁场的参考方向*/
	float hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
	float hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
	float bx = sqrt(hx * hx + hy * hy);
	float bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
		
	/*提取重力分量*/
	/*四元数提取的重力分量，对应理论重力方向[vx, vy, vz]*/
	float vx = 2.0f*(q1q3 - q0q2);
	float vy = 2.0f*(q0q1 + q2q3);
	float vz = 1.0f - 2.0f*(q1q1 + q2q2);
	
	/*提取磁力分量*/
	/*四元数提取的磁力分量，对应理论磁力方向[wx, wy, wz]*/
    float wx = 2.0f * bx * (0.5f - q2q2 - q3q3) + 2.0f * bz * (q1q3 - q0q2);
    float wy = 2.0f * bx * (q1q2 - q0q3) + 2.0f * bz * (q0q1 + q2q3);
    float wz = 2.0f * bx * (q0q2 + q1q3) + 2.0f * bz * (0.5f - q1q1 - q2q2);  
	
	/*叉乘得到误差*/
	float ex = (ay * vz - az * vy) + (my * wz - mz * wy);
	float ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
	float ez = (ax * vy - ay * vx) + (mx * wy - my * wx);

	/*误差补偿*/
	float Kp = 0.2f;		//PI互补滤波器的Kp系数，决定了四元数姿态贴近加速度计姿态和磁力计姿态的快慢
	float Ki = 0.1f;		//PI互补滤波器的Ki系数，用于消除稳态误差，此值过大可能导致不稳定
	static float exInt, eyInt, ezInt;	//积分变量
	
	/*误差逐渐累加到积分变量中，进行误差积分*/
	exInt += ex * dt;
	eyInt += ey * dt;
	ezInt += ez * dt;
	
	/*误差[ex, ey, ez]经过PI互补滤波器，补偿给陀螺仪角速度，进而使四元数姿态逐渐贴近加速度计姿态和磁力计姿态*/
	gx += Kp * ex + Ki * exInt;
	gy += Kp * ey + Ki * eyInt;
	gz += Kp * ez + Ki * ezInt;
	
	/*四元数更新*/
	/*此时的[gx, gy, gz]，包含了真实转动和补偿转动，一起用于姿态更新*/
    q0 += 0.5f * (-q1*gx - q2*gy - q3*gz) * dt;
    q1 += 0.5f * ( q0*gx - q3*gy + q2*gz) * dt;
    q2 += 0.5f * ( q3*gx + q0*gy - q1*gz) * dt;
    q3 += 0.5f * (-q2*gx + q1*gy + q0*gz) * dt;
    
	/*四元数归一化*/
    norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 /= norm;
    q1 /= norm;
    q2 /= norm;
    q3 /= norm;
	
	/*四元数转欧拉角*/
    *roll = atan2f(2.0f*(q2*q3 + q0*q1), 1.0f - 2.0f*(q1*q1 + q2*q2)) * RAD_TO_DEG;
	*pitch = asinf(2.0f * (q0*q2 - q1*q3)) * RAD_TO_DEG;
    *yaw = atan2f(2.0f*(q1*q2 + q0*q3), 1.0f - 2.0f*(q2*q2 + q3*q3)) * RAD_TO_DEG;
}




