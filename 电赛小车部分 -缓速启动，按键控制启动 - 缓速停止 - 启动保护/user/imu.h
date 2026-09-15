#ifndef __IMU_H
#define __IMU_H

#define RAD_TO_DEG (180.0f/3.1415926535f)
#define DEG_TO_RAD (3.1415926535f/180.0f)

void IMU_InitQuaternion(float ax, float ay, float az);

void IMU_3Axis_Acc(float ax, float ay, float az, float *roll, float *pitch);
void IMU_3Axis_Gyro(float gx, float gy, float gz, float *roll, float *pitch, float *yaw, float dt);
void IMU_6Axis_Gyro_Acc(float ax, float ay, float az, 
						float gx, float gy, float gz, 
						float *roll, float *pitch, float *yaw, float dt);
void IMU_6Axis_Gyro_Acc_v(float ax, float ay, float az, 
							 float gx, float gy, float gz, 
							 float *roll, float *pitch, float *yaw, float *a_down, float dt);
void IMU_6Axis_Gyro_Acc_3v(float ax, float ay, float az, 
							 float gx, float gy, float gz, 
							 float *roll, float *pitch, float *yaw,
							 float *a_forward, float *a_right, float *a_down, float dt);
void IMU_9Axis_Gyro_Acc_Mag(float ax, float ay, float az, 
							float gx, float gy, float gz, 
							float mx, float my, float mz, 
							float *roll, float *pitch, float *yaw, float dt);


#endif
