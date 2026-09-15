#ifndef __KALMAN_H
#define __KALMAN_H

#include <stdint.h>


// 一维卡尔曼滤波器结构体
typedef struct {
    float x;      // 状态估计值（滤波输出）
    float P;      // 估计误差协方差
    float Q;      // 过程噪声协方差
    float R;      // 测量噪声协方差

    // ---- 防发散相关 ----
    uint8_t consecutive_invalid; // 连续无效测量计数
    uint8_t max_invalid;         // 触发复位的连续无效次数阈值，即累计收到
} Kalman1D_t;



void Kalman1D_Init(Kalman1D_t *kf, float init_x, float init_P, float Q, float R, uint8_t max_invalid);
float Kalman1D_UpdateWithCheck(Kalman1D_t *kf, float z, float min_valid, float max_valid);

#endif

