/**
 * 通用线性 Kalman 滤波器
 *
 * 系统模型：
 *
 *     x(k) = A * x(k-1) + B * u(k) + w(k)
 *     z(k) = H * x(k) + v(k)
 *
 * 预测：
 *
 *     x_predict = A * x + B * u
 *
 *     P_predict = A * P * A^T + Q
 *
 * 更新：
 *
 *     K = P_predict * H^T *
 *         inverse(H * P_predict * H^T + R)
 *
 *     x = x_predict + K * (z - H * x_predict)
 *
 *     P = (I - K * H) * P_predict * (I - K * H)^T
 *         + K * R * K^T
 *
 * 矩阵说明：
 *
 *     x       : 状态向量                  (n × 1)
 *     A       : 状态转移矩阵              (n × n)
 *     B       : 控制输入矩阵              (n × l)
 *     u       : 控制输入向量              (l × 1)
 *     H       : 测量矩阵                  (m × n)
 *     z       : 传感器测量向量            (m × 1)
 *     P       : 状态估计误差协方差        (n × n)
 *     Q       : 过程噪声协方差            (n × n)
 *     R       : 測量噪声协方差            (m × m)
 *     K       : Kalman 增益               (n × m)
 *     I       : 单位矩阵                  (n × n)
 *
 *     w       : 过程噪声                  (n × 1)
 *     v       : 测量噪声                  (m × 1)
 *
 * n : 状态维度
 * m : 测量维度
 * l : 控制输入维度
 */

#ifndef KALMAN_H
#define KALMAN_H

#include "arm_math.h"
#include <stdint.h>

/**
 * @brief 卡尔曼滤波器矩阵数据缓存
 */
typedef struct
{
    /* 状态变量 */
    float32_t *x_data;
    float32_t *P_data;

    /* 状态转移模型 */
    float32_t *A_data;
    float32_t *B_data;
    float32_t *u_data;
    float32_t *Q_data;

    /* 观测模型 */
    float32_t *H_data;
    float32_t *z_data;
    float32_t *R_data;

    /* 卡尔曼增益 */
    float32_t *K_data;

    /* 临时矩阵工作区 */
    float32_t *workspace;

    /* workspace 容量，单位：float32_t */
    uint32_t workspace_size;

} kalman_data_t;


/**
 * @brief 通用线性卡尔曼滤波器
 */
typedef struct
{
    /* 维度信息 */
    uint16_t state_dim;       // 状态维度 n
    uint16_t control_dim;     // 控制输入维度 l
    uint16_t measure_dim;     // 测量维度 m

    /* 状态变量 */
    arm_matrix_instance_f32 x;    // 状态向量                  (n × 1)
    arm_matrix_instance_f32 P;    // 状态估计误差协方差        (n × n)

    /* 状态转移模型 */
    arm_matrix_instance_f32 A;    // 状态转移矩阵              (n × n)
    arm_matrix_instance_f32 B;    // 控制输入矩阵              (n × l)
    arm_matrix_instance_f32 u;    // 控制输入向量              (l × 1)
    arm_matrix_instance_f32 Q;    // 过程噪声协方差            (n × n)

    /* 观测模型 */
    arm_matrix_instance_f32 H;    // 测量矩阵                  (m × n)
    arm_matrix_instance_f32 z;    // 传感器测量向量            (m × 1)
    arm_matrix_instance_f32 R;    // 测量噪声协方差            (m × m)

    /* 卡尔曼增益 */
    arm_matrix_instance_f32 K;    // Kalman 增益               (n × m)


    /* ================= 临时矩阵 ================= */

    /* N×1 临时向量 */
    arm_matrix_instance_f32 temp_n1_1;
    arm_matrix_instance_f32 temp_n1_2;

    /* Z×1 临时向量 */
    arm_matrix_instance_f32 temp_z1;

    /* N×N 临时矩阵 */
    arm_matrix_instance_f32 temp_nn_1;
    arm_matrix_instance_f32 temp_nn_2;
    arm_matrix_instance_f32 temp_nn_3;

    /* N×Z 临时矩阵 */
    arm_matrix_instance_f32 temp_nz;

    /* Z×N 临时矩阵 */
    arm_matrix_instance_f32 temp_zn;

    /* Z×Z 临时矩阵 */
    arm_matrix_instance_f32 temp_zz_1;
    arm_matrix_instance_f32 temp_zz_2;


    kalman_data_t data;

} kalman_t;


/**
 * @brief 初始化卡尔曼滤波器矩阵
 *
 * @param kalman 卡尔曼滤波器实例
 */
void kalman_init(kalman_t *kalman);


/**
 * @brief 卡尔曼滤波预测
 *
 * @param kalman 卡尔曼滤波器实例
 */
void kalman_predict(kalman_t *kalman);


/**
 * @brief 卡尔曼滤波观测更新
 *
 * @param kalman 卡尔曼滤波器实例
 */
void kalman_update(kalman_t *kalman);


#endif /* KALMAN_H */