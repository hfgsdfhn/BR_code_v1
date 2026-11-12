/*
 * @file kalman.c
 * @brief kalman模块的用户接口与实现。
 */

#include "kalman.h"


/**
 * @brief 卡尔曼滤波器初始化
 * @param kalman 卡尔曼滤波器实例
 */
void kalman_init(kalman_t *kalman)
{
    uint16_t N = kalman->state_dim;
    uint16_t M = kalman->control_dim;
    uint16_t Z = kalman->measure_dim;

    float32_t *workspace = kalman->data.workspace;

    /* 状态变量 */
    arm_mat_init_f32(&kalman->x, N, 1, kalman->data.x_data);
    arm_mat_init_f32(&kalman->P, N, N, kalman->data.P_data);

    /* 状态转移模型 */
    arm_mat_init_f32(&kalman->A, N, N, kalman->data.A_data);
    arm_mat_init_f32(&kalman->B, N, M, kalman->data.B_data);
    arm_mat_init_f32(&kalman->u, M, 1, kalman->data.u_data);
    arm_mat_init_f32(&kalman->Q, N, N, kalman->data.Q_data);

    /* 观测模型 */
    arm_mat_init_f32(&kalman->H, Z, N, kalman->data.H_data);
    arm_mat_init_f32(&kalman->z, Z, 1, kalman->data.z_data);
    arm_mat_init_f32(&kalman->R, Z, Z, kalman->data.R_data);

    /* 卡尔曼增益 */
    arm_mat_init_f32(&kalman->K, N, Z, kalman->data.K_data);

    /* 临时矩阵 */
    arm_mat_init_f32(&kalman->temp_n1_1, N, 1, workspace);  workspace += N;
    arm_mat_init_f32(&kalman->temp_n1_2, N, 1, workspace);  workspace += N;
    arm_mat_init_f32(&kalman->temp_z1, Z, 1, workspace);    workspace += Z;
    arm_mat_init_f32(&kalman->temp_nn_1, N, N, workspace);  workspace += N * N;
    arm_mat_init_f32(&kalman->temp_nn_2, N, N, workspace);  workspace += N * N;
    arm_mat_init_f32(&kalman->temp_nn_3, N, N, workspace);  workspace += N * N;
    arm_mat_init_f32(&kalman->temp_nz, N, Z, workspace);    workspace += N * Z;
    arm_mat_init_f32(&kalman->temp_zn, Z, N, workspace);    workspace += Z * N;
    arm_mat_init_f32(&kalman->temp_zz_1, Z, Z, workspace);  workspace += Z * Z;
    arm_mat_init_f32(&kalman->temp_zz_2, Z, Z, workspace);
}


/**
 * @brief 卡尔曼滤波预测
 *
 * 状态预测：
 * x = A * x + B * u
 *
 * 协方差预测：
 * P = A * P * A^T + Q
 *
 * @param kalman 卡尔曼滤波器实例
 */
void kalman_predict(kalman_t *kalman)
{
    /* x = A * x + B * u */
    arm_mat_mult_f32(&kalman->A, &kalman->x, &kalman->temp_n1_1);
    arm_mat_mult_f32(&kalman->B, &kalman->u, &kalman->temp_n1_2);
    arm_mat_add_f32(&kalman->temp_n1_1, &kalman->temp_n1_2, &kalman->x);

    /* P = A * P * A^T + Q */
    arm_mat_mult_f32(&kalman->A, &kalman->P, &kalman->temp_nn_1);
    arm_mat_trans_f32(&kalman->A, &kalman->temp_nn_2);
    arm_mat_mult_f32(&kalman->temp_nn_1, &kalman->temp_nn_2, &kalman->P);
    arm_mat_add_f32(&kalman->P, &kalman->Q, &kalman->P);
}

/**
 * @brief 卡尔曼滤波观测更新
 *
 * @param kalman 卡尔曼滤波器实例
 */
void kalman_update(kalman_t *kalman)
{
    uint16_t i;
    uint16_t j;
    uint16_t N = kalman->state_dim;

    /* 模块实现细节。 */

    /* H^T */
    arm_mat_trans_f32(&kalman->H, &kalman->temp_zn);
    /* P * H^T */
    arm_mat_mult_f32(&kalman->P, &kalman->temp_zn, &kalman->temp_nz);
    /* S = H * P * H^T + R */
    arm_mat_mult_f32(&kalman->H, &kalman->temp_nz, &kalman->temp_zz_1);
    arm_mat_add_f32(&kalman->temp_zz_1, &kalman->R, &kalman->temp_zz_1);
    /* S^-1 */
    arm_mat_inverse_f32(&kalman->temp_zz_1, &kalman->temp_zz_2);
    /* K = P * H^T * S^-1 */
    arm_mat_mult_f32(&kalman->temp_nz, &kalman->temp_zz_2, &kalman->K);

    /*
     * x = x + K * (z - H * x)
     */

    /* H * x */
    arm_mat_mult_f32(&kalman->H, &kalman->x, &kalman->temp_z1);
    /* z - H * x */
    arm_mat_sub_f32(&kalman->z, &kalman->temp_z1, &kalman->temp_z1);
    /* K * (z - H * x) */
    arm_mat_mult_f32(&kalman->K, &kalman->temp_z1, &kalman->temp_n1_1);
    /* x = x + K * (z - H * x) */
    arm_mat_add_f32(&kalman->x, &kalman->temp_n1_1, &kalman->x);


    /* 模块实现细节。 */

    /* K * H */
    arm_mat_mult_f32(&kalman->K, &kalman->H, &kalman->temp_nn_1);
    /* 生成单位矩阵 I */
    for (i = 0; i < N; i++)
    {
        for (j = 0; j < N; j++)
        {
            kalman->temp_nn_2.pData[i * N + j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    /* I - K * H */
    arm_mat_sub_f32(&kalman->temp_nn_2, &kalman->temp_nn_1, &kalman->temp_nn_2);
    /* (I - K * H) * P */
    arm_mat_mult_f32(&kalman->temp_nn_2, &kalman->P, &kalman->temp_nn_1);
    /* (I - K * H)^T */
    arm_mat_trans_f32(&kalman->temp_nn_2, &kalman->temp_nn_3);
    /* (I - K * H) * P * (I - K * H)^T */
    arm_mat_mult_f32(&kalman->temp_nn_1, &kalman->temp_nn_3, &kalman->P);
    /* K * R */
    arm_mat_mult_f32(&kalman->K, &kalman->R, &kalman->temp_nz);
    /* K^T */
    arm_mat_trans_f32(&kalman->K, &kalman->temp_zn);
    /* K * R * K^T */
    arm_mat_mult_f32(&kalman->temp_nz, &kalman->temp_zn, &kalman->temp_nn_1);
    /* P = P + K * R * K^T */
    arm_mat_add_f32(&kalman->P, &kalman->temp_nn_1, &kalman->P);
}