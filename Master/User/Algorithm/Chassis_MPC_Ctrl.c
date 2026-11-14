/*
 * @file Chassis_MPC_Ctrl.c
 * @brief Chassis_MPC_Ctrl模块的用户接口与实现。
 */

#include "Chassis_MPC_Ctrl.h"
#include <stdlib.h>
#include <string.h>

/* 功能：使用上一周期结果热启动 qpOASES 求解器。 */
extern int MPC_QPOASES_Hotstart(const float *g, const float *lb, const float *ub, float *x);

static mpc_t mpc =
{
    .Q = {100.0f, 100.0f, 50.0f, 1.0f, 1.0f, 5.0f}, // 状态权重 [x, y, yaw, vx, vy, w]
    .R = {1.0f, 1.0f, 10.0f},                       // 控制权重 [Fx, Fy, Mz]
    .RobotParam =
    {
        .mass = 30.0f,
        .inertia_z = 5.3f,
        .max_force_x = 50.0f,
        .max_force_y = 50.0f,
        .max_torque_z = 10.0f
    },
};

// ==========================================
// 全局静态变量定义
// ==========================================
static float32_t H_data[NV * NV];
static arm_matrix_instance_f32 H_mat;

static float32_t A_data[NX * NX];

static float32_t AB_data[MPC_MAX_N * NX * NU];

// 累加 H 时用的一行缓存，长度 NV
static float32_t row_cache[NV];

//控制量上下界
static float32_t lb[NV];
static float32_t ub[NV];

//提前计算好H矩阵和约束边界
void MPC_Init(void)
{
    // ==========================================
    // 1. 初始化 H 矩阵描述符
    // ==========================================
    arm_mat_init_f32(&H_mat, NV, NV, H_data);

    // ==========================================
    // 2. 生成单步 A 和 B 矩阵
    // ==========================================
    float32_t B_data[NX * NU] = {0};
    arm_matrix_instance_f32 A_mat, B_mat;

    memset(A_data, 0, sizeof(A_data));
    for (int i = 0; i < NX; i++) {
        A_data[i * NX + i] = 1.0f;
    }
    for (int i = 0; i < NX / 2; i++) {
        A_data[i * NX + (i + NX / 2)] = mpc_dt;
    }
    arm_mat_init_f32(&A_mat, NX, NX, A_data);

    float m_val = (mpc.RobotParam.mass > 0.001f) ? mpc.RobotParam.mass : 1.0f;
    float Iz_val = (mpc.RobotParam.inertia_z > 0.001f) ? mpc.RobotParam.inertia_z : 1.0f;
    B_data[3*NU + 0] = mpc_dt / m_val;
    B_data[4*NU + 1] = mpc_dt / m_val;
    B_data[5*NU + 2] = mpc_dt / Iz_val;
    arm_mat_init_f32(&B_mat, NX, NU, B_data);

    // ==========================================

    // ==========================================
    memcpy(AB_data, B_data, sizeof(B_data));          // d = 0 就是 B
    for (uint16_t d = 1; d < MPC_MAX_N; d++)
    {
        arm_matrix_instance_f32 prev_mat, cur_mat;
        arm_mat_init_f32(&prev_mat, NX, NU, &AB_data[(uint32_t)(d - 1) * (NX * NU)]);
        arm_mat_init_f32(&cur_mat,  NX, NU, &AB_data[(uint32_t)d * (NX * NU)]);
        arm_mat_mult_f32(&A_mat, &prev_mat, &cur_mat);
    }

    // ==========================================

    // ==========================================
    memset(H_data, 0, sizeof(H_data));

    for (uint16_t i = 0; i < MPC_MAX_N; i++)
    {
        uint16_t len = (uint16_t)((i + 1) * NU);       // 这一行块的有效列数

        for (uint16_t r = 0; r < NX; r++)
        {
            float32_t q2 = 2.0f * mpc.Q[r];

            for (uint16_t j = 0; j <= i; j++)
            {
                const float32_t *src =
                    &AB_data[(uint32_t)(i - j) * (NX * NU) + (uint32_t)r * NU];
                for (uint16_t c = 0; c < NU; c++) {
                    row_cache[j * NU + c] = src[c];
                }
            }

            for (uint16_t a = 0; a < len; a++)
            {
                float32_t va = q2 * row_cache[a];
                float32_t *Hrow = &H_data[(uint32_t)a * NV];
                for (uint16_t b = 0; b < len; b++) {
                    Hrow[b] += va * row_cache[b];
                }
            }
        }
    }

    // 4.1 对角线上加 2R
    for (uint16_t a = 0; a < NV; a++) {
        H_data[(uint32_t)a * NV + a] += 2.0f * mpc.R[a % NU];
    }

    // ==========================================
    // 5. 构建控制量约束边界 lb 和 ub
    // ==========================================
    for (int k = 0; k < MPC_MAX_N; k++)
    {
        lb[k * NU + 0] = -mpc.RobotParam.max_force_x;
        ub[k * NU + 0] =  mpc.RobotParam.max_force_x;

        lb[k * NU + 1] = -mpc.RobotParam.max_force_y;
        ub[k * NU + 1] =  mpc.RobotParam.max_force_y;

        lb[k * NU + 2] = -mpc.RobotParam.max_torque_z;
        ub[k * NU + 2] =  mpc.RobotParam.max_torque_z;
    }
}

/* 功能：根据底盘模型计算 MPC 优化问题的梯度项。 */
void MPC_g_figure(const float32_t *x0, const float32_t *X_ref, float32_t *g_data)
{
    float32_t e_all[MPC_MAX_N * NX]; // 存放正向误差 (30 * 6 * 4 = 720字节)
    float32_t x_pred[NX];
    float32_t lambda[NX] = {0};      // 协态向量

    // 1. 正向预测：获取惯性下的误差序列
    for(int i=0; i<NX; i++) x_pred[i] = x0[i];

    for (int k = 0; k < MPC_MAX_N; k++) {
        // x_{k+1} = A * x_k (单步预测)
        float32_t x_next[NX] = {0};
        for(int i = 0; i < NX; i++){
            for(int j = 0; j < NX; j++){
                x_next[i] += A_data[i * NX + j] * x_pred[j];
            }
        }
        for(int i=0; i<NX; i++) x_pred[i] = x_next[i];

        // 计算误差 e_k = 预测状态 - 参考状态
        for (int i = 0; i < NX; i++) {
            e_all[k * NX + i] = x_pred[i] - X_ref[k * NX + i];
        }
    }

    float m_val = (mpc.RobotParam.mass > 0.001f) ? mpc.RobotParam.mass : 1.0f;
    float Iz_val = (mpc.RobotParam.inertia_z > 0.001f) ? mpc.RobotParam.inertia_z : 1.0f;

    float b_fx = mpc_dt / m_val;
    float b_fy = mpc_dt / m_val;
    float b_mz = mpc_dt / Iz_val;

    // 从第 N 步倒推到第 1 步
    for (int k = MPC_MAX_N - 1; k >= 0; k--) {

        float32_t A_T_lambda[NX] = {0};
        for(int i = 0; i < NX; i++){
            for(int j = 0; j < NX; j++){
                // A^T 的 (i,j) 其实就是 A 的 (j,i)
                A_T_lambda[i] += A_data[j * NX + i] * lambda[j];
            }
        }

        for(int i = 0; i < NX; i++){
            lambda[i] = A_T_lambda[i] + 2.0f * mpc.Q[i] * e_all[k * NX + i];
        }

        g_data[k * NU + 0] = b_fx * lambda[3]; // Fx 梯度
        g_data[k * NU + 1] = b_fy * lambda[4]; // Fy 梯度
        g_data[k * NU + 2] = b_mz * lambda[5]; // Mz 梯度
    }
}

/* 功能：计算MPC_Solve对应的控制量或规划结果。 */
void MPC_Solve(const float32_t *x0, const float32_t *X_ref)
{

    static float32_t g_data[NV];
    static float32_t x_opt[NV];

    // 算出当前梯度 g
    MPC_g_figure(x0, X_ref, g_data);

    static uint8_t s_cold_start = 1;
    int status;

    if (s_cold_start)
    {

        if (MPC_QPOASES_Init((int)NV) == 0)
        {
            status = MPC_QPOASES_Solve(H_data, g_data, lb, ub, x_opt);
            if ((status == 0) || (status == 1))
                s_cold_start = 0;
        }
        else
        {
            status = -1;
        }
    }
    else
    {
        /* 之后每个控制周期：热启动 */
        status = MPC_QPOASES_Hotstart(g_data, lb, ub, x_opt);
    }

    // 3. 检查求解结果
    // 0 = 成功，1 = 达到最大迭代次数但也得到了可行解
    if (status == 0 || status == 1)
    {
        // 只取第一步的控制量给电机
        mpc.u[0] = x_opt[0]; // Fx
        mpc.u[1] = x_opt[1]; // Fy
        mpc.u[2] = x_opt[2]; // Mz
    }
    else
    {

        mpc.u[0] = 0.0f;
        mpc.u[1] = 0.0f;
        mpc.u[2] = 0.0f;
    }
}

/* ------------------------------------------------------------------ */
/* 只读访问接口（测试 / 调试代码用）                                   */
/* ------------------------------------------------------------------ */
const float32_t *MPC_GetH(void)  { return H_data; }
const float32_t *MPC_GetLb(void) { return lb; }
const float32_t *MPC_GetUb(void) { return ub; }