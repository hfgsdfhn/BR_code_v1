
#include "Chassis_MPC_Ctrl.h"
#include <stdlib.h>
#include <string.h>

extern int MPC_QPOASES_Hotstart(const float *g, const float *lb, const float *ub, float *x);

static mpc_t mpc =
{
    .Q = {100.0f, 100.0f, 50.0f, 1.0f, 1.0f, 5.0f},
    .R = {1.0f, 1.0f, 10.0f},
    .RobotParam =
    {
        .mass = 30.0f,
        .inertia_z = 5.3f,
        .max_force_x = 50.0f,
        .max_force_y = 50.0f,
        .max_torque_z = 10.0f
    },
};

static float32_t H_data[NV * NV];
static arm_matrix_instance_f32 H_mat;

static float32_t A_data[NX * NX];

static float32_t AB_data[MPC_MAX_N * NX * NU];

static float32_t row_cache[NV];

static float32_t lb[NV];
static float32_t ub[NV];

void MPC_Init(void)
{
    arm_mat_init_f32(&H_mat, NV, NV, H_data);

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


    memcpy(AB_data, B_data, sizeof(B_data));
    for (uint16_t d = 1; d < MPC_MAX_N; d++)
    {
        arm_matrix_instance_f32 prev_mat, cur_mat;
        arm_mat_init_f32(&prev_mat, NX, NU, &AB_data[(uint32_t)(d - 1) * (NX * NU)]);
        arm_mat_init_f32(&cur_mat,  NX, NU, &AB_data[(uint32_t)d * (NX * NU)]);
        arm_mat_mult_f32(&A_mat, &prev_mat, &cur_mat);
    }


    memset(H_data, 0, sizeof(H_data));

    for (uint16_t i = 0; i < MPC_MAX_N; i++)
    {
        uint16_t len = (uint16_t)((i + 1) * NU);

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

    for (uint16_t a = 0; a < NV; a++) {
        H_data[(uint32_t)a * NV + a] += 2.0f * mpc.R[a % NU];
    }

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

void MPC_g_figure(const float32_t *x0, const float32_t *X_ref, float32_t *g_data)
{
    float32_t e_all[MPC_MAX_N * NX];
    float32_t x_pred[NX];
    float32_t lambda[NX] = {0};

    for(int i=0; i<NX; i++) x_pred[i] = x0[i];

    for (int k = 0; k < MPC_MAX_N; k++) {
        float32_t x_next[NX] = {0};
        for(int i = 0; i < NX; i++){
            for(int j = 0; j < NX; j++){
                x_next[i] += A_data[i * NX + j] * x_pred[j];
            }
        }
        for(int i=0; i<NX; i++) x_pred[i] = x_next[i];

        for (int i = 0; i < NX; i++) {
            e_all[k * NX + i] = x_pred[i] - X_ref[k * NX + i];
        }
    }

    float m_val = (mpc.RobotParam.mass > 0.001f) ? mpc.RobotParam.mass : 1.0f;
    float Iz_val = (mpc.RobotParam.inertia_z > 0.001f) ? mpc.RobotParam.inertia_z : 1.0f;

    float b_fx = mpc_dt / m_val;
    float b_fy = mpc_dt / m_val;
    float b_mz = mpc_dt / Iz_val;

    for (int k = MPC_MAX_N - 1; k >= 0; k--) {

        float32_t A_T_lambda[NX] = {0};
        for(int i = 0; i < NX; i++){
            for(int j = 0; j < NX; j++){
                A_T_lambda[i] += A_data[j * NX + i] * lambda[j];
            }
        }

        for(int i = 0; i < NX; i++){
            lambda[i] = A_T_lambda[i] + 2.0f * mpc.Q[i] * e_all[k * NX + i];
        }

        g_data[k * NU + 0] = b_fx * lambda[3];
        g_data[k * NU + 1] = b_fy * lambda[4];
        g_data[k * NU + 2] = b_mz * lambda[5];
    }
}

void MPC_Solve(const float32_t *x0, const float32_t *X_ref)
{

    static float32_t g_data[NV];
    static float32_t x_opt[NV];

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
        status = MPC_QPOASES_Hotstart(g_data, lb, ub, x_opt);
    }

    if (status == 0 || status == 1)
    {
        mpc.u[0] = x_opt[0];
        mpc.u[1] = x_opt[1];
        mpc.u[2] = x_opt[2];
    }
    else
    {

        mpc.u[0] = 0.0f;
        mpc.u[1] = 0.0f;
        mpc.u[2] = 0.0f;
    }
}

const float32_t *MPC_GetH(void)  { return H_data; }
const float32_t *MPC_GetLb(void) { return lb; }
const float32_t *MPC_GetUb(void) { return ub; }
