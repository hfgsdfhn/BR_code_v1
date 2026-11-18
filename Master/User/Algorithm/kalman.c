
#include "kalman.h"

void kalman_init(kalman_t *kalman)
{
    uint16_t N = kalman->state_dim;
    uint16_t M = kalman->control_dim;
    uint16_t Z = kalman->measure_dim;

    float32_t *workspace = kalman->data.workspace;

    arm_mat_init_f32(&kalman->x, N, 1, kalman->data.x_data);
    arm_mat_init_f32(&kalman->P, N, N, kalman->data.P_data);

    arm_mat_init_f32(&kalman->A, N, N, kalman->data.A_data);
    arm_mat_init_f32(&kalman->B, N, M, kalman->data.B_data);
    arm_mat_init_f32(&kalman->u, M, 1, kalman->data.u_data);
    arm_mat_init_f32(&kalman->Q, N, N, kalman->data.Q_data);

    arm_mat_init_f32(&kalman->H, Z, N, kalman->data.H_data);
    arm_mat_init_f32(&kalman->z, Z, 1, kalman->data.z_data);
    arm_mat_init_f32(&kalman->R, Z, Z, kalman->data.R_data);

    arm_mat_init_f32(&kalman->K, N, Z, kalman->data.K_data);

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

void kalman_predict(kalman_t *kalman)
{
    arm_mat_mult_f32(&kalman->A, &kalman->x, &kalman->temp_n1_1);
    arm_mat_mult_f32(&kalman->B, &kalman->u, &kalman->temp_n1_2);
    arm_mat_add_f32(&kalman->temp_n1_1, &kalman->temp_n1_2, &kalman->x);

    arm_mat_mult_f32(&kalman->A, &kalman->P, &kalman->temp_nn_1);
    arm_mat_trans_f32(&kalman->A, &kalman->temp_nn_2);
    arm_mat_mult_f32(&kalman->temp_nn_1, &kalman->temp_nn_2, &kalman->P);
    arm_mat_add_f32(&kalman->P, &kalman->Q, &kalman->P);
}

void kalman_update(kalman_t *kalman)
{
    uint16_t i;
    uint16_t j;
    uint16_t N = kalman->state_dim;

    arm_mat_trans_f32(&kalman->H, &kalman->temp_zn);
    arm_mat_mult_f32(&kalman->P, &kalman->temp_zn, &kalman->temp_nz);
    arm_mat_mult_f32(&kalman->H, &kalman->temp_nz, &kalman->temp_zz_1);
    arm_mat_add_f32(&kalman->temp_zz_1, &kalman->R, &kalman->temp_zz_1);
    arm_mat_inverse_f32(&kalman->temp_zz_1, &kalman->temp_zz_2);
    arm_mat_mult_f32(&kalman->temp_nz, &kalman->temp_zz_2, &kalman->K);


    arm_mat_mult_f32(&kalman->H, &kalman->x, &kalman->temp_z1);
    arm_mat_sub_f32(&kalman->z, &kalman->temp_z1, &kalman->temp_z1);
    arm_mat_mult_f32(&kalman->K, &kalman->temp_z1, &kalman->temp_n1_1);
    arm_mat_add_f32(&kalman->x, &kalman->temp_n1_1, &kalman->x);

    arm_mat_mult_f32(&kalman->K, &kalman->H, &kalman->temp_nn_1);
    for (i = 0; i < N; i++)
    {
        for (j = 0; j < N; j++)
        {
            kalman->temp_nn_2.pData[i * N + j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    arm_mat_sub_f32(&kalman->temp_nn_2, &kalman->temp_nn_1, &kalman->temp_nn_2);
    arm_mat_mult_f32(&kalman->temp_nn_2, &kalman->P, &kalman->temp_nn_1);
    arm_mat_trans_f32(&kalman->temp_nn_2, &kalman->temp_nn_3);
    arm_mat_mult_f32(&kalman->temp_nn_1, &kalman->temp_nn_3, &kalman->P);
    arm_mat_mult_f32(&kalman->K, &kalman->R, &kalman->temp_nz);
    arm_mat_trans_f32(&kalman->K, &kalman->temp_zn);
    arm_mat_mult_f32(&kalman->temp_nz, &kalman->temp_zn, &kalman->temp_nn_1);
    arm_mat_add_f32(&kalman->P, &kalman->temp_nn_1, &kalman->P);
}
