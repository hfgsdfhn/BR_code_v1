
#ifndef KALMAN_H
#define KALMAN_H

#include "arm_math.h"
#include <stdint.h>

typedef struct
{
    float32_t *x_data;
    float32_t *P_data;

    float32_t *A_data;
    float32_t *B_data;
    float32_t *u_data;
    float32_t *Q_data;

    float32_t *H_data;
    float32_t *z_data;
    float32_t *R_data;

    float32_t *K_data;

    float32_t *workspace;

    uint32_t workspace_size;

} kalman_data_t;

typedef struct
{
    uint16_t state_dim;
    uint16_t control_dim;
    uint16_t measure_dim;

    arm_matrix_instance_f32 x;
    arm_matrix_instance_f32 P;

    arm_matrix_instance_f32 A;
    arm_matrix_instance_f32 B;
    arm_matrix_instance_f32 u;
    arm_matrix_instance_f32 Q;

    arm_matrix_instance_f32 H;
    arm_matrix_instance_f32 z;
    arm_matrix_instance_f32 R;

    arm_matrix_instance_f32 K;


    arm_matrix_instance_f32 temp_n1_1;
    arm_matrix_instance_f32 temp_n1_2;

    arm_matrix_instance_f32 temp_z1;

    arm_matrix_instance_f32 temp_nn_1;
    arm_matrix_instance_f32 temp_nn_2;
    arm_matrix_instance_f32 temp_nn_3;

    arm_matrix_instance_f32 temp_nz;

    arm_matrix_instance_f32 temp_zn;

    arm_matrix_instance_f32 temp_zz_1;
    arm_matrix_instance_f32 temp_zz_2;

    kalman_data_t data;

} kalman_t;

void kalman_init(kalman_t *kalman);

void kalman_predict(kalman_t *kalman);

void kalman_update(kalman_t *kalman);

#endif
