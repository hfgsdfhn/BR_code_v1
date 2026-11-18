
#ifndef CHASSIS_MPC_CTRL_H
#define CHASSIS_MPC_CTRL_H

#include "stdint.h"
#include "arm_math.h"
#include "mpc_qpOASES.h"

#define mpc_dt 0.01f
#define MPC_MAX_N 50
#define NX   6
#define NU   3
#define NV   (MPC_MAX_N*NU)

typedef struct MPC mpc_t;

typedef struct {

    float mass;
    float inertia_z;
    float max_force_x;
    float max_force_y;
    float max_torque_z;

} MpcPhysicalParams;

typedef enum {
    x, y, yaw, vx, vy, w
} mpc_state_t;
typedef enum
{
    Fx,Fy,Mz
} mpc_ctrl;
struct MPC
{
    mpc_state_t chassis_state;

    float x[NX];
    float u[NU];

    float Q[NX];
    float R[NU];

     uint8_t nWSR;

    MpcPhysicalParams RobotParam;
    uint8_t i;
};
void MPC_Init(void);
void MPC_g_figure(const float32_t *x0, const float32_t *X_ref, float32_t *g_data);
void MPC_Solve(const float32_t *x0, const float32_t *X_ref);

const float32_t *MPC_GetH(void);
const float32_t *MPC_GetLb(void);
const float32_t *MPC_GetUb(void);

#endif
