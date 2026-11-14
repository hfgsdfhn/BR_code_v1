/*
 * @file mpc_qpOASES.h
 * @brief mpc_qpOASES模块的用户接口与实现。
 */

/**
 * @file    mpc_qpOASES.h
 * @brief   Thin C adapter between the chassis MPC code and qpOASES' C API.
 *
 * The qpOASES C interface is NOT a pre-compiled library.  It is
 * qpOASES_wrapper.cpp + the C++ qpOASES sources.  This adapter hides that
 * C++ interface from the rest of the project and fixes real_t to float.
 *
 * Add mpc_qpOASES.c + qpOASES_wrapper.cpp to the Keil project, then use only
 * the functions below from plain C.
 */
#ifndef MPC_QPOASES_H
#define MPC_QPOASES_H

#ifdef __cplusplus
extern "C" {
#endif

#define MPC_QPOASES_MAX_NV 192

/* 功能：初始化MPC_QPOASES_Init对应的外设、参数和运行状态。 */
int MPC_QPOASES_Init(int nV);

/* 功能：计算MPC_QPOASES_Solve对应的控制量或规划结果。 */
int MPC_QPOASES_Solve(const float *H,
                      const float *g,
                      const float *lb,
                      const float *ub,
                      float *x);

/* 功能：使用上一周期结果热启动 qpOASES 求解器。 */
int MPC_QPOASES_Hotstart(const float *g,
                         const float *lb,
                         const float *ub,
                         float *x);

/* 功能：读取MPC_QPOASES_GetLastStatus对应的数据或运行状态。 */
int MPC_QPOASES_GetLastStatus(void);

/* 功能：读取MPC_QPOASES_GetLastNWSR对应的数据或运行状态。 */
int MPC_QPOASES_GetLastNWSR(void);

/* 功能：读取MPC_QPOASES_GetLastObj对应的数据或运行状态。 */
float MPC_QPOASES_GetLastObj(void);

/* 功能：重置或释放MPC_QPOASES_Cleanup对应的内部状态和资源。 */
void MPC_QPOASES_Cleanup(void);

#ifdef __cplusplus
}
#endif

#endif
