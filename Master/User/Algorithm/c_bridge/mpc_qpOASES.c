/*
 * @file mpc_qpOASES.c
 * @brief mpc_qpOASES模块的用户接口与实现。
 */

/**
 * @file    mpc_qpOASES.c
 * @brief   C adapter for qpOASES 3.2 (QProblemB, single precision).
 *
 * Important:
 *   - qpOASES_wrapper.h has no "extern C" guard and selects real_t from
 *     __USE_SINGLE_PRECISION__ itself.  The macro MUST be defined in every
 *     C translation unit that includes the wrapper, otherwise the C side
 *     thinks real_t is double while the C++ side uses float.
 *   - The qpOASES copy in C:\Users\HP\Desktop\qpOASES already has
 *     __USE_SINGLE_PRECISION__ enabled in include/qpOASES/Types.hpp.
 */
#define __USE_SINGLE_PRECISION__
#include "qpOASES_wrapper.h"

#include "mpc_qpOASES.h"

typedef char mpc_qpoases_real_t_must_be_float[
    (sizeof(real_t) == sizeof(float)) ? 1 : -1];

static int_t  s_nV     = 0;
static int_t  s_status = -1;
static int_t  s_nWSR   = 0;
static real_t s_obj    = 0.0f;
static real_t s_y[MPC_QPOASES_MAX_NV];
static qpOASES_Options s_options;

#define MPC_QPOASES_WANT_OBJ   0

#if MPC_QPOASES_WANT_OBJ
#define MPC_QPOASES_OBJ_PTR    (&s_obj)
#else
#define MPC_QPOASES_OBJ_PTR    ((real_t *)0)
#endif

/* 功能：初始化MPC_QPOASES_Init对应的外设、参数和运行状态。 */
int MPC_QPOASES_Init(int nV)
{
    if ((nV < 1) || (nV > MPC_QPOASES_MAX_NV))
        return -1;

    if (QProblemB_setup((int_t)nV, HST_UNKNOWN) != 0)
        return -2;

    if (qpOASES_Options_init(&s_options, 2) != 0)
        return -3;

    s_options.printLevel = PL_NONE;
    s_nV     = (int_t)nV;
    s_status = -1;
    s_nWSR   = 0;
    s_obj    = 0.0f;

    return 0;
}

/* 功能：计算MPC_QPOASES_Solve对应的控制量或规划结果。 */
int MPC_QPOASES_Solve(const float *H,
                      const float *g,
                      const float *lb,
                      const float *ub,
                      float *x)
{
    int_t nWSR;
    int_t ret;

    if ((s_nV < 1) || (H == 0) || (g == 0) || (lb == 0) || (ub == 0) || (x == 0))
        return -1;

    nWSR = 200;

    ret = QProblemB_init((const real_t *)H,
                         (const real_t *)g,
                         (const real_t *)lb,
                         (const real_t *)ub,
                         &nWSR,
                         0,
                         &s_options,
                         (real_t *)x,
                         s_y,
                         MPC_QPOASES_OBJ_PTR,
                         &s_status);

    s_nWSR = nWSR;

    if (ret != 0)
        return (int)ret;

    return (int)s_status;
}

/* 功能：使用上一周期结果热启动 qpOASES 求解器。 */
int MPC_QPOASES_Hotstart(const float *g,
                         const float *lb,
                         const float *ub,
                         float *x)
{
    int_t nWSR;
    int_t ret;

    if ((s_nV < 1) || (g == 0) || (lb == 0) || (ub == 0) || (x == 0))
        return -1;

    nWSR = 200;

    ret = QProblemB_hotstart((const real_t *)g,
                             (const real_t *)lb,
                             (const real_t *)ub,
                             &nWSR,
                             0,
                             (real_t *)x,
                             s_y,
                             MPC_QPOASES_OBJ_PTR,
                             &s_status);

    s_nWSR = nWSR;

    if (ret != 0)
        return (int)ret;

    return (int)s_status;
}

/* 功能：读取MPC_QPOASES_GetLastStatus对应的数据或运行状态。 */
int MPC_QPOASES_GetLastStatus(void)
{
    return (int)s_status;
}

/* 功能：读取MPC_QPOASES_GetLastNWSR对应的数据或运行状态。 */
int MPC_QPOASES_GetLastNWSR(void)
{
    return (int)s_nWSR;
}

/* 功能：读取MPC_QPOASES_GetLastObj对应的数据或运行状态。 */
float MPC_QPOASES_GetLastObj(void)
{
    return (float)s_obj;
}

/* 功能：重置或释放MPC_QPOASES_Cleanup对应的内部状态和资源。 */
void MPC_QPOASES_Cleanup(void)
{
    QProblemB_cleanup();
    s_nV     = 0;
    s_status = -1;
    s_nWSR   = 0;
    s_obj    = 0.0f;
}
