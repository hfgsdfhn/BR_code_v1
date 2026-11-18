
#ifndef MPC_QPOASES_H
#define MPC_QPOASES_H

#ifdef __cplusplus
extern "C" {
#endif

#define MPC_QPOASES_MAX_NV 192

int MPC_QPOASES_Init(int nV);

int MPC_QPOASES_Solve(const float *H,
                      const float *g,
                      const float *lb,
                      const float *ub,
                      float *x);

int MPC_QPOASES_Hotstart(const float *g,
                         const float *lb,
                         const float *ub,
                         float *x);

int MPC_QPOASES_GetLastStatus(void);

int MPC_QPOASES_GetLastNWSR(void);

float MPC_QPOASES_GetLastObj(void);

void MPC_QPOASES_Cleanup(void);

#ifdef __cplusplus
}
#endif

#endif
