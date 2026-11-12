




#ifndef PID_H
#define PID_H

#include <stdbool.h>


typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float previous_error;
    float output_limit;
    float integral_limit;
    float full_integral_separation;
    float integral_separation;
    bool initialized;
} pid_t;


bool PID_Init(pid_t *pid                    , float kp            , float ki            ,
              float kd            , float output_limit               ,
              float integral_limit                );


void PID_SetIntegralSeparationRange(pid_t *pid                   ,
                                    float full_separation                   ,
                                    float separation                   );


void PID_Reset(pid_t *pid                     );


float PID_Calc(pid_t *pid                      , float target             ,
               float feedback             , float dt_s                 );

#endif
