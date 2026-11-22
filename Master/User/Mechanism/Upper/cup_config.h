
#ifndef BR_CUP_CONFIG_H
#define BR_CUP_CONFIG_H

#define BR_AUX_M3508_TRAJECTORY_TIME_S 0.0f
#define BR_M2006_CUP0_TRAJECTORY_TIME_S 0.0f
#define BR_M2006_CUP0_USE_QUINTIC 0U
#define BR_M2006_CUP1_TRAJECTORY_TIME_S 2.0f
#define BR_M2006_CUP1_ROTATION_TRAJECTORY_TIME_S 4.0f
#define BR_M2006_CUP1_USE_QUINTIC 1U

#define BR_M2006_CUP0_POSITION_KP 60.247779608f
#define BR_M2006_CUP0_POSITION_KI 60.359877560f
#define BR_M2006_CUP0_POSITION_KD 0.0f
#define BR_M2006_CUP0_SPEED_KP 2.0f
#define BR_M2006_CUP0_SPEED_KI 3.0f
#define BR_M2006_CUP0_SPEED_KD 0.0f

#define BR_M2006_CUP1_POSITION_KP 60.247779608f
#define BR_M2006_CUP1_POSITION_KI 60.359877560f
#define BR_M2006_CUP1_POSITION_KD 0.0f
#define BR_M2006_CUP1_SPEED_KP 2.0f
#define BR_M2006_CUP1_SPEED_KI 3.0f
#define BR_M2006_CUP1_SPEED_KD 0.0f


extern volatile float br_m2006_cup0_position_kp;
extern volatile float br_m2006_cup0_position_ki;
extern volatile float br_m2006_cup0_position_kd;
extern volatile float br_m2006_cup0_speed_kp;
extern volatile float br_m2006_cup0_speed_ki;
extern volatile float br_m2006_cup0_speed_kd;

extern volatile float br_m2006_cup1_position_kp;
extern volatile float br_m2006_cup1_position_ki;
extern volatile float br_m2006_cup1_position_kd;
extern volatile float br_m2006_cup1_speed_kp;
extern volatile float br_m2006_cup1_speed_ki;
extern volatile float br_m2006_cup1_speed_kd;

#endif
