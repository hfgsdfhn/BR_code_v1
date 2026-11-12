/*
 * @file remote.h
 * @brief remote妯″潡鐨勭敤鎴锋帴鍙ｄ笌瀹炵幇銆? */

#ifndef HAND_REMOTE_H
#define HAND_REMOTE_H

#include <stdint.h>
#include "main.h"

#define REMOTE_FRAME_HEADER_0 0xA5U
#define REMOTE_FRAME_HEADER_1 0x5AU
#define REMOTE_FRAME_SIZE 8U
#define REMOTE_TX_PERIOD_MS 1U
#define REMOTE_KEY_DEBOUNCE_SAMPLES 5U
#define REMOTE_ADC_FILTER_SHIFT 2U
#define REMOTE_TX_QUEUE_DEPTH 4U

/* 鍔熻兘锛氬垵濮嬪寲妯″潡骞跺噯澶囧璁捐祫婧愩€?*/
void Remote_Init(void);

/* 鍔熻兘锛氭墽琛?Remote_Loop 妯″潡鎿嶄綔骞剁淮鎶ょ浉鍏崇姸鎬併€?*/
void Remote_Loop(void);

#endif


