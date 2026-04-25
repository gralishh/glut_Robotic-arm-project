#ifndef __GENERAL_MOVEMENT__
#define __GENERAL_MOVEMENT__

#include "main.h"

typedef enum
{
  NON = 0,
  BTD,
  ONCE_CLICK_GET_MINE,
  ONCE_CLICK_SAVE_MINE,
} movement_t;

typedef enum
{
  NON_DEIR = 0,
  LEFT,
  RIGHT,
} mine_place_t;

/*get_storage_mine*/
typedef enum
{
  GSM_uplift_to_higher = 0,
  GSM_hand_to_pos1,
  GSM_hand_to_pos2,
  GSM_hand_to_pos3,
  GSM_hand_gri_close,
  GSM_uplift_to_up,
  GSM_complete,
} get_storage_mine_t;
#define GSM_STEP1_HEIGHT 64.0f

#define GSM_STEP1_G_ANGLE 0.0f
#define GSM_STEP1_J5_ANGLE 0.0f
#define GSM_STEP1_J4_ANGLE 93.0f
#define GSM_STEP1_J3_ANGLE 1.564f

#define GSM_STEP2_J2_ANGLE_RIGHT -1.896f
#define GSM_STEP2_J1_ANGLE_RIGHT 0.306f
#define GSM_STEP2_J2_ANGLE_LEFT 1.92f
#define GSM_STEP2_J1_ANGLE_LEFT  2.827f

#define GSM_STEP3_J2_ANGLE_RIGHT -1.709f
#define GSM_STEP3_J1_ANGLE_RIGHT 0.145f
#define GSM_STEP3_J2_ANGLE_LEFT 1.721f
#define GSM_STEP3_J1_ANGLE_LEFT 2.99f

#define GSM_STEP4_J2_ANGLE_RIGHT -1.549f
#define GSM_STEP4_J1_ANGLE_RIGHT 0.0f
#define GSM_STEP4_J2_ANGLE_LEFT 1.579f
#define GSM_STEP4_J1_ANGLE_LEFT 3.14f

#define GSM_STEP5_G_ANGLE 0.62f

#define GSM_STEP6_HEIGHT 180.0f
//#define GGM_CPLT_HEIGHT 30

/*save_mine_t*/
typedef enum
{
  SM_uplift_to_pos = 0,
  SM_hand_to_pos,
  SM_uplift_to_pos2,
  SM_hand_gri_open,
  SM_hand_out,
  SM_complete,
} save_mine_t;
#define SM_STEP1_HEIGHT 179.5f
#define SM_STEP1_G_ANGLE 0.62f
#define SM_STEP1_J5_ANGLE 0.0f
#define SM_STEP1_J4_ANGLE 93.0f
#define SM_STEP1_J3_ANGLE 1.564f

#define SM_STEP2_J2_ANGLE_RIGHT -1.593f
#define SM_STEP2_J1_ANGLE_RIGHT 0.0f
#define SM_STEP2_J2_ANGLE_LEFT 1.593f
#define SM_STEP2_J1_ANGLE_LEFT 3.14f

#define SM_STEP3_HEIGHT 64

#define SM_STEP4_JG_ANGLE 0.0f

#define SM_STEP5_J1_ANGLE_RIGHT 0.413f
#define SM_STEP5_J2_ANGLE_RIGHT -1.922f
#define SM_STEP5_J1_ANGLE_LEFT 2.727f
#define SM_STEP5_J2_ANGLE_LEFT 1.922f
// #define SM_STEP4_DELAY_LOOP 500
// #define SM_CPLT_HEIGHT 275

typedef enum
{
  BTD_hand_to_pos = 0,
  BTD_uplift_to_pos,
  BTD_complete,
} back_to_default_t;

#define BTD_STEP1_G_ANGLE 0.62f
#define BTD_STEP1_J5_ANGLE 0.0f
#define BTD_STEP1_J4_ANGLE 10.0f
#define BTD_STEP1_J3_ANGLE 0.0f

#define BTD_STEP2_HEIGHT 6.0f
#define BTD_STEP2_J2_ANGLE -1.812f
#define BTD_STEP2_J1_ANGLE 0.251f
typedef struct
{
  movement_t movement_type;
  mine_place_t mine_place;
  uint8_t movement_step;

  uint8_t movement_mode;

  uint8_t height_step_complete;
  uint8_t hand_step_complete;

} movement_handler_t;

extern movement_handler_t movement;

//uint8_t get_out_flag(void);
void set_movement(movement_t movement);
uint8_t get_movement(void);
uint8_t get_step(void);
void next_step(void);

#endif
