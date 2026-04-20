#ifndef __GENERAL_MOVEMENT__
#define __GENERAL_MOVEMENT__

#include "main.h"

typedef enum
{
  NON = 0,
  BTD,
  GGM,
  ONCE_CLICK_SAVE_MINE,
} movement_t;

/*get_gound_mine*/
typedef enum
{
  GGM_uplift_to_higher = 0,
  GGM_hand_to_pos,
  GGM_uplift_down,
  GGM_complete,
} get_gound_mine_t;
#define GGM_STEP1_HEIGHT 56.0f
#define GGM_STEP2_J1_ANGLE -0.43f
#define GGM_STEP2_J2_ANGLE -1.738f
#define GGM_STEP2_J3_ANGLE -0.56f
#define GGM_STEP2_PITCH_ANGLE 1.84f
#define GGM_STEP3_HEIGHT 10.0f
#define GGM_CPLT_HEIGHT 30

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
#define SM_STEP1_G_ANGLE 0.59f
#define SM_STEP1_J5_ANGLE 0.0f
#define SM_STEP1_J4_ANGLE 93.0f
#define SM_STEP1_J3_ANGLE 1.564f

#define SM_STEP2_J2_ANGLE -1.593f
#define SM_STEP2_J1_ANGLE 0.0f

#define SM_STEP3_HEIGHT 26

#define SM_STEP4_JG_ANGLE 0.0f

#define SM_STEP5_J1_ANGLE 0.413f
#define SM_STEP5_J2_ANGLE -1.922f


// #define SM_STEP4_DELAY_LOOP 500
// #define SM_CPLT_HEIGHT 275

typedef enum
{
  BTD_hand_to_pos = 0,
  BTD_uplift_to_pos,
  BTD_complete
} back_to_default_t;

typedef struct
{
  movement_t movement_type;
  uint8_t movement_step;

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
