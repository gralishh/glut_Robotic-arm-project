#ifndef __GENERAL_MOVEMENT__
#define __GENERAL_MOVEMENT__

#include "main.h" 

typedef enum{
  BTD=1,
  GGM,
  SM
} movement_t;

/*get_gound_mine*/
typedef enum{
  GGM_uplift_to_higher=0,
  GGM_hand_to_pos,
  GGM_uplift_down,
  GGM_complete
} get_gound_mine_t;
#define GGM_STEP1_HEIGHT 56.0f
#define GGM_STEP2_J1_ANGLE -0.43f
#define GGM_STEP2_J2_ANGLE -1.738f
#define GGM_STEP2_J3_ANGLE -0.56f
#define GGM_STEP2_PITCH_ANGLE 1.84f
#define GGM_STEP3_HEIGHT 12.0f
#define GGM_CPLT_HEIGHT 20

/*save_mine_t*/
typedef enum{
  SM_uplift_to_pos=0,
  SM_hand_to_pos,
  SM_uplift_down,
  SM_complete
} save_mine_t;
#define SM_STEP1_HEIGHT       0
#define SM_STEP2_J1_ANGLE     0
#define SM_STEP2_J2_ANGLE     0
#define SM_STEP2_J3_ANGLE     0
#define SM_STEP2_PITCH_ANGLE  0
#define SM_STEP3_HEIGHT       0
#define SM_CPLT_HEIGHT        0

typedef enum{
  BTD_hand_to_pos=0,
  BTD_uplift_to_pos,
  BTD_complete
} back_to_default_t;

typedef struct{
  uint8_t movement_type;
  uint8_t movement_step;
} movement_handler_t;

extern movement_handler_t movement;

void set_movement(uint8_t movement);
uint8_t get_movement(void);
uint8_t get_step(void);
void next_step(void);

#endif
