#include "general_movement.h"

movement_handler_t movement;

void set_movement(movement_t movement_type)
{
  movement.movement_type = movement_type;
  movement.movement_step=0;
}
        
uint8_t get_movement(void)
{
  return movement.movement_type;
}
// uint8_t get_out_flag(void)
// {
//   return movement.hand_move_out_flag;
// }
uint8_t get_step(void)
{
  return movement.movement_step;
}

void next_step(void)
{
  movement.movement_step++;
}
