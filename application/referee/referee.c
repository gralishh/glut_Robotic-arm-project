#include "referee.h"
#include "string.h"
#include "stdio.h"
#include "CRC8_CRC16.h"
#include "protocol.h"
#include "CAN_receive.h"


frame_header_struct_t referee_receive_header;
frame_header_struct_t referee_send_header;

ext_game_state_t game_state;
ext_game_result_t game_result;
ext_game_robot_HP_t game_robot_HP_t;

ext_event_data_t field_event;
ext_supply_projectile_action_t supply_projectile_action_t;
ext_supply_projectile_booking_t supply_projectile_booking_t;
ext_referee_warning_t referee_warning_t;


ext_game_robot_state_t robot_state;
ext_power_heat_data_t power_heat_data_t;
ext_game_robot_pos_t game_robot_pos_t;
ext_buff_musk_t buff_musk_t;
aerial_robot_energy_t robot_energy_t;
ext_robot_hurt_t robot_hurt_t;
ext_shoot_data_t shoot_data_t;
ext_bullet_remaining_t bullet_remaining_t;
ext_student_interactive_data_t student_interactive_data_t;
map_command_t map_command;
//extern cap_measure_t cap;
extern uint8_t Fast;

void init_referee_struct_data(void)
{
    memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));
    memset(&referee_send_header, 0, sizeof(frame_header_struct_t));

    memset(&game_state, 0, sizeof(ext_game_state_t));
    memset(&game_result, 0, sizeof(ext_game_result_t));
    memset(&game_robot_HP_t, 0, sizeof(ext_game_robot_HP_t));


    memset(&field_event, 0, sizeof(ext_event_data_t));
    memset(&supply_projectile_action_t, 0, sizeof(ext_supply_projectile_action_t));
    memset(&supply_projectile_booking_t, 0, sizeof(ext_supply_projectile_booking_t));
    memset(&referee_warning_t, 0, sizeof(ext_referee_warning_t));

    memset(&map_command, 0, sizeof(map_command_t));
    memset(&robot_state, 0, sizeof(ext_game_robot_state_t));
    memset(&power_heat_data_t, 0, sizeof(ext_power_heat_data_t));
    memset(&game_robot_pos_t, 0, sizeof(ext_game_robot_pos_t));
    memset(&buff_musk_t, 0, sizeof(ext_buff_musk_t));
    memset(&robot_energy_t, 0, sizeof(aerial_robot_energy_t));
    memset(&robot_hurt_t, 0, sizeof(ext_robot_hurt_t));
    memset(&shoot_data_t, 0, sizeof(ext_shoot_data_t));
    memset(&bullet_remaining_t, 0, sizeof(ext_bullet_remaining_t));


    memset(&student_interactive_data_t, 0, sizeof(ext_student_interactive_data_t));



}

void referee_data_solve(uint8_t *frame)
{
    uint16_t cmd_id = 0;

    uint8_t index = 0;

    memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));

    index += sizeof(frame_header_struct_t);

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

    switch (cmd_id)
    {
        case GAME_STATE_CMD_ID:
        {
            memcpy(&game_state, frame + index, sizeof(ext_game_state_t));
        }
        break;
        case GAME_RESULT_CMD_ID:
        {
            memcpy(&game_result, frame + index, sizeof(game_result));
        }
        break;
        case GAME_ROBOT_HP_CMD_ID:
        {
            memcpy(&game_robot_HP_t, frame + index, sizeof(ext_game_robot_HP_t));
        }
        break;
        case FIELD_EVENTS_CMD_ID:
        {
            memcpy(&field_event, frame + index, sizeof(field_event));
        }
        break;
        case SUPPLY_PROJECTILE_ACTION_CMD_ID:
        {
            memcpy(&supply_projectile_action_t, frame + index, sizeof(supply_projectile_action_t));
        }
        break;
        case SUPPLY_PROJECTILE_BOOKING_CMD_ID:
        {
            memcpy(&supply_projectile_booking_t, frame + index, sizeof(supply_projectile_booking_t));
        }
        break;
        case REFEREE_WARNING_CMD_ID:
        {
            memcpy(&referee_warning_t, frame + index, sizeof(ext_referee_warning_t));
        }
        break;

        case ROBOT_STATE_CMD_ID:
        {
            memcpy(&robot_state, frame + index, sizeof(robot_state));
        }
        break;
        case POWER_HEAT_DATA_CMD_ID:
        {
            memcpy(&power_heat_data_t, frame + index, sizeof(power_heat_data_t));
        }
        break;
        case ROBOT_POS_CMD_ID:
        {
            memcpy(&game_robot_pos_t, frame + index, sizeof(game_robot_pos_t));
        }
        break;
        case BUFF_MUSK_CMD_ID:
        {
            memcpy(&buff_musk_t, frame + index, sizeof(buff_musk_t));
        }
        break;
        case AERIAL_ROBOT_ENERGY_CMD_ID:
        {
            memcpy(&robot_energy_t, frame + index, sizeof(robot_energy_t));
        }
        break;
        case ROBOT_HURT_CMD_ID:
        {
            memcpy(&robot_hurt_t, frame + index, sizeof(robot_hurt_t));
        }
        break;
        case SHOOT_DATA_CMD_ID:
        {
            memcpy(&shoot_data_t, frame + index, sizeof(shoot_data_t));
        }
        break;
        case BULLET_REMAINING_CMD_ID:
        {
            memcpy(&bullet_remaining_t, frame + index, sizeof(ext_bullet_remaining_t));
        }
        break;
        case STUDENT_INTERACTIVE_DATA_CMD_ID:
        {
            memcpy(&student_interactive_data_t, frame + index, sizeof(student_interactive_data_t));
        }
        case 0x0303:
        {
            memcpy(&map_command, frame + index, sizeof(map_command));
        }
        break;
        default:
        {
            break;
        }
    }
}


 bool_t refereeControl=1;
//得到机器人ID
uint8_t GetRobotID()
 {
	 if(refereeControl)
	 {
   return (uint8_t)robot_state.robot_id;
	 }
	 else 
	 {
		 return 1;
	 }
 }
 uint8_t GetEenmyColor()
{
  uint8_t enemyColor = 0;
	if(refereeControl)
	{
		if(robot_state.robot_id >0 && robot_state.robot_id < 10 )
		{
			//我方为红色，敌方为蓝色
			enemyColor = 1;
		}
		else if(robot_state.robot_id >100 && robot_state.robot_id < 110)
		{
			//我方为蓝色，敌方为红色
			enemyColor = 0;
		}
		return enemyColor;
	}
	else 
	{
		return 0;
	}
}
//得到底盘功率上限
uint16_t Get_chassis_power_limit()
{
	if(refereeControl)
	{
		uint16_t chassis_power_limit=0;
		chassis_power_limit=robot_state.chassis_power_limit;
		return   chassis_power_limit;
	}
	else 
	{
		return 70;
	}
}

//得到42毫米子弹射速上限
int16_t Get_shooter_42mm_speed_limit()
{
	if(refereeControl)
	{
	int16_t shooter_42mm_speed_limit=0;
	if(robot_state.shooter_heat1_speed_limit<256)
		   shooter_42mm_speed_limit=robot_state.shooter_heat2_speed_limit;
  else shooter_42mm_speed_limit=robot_state.shooter_heat2_speed_limit/256; 
  return shooter_42mm_speed_limit;	
	}
	else 
	{
		return 16;
	}
}// 获取底盘功率上限带参数版本
 void get_chassis_power_limit(uint16_t *chassis_power_limit_t)
{
	if(refereeControl)
	{
    *chassis_power_limit_t =robot_state.chassis_power_limit;
	}
	else 
	{
		*chassis_power_limit_t = 55;
	}

}
bool_t capstart=0;
float GetRealPower() //得到底盘实时功率
{
//	if(capstart)
//	{
//		power_heat_data_t.chassis_power=cap.Power;
//	}
//	else
//	{
//		power_heat_data_t.chassis_power=60;
//	}
//		return power_heat_data_t.chassis_power;
}
float GetPowerBuffer()  //得到底盘实时缓存功率
{
	if(refereeControl)
	{
		return power_heat_data_t.chassis_power_buffer;
	}
	else 
	{
		return 25;
	}
}
//float GetPowerBuffer()  //得到底盘实时缓存功率
//{
//	if(refereeControl)
//		{
//			uint8_t a=Get_chassis_power_limit()+60-cap.Power;
//			if(a<=60&&a>0)
//			{
//				power_heat_data_t.chassis_power_buffer=a;
//			}
//			else if(a>60)
//			{
//				power_heat_data_t.chassis_power_buffer=60;
//			}
//			else 
//			{
//				power_heat_data_t.chassis_power_buffer=0;
//			}
//		}
//		else
//		{
//			uint8_t a=Get_chassis_power_limit()-cap.Power;
//			if(a<=60&&a>0)
//			{
//				power_heat_data_t.chassis_power_buffer=a;
//			}
//			else if(a>60)
//			{
//				power_heat_data_t.chassis_power_buffer=60;
//			}
//			else 
//			{
//				power_heat_data_t.chassis_power_buffer=0;
//			}
//		}
//			if(Fast)
//			{
//				power_heat_data_t.chassis_power_buffer=50;
//			}
//		return power_heat_data_t.chassis_power_buffer;
//}
float GetPowerHeat() 
{
	if(refereeControl)
	{
		return power_heat_data_t.shooter_id1_42mm_cooling_heat;
	}
	else 
	{
		return 10;
	}
}
int16_t GetBullet()
{
	if(refereeControl)
	{
    return power_heat_data_t.shooter_id1_42mm_cooling_heat;
	}
	
	else 
	{
		return 1;
	}
}
float bullet_speed()
{
	if(refereeControl)
	{
    return shoot_data_t.bullet_speed;
	}
	else 
	{
		return 15;
	}
}
void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer)
{
    *power = power_heat_data_t.chassis_power;
    *buffer = power_heat_data_t.chassis_power_buffer;

}
uint8_t GetHp()
{
	if(refereeControl)
	{
		if(robot_state.remain_HP > 0)
		return 1;
		else 
		return 0;
	}
	else
	{
		return 1;
	}
}

/*如果5秒倒计时返回true*/
uint8_t GetMatchReady(void)
{
  return game_state.game_progress==3;
}

