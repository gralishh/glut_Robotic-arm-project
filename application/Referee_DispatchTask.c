/**
 * @brief 裁判系统解包代码
 */
#include "Referee_DispatchTask.h"
#include "string.h"
#include "stdbool.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "gimbal_task.h"
#include "bsp_usart.h"
#include "referee.h"
#include "core_cm7.h"

unsigned char Transmission_BufferOfUsart6[50] = {0};  //unsigned char类型长度??1个字??
Referee_Date Referee_date1;
ext_self_control_t self_control_date;
ext_self_control_t *get_self_control_original_point(void)
{
return &self_control_date;
}

custom_client_data_t custom_client_data;

unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);

int16_t crc_fail_cnt = 0;
unsigned int Len = 0,frame_len;
int16_t iii = 0;


void DispchRefereeTask(void const *parmas)
{

	
	for(;;)
	{
		next:	Len = USART1_GetDataCount();  //得出数据的长度，帧头减去帧尾，这个长度（包括帧头，帧尾，ID，和有用的数据）
		
		if(Len >= 10)
		{
			if(	USART1_At(5)  == 0x02   && 	USART1_At(6)  == 0x03 )  //只接收自定义控制器数据
			{
				frame_len =  USART1_At(2) << 8 | USART1_At(1);   //得出传输数据的字??
				if(Len >= 15) //
				{
					 USART1_Recv(Transmission_BufferOfUsart6,frame_len + 9);  //把数据出栈并存储在Transmission_BufferOfUsart6，，数据处理，数据的接收在中断里
					 USART1_Drop(4096);

				}
				else 
				{
					
				    vTaskDelay(2);
					goto next;
				}
					 
				
				if(Verify_CRC8_Check_Sum(Transmission_BufferOfUsart6,5) &&  Verify_CRC16_Check_Sum(Transmission_BufferOfUsart6,frame_len + 9) == true) 
				{
					memcpy(&Referee_date1.frame_header,Transmission_BufferOfUsart6,sizeof(Referee_date1.frame_header));
					Referee_date1.CmdID  =  Transmission_BufferOfUsart6[6] << 8 | Transmission_BufferOfUsart6[5];  //帧头??5个字节，ID占两个字节，所以第5和第6个字节表示ID
					switch(Referee_date1.CmdID)
					{   //??7个字节到??50个字节中存储着用户所需的数据，同一数据Transmission_BufferOfUsart6中只存储着一种数??
					 case Competition_Satus_e:
						 memcpy(&Referee_date1.game_state,&Transmission_BufferOfUsart6[7],Referee_date1.frame_header.data_length);
						 break;
					 
					 case Competition_Result_e:
						 memcpy(&Referee_date1.game_result,&Transmission_BufferOfUsart6[7],Referee_date1.frame_header.data_length);
						 break;
					 
					  case Robot_Survive_Data_e:
						 memcpy(&Referee_date1.game_robot_survivors,&Transmission_BufferOfUsart6[7],Referee_date1.frame_header.data_length);
						 break;	
					  case Rart_launch_Satus_e:
						 memcpy(&Referee_date1.dart_status,&Transmission_BufferOfUsart6[7],Referee_date1.frame_header.data_length);
						 break;	
					  case Site_Event_Data_e:
						 memcpy(&Referee_date1.event_data,&Transmission_BufferOfUsart6[7],Referee_date1.frame_header.data_length);
						 break;		
					  
					  case Supply_Station_Data_e:
						 memcpy(&Referee_date1.supply_projectile_action,&Transmission_BufferOfUsart6[7],Referee_date1.frame_header.data_length);
						 break;
					  
					  case Request_Bullet_Data_e:  //裁判系统没有
						 memcpy(&Referee_date1.supply_projectile_booking,&Transmission_BufferOfUsart6[7],Referee_date1.frame_header.data_length);
						break;
					  
						case Referee_Warning_Message_t:  //裁判警告信息
						 memcpy(&Referee_date1.referee_warning_message,&Transmission_BufferOfUsart6[7],Referee_date1.frame_header.data_length);
						break;
						
				  	case Rart_Remaining_Time_t:  //飞镖发射口倒计??
						 memcpy(&Referee_date1.dart_remaining_time,&Transmission_BufferOfUsart6[7],Referee_date1.frame_header.data_length);
						break;
						
					  case Bullet_Remaining_t:  //子弹剩余发射??
						 memcpy(&Referee_date1.bullet_remaining,&Transmission_BufferOfUsart6[7],Referee_date1.frame_header.data_length);
						break;
						
					 	case Robot_Rfid_Status_t:  //机器?? RFID 状??
						 memcpy(&Referee_date1.rfid_status,&Transmission_BufferOfUsart6[7],Referee_date1.frame_header.data_length);
						break;
					 
					 	case Dart_Client_Cmd_t:  //飞镖机器人客户端指令数据
						 memcpy(&Referee_date1.dart_client_cmd,&Transmission_BufferOfUsart6[7],Referee_date1.frame_header.data_length);
						break;						
						
						
					  case Robot_Status_Data_e:  //这里数据缺失 机器人状态数??
						 memcpy(&Referee_date1.game_robot_state,&Transmission_BufferOfUsart6[7],Referee_date1.frame_header.data_length);
						break;
					  
					  case Power_Heat_Data_e:  //这里数据缺失
						 memcpy(&Referee_date1.power_heat_data,&Transmission_BufferOfUsart6[7],Referee_date1.frame_header.data_length);
						break;		

					  case Robot_Position:
						 memcpy(&Referee_date1.game_robot_pos,&Transmission_BufferOfUsart6[7],Referee_date1.frame_header.data_length);
						break;

					  case Robot_Gain_Data_e:
						 memcpy(&Referee_date1.buff_musk,&Transmission_BufferOfUsart6[7],Referee_date1.frame_header.data_length);
						break;	
					  
					  case Air_Robot_Power_Data_e:
						 memcpy(&Referee_date1.aerial_robot_energy,&Transmission_BufferOfUsart6[7],Referee_date1.frame_header.data_length);
						break; 
					  
					  case Hurt_data_e:
						 memcpy(&Referee_date1.robot_hurt,&Transmission_BufferOfUsart6[7],Referee_date1.frame_header.data_length);
						break; 							  
					  
					  case Shoot_Data_e: //全了
						 memcpy(&Referee_date1.shoot_data,&Transmission_BufferOfUsart6[7],Referee_date1.frame_header.data_length);
						break; 
					  
					  case Robot_Interaction:
					  	if(Transmission_BufferOfUsart6[21]==0x01)
						{
					    	if (Verify_CRC16_Check_Sum(Transmission_BufferOfUsart6,sizeof(Transmission_BufferOfUsart6)))
					  		{
								memcpy(&Referee_date1.student_interactive_header_data,&Transmission_BufferOfUsart6[7],Referee_date1.frame_header.data_length);
							}
						}
						break; 
					  
					  case 0x0302:
						memcpy(&Referee_date1.self_control,&Transmission_BufferOfUsart6[7],Referee_date1.frame_header.data_length);
						memcpy(&self_control_date,&Transmission_BufferOfUsart6[7],Referee_date1.frame_header.data_length);
					  break; 
					  case Custom_Client_Data_e:
						 memcpy(&custom_client_data,&Transmission_BufferOfUsart6[7],Referee_date1.frame_header.data_length);
						break; 
					  default:
						break;								  		  
					}					
				}
				else 
				{
				  crc_fail_cnt++;
				}
			}	
			else 
			{
				USART1_Drop(1);//丢掉
				for(;;)
				{
					if(USART1_GetDataCount() > 0)
					{
						if(	USART1_At(0)  == 0xA5 )// Frame head
						{
							break;
						}
						else
						{
							USART1_Drop(1);
							vTaskDelay(1);
						}
						Len =  USART1_GetDataCount();
						if(Len > 3000)
						{
							USART1_Drop(4096);
							break;
						}
					}
					else
					{
						break;
					}
				}
			}			
		}
	   vTaskDelay(2);	
   }
}
      
//extern Referee_Date Referee_date1;
//	/*********************self_control**********************/

///*********************self_control**********************/
//const ext_self_control_t *get_self_control_date_point(void)
//{
//    return &self_control_date;
//}
///*********************self_control**********************/
//得到机器人ID
uint8_t GetRobotID()
 {
   return (uint8_t)Referee_date1.game_robot_state.robot_id;
 }
uint8_t GetEenmyColor()
{
  uint8_t enemyColor = 0;
	if(Referee_date1.game_robot_state.robot_id >0 && Referee_date1.game_robot_state.robot_id < 10 )
	{
		//我方为红色，敌方为蓝??
		enemyColor = 1;
	}
	else if(Referee_date1.game_robot_state.robot_id >100 && Referee_date1.game_robot_state.robot_id < 110)
	{
		//我方为蓝色，敌方为红??
		enemyColor = 0;
	}
   return enemyColor;
}
//得到17毫米子弹射速上??
int16_t Get_shooter_17mm_speed_limit()
{
	 int16_t shooter_17mm_speed_limit=0;
	 if(Referee_date1.game_robot_state.shooter_id1_17mm_speed_limit<256)
		    shooter_17mm_speed_limit=Referee_date1.game_robot_state.shooter_id1_17mm_speed_limit;
   else shooter_17mm_speed_limit=Referee_date1.game_robot_state.shooter_id1_17mm_speed_limit/256; 
    return shooter_17mm_speed_limit;	
}
//得到42毫米子弹射速上??
int16_t Get_shooter_42mm_speed_limit()
{
	int16_t shooter_42mm_speed_limit=0;
	if(Referee_date1.game_robot_state.shooter_id1_42mm_speed_limit<256)
		   shooter_42mm_speed_limit=Referee_date1.game_robot_state.shooter_id1_42mm_speed_limit;
  else shooter_42mm_speed_limit=Referee_date1.game_robot_state.shooter_id1_42mm_speed_limit/256; 
   return shooter_42mm_speed_limit;	
}
//得到底盘功率上限
uint16_t Get_chassis_power_limit()
{
  uint16_t chassis_power_limit=0;
	if(Referee_date1.game_robot_state.chassis_power_limit<256)
		chassis_power_limit=Referee_date1.game_robot_state.chassis_power_limit;
	else chassis_power_limit=Referee_date1.game_robot_state.chassis_power_limit/256;
	return   chassis_power_limit;
}
float GetRealPower() //得到底盘实时功率
{
	return Referee_date1.power_heat_data.chassis_power;
}

float GetPowerBuffer()  //得到底盘缓存功率上线
{
  return Referee_date1.power_heat_data.chassis_power_buffer;
}
 //得到实时射??
float Get_shouter_speed()
{
  	 int16_t shooter_speed=0;
	 if(Referee_date1.shoot_data.bullet_speed<256)
		    shooter_speed=Referee_date1.shoot_data.bullet_speed;
   else shooter_speed=Referee_date1.shoot_data.bullet_speed/256; 
    return shooter_speed;
}
//得到实时射频
int Get_shouter_freq()
{
     int16_t shooter_speed=0;
		 shooter_speed=Referee_date1.shoot_data.bullet_freq; 
    return shooter_speed;
}
Referee_Date *GetRefereeDataPoint()
{
	return &Referee_date1;
}

extern bool iSIdentifySuccess(void);
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);
void SendDataToClient()
{	
//	static clientData data;
//     
//	data.head.sof = 0XA5;
//	data.head.data_length = 19+9;
//	data.head.seq++;
//    Append_CRC8_Check_Sum((uint8_t *)&data.head,sizeof(data.head));
//   
//    data.cmd_id = Robot_Interaction;
//    
//    data.student_interactive_header_data.data_cmd_id = 0xD180; 
//    data.student_interactive_header_data.send_ID =  Referee_date1.game_robot_state.robot_id;
//    data.student_interactive_header_data.receiver_ID = (Referee_date1.game_robot_state.robot_id ==  1)? 0x0101:0x0111; 
//    
//    data.customize_data1 = 0.0f;
//    data.customize_data2 = 0.0f;
//    data.customize_data3 = 0.0f;
//   // data.Indicator_light =  iSIdentifySuccess() ? (data.Indicator_light |(1<<0)):data.Indicator_light & (~(1<<0));
//	//data.Indicator_light =	Butten_Trig_Pin     ? data.Indicator_light & (~(1<<1)):(data.Indicator_light |(1<<1));
//   
//    Append_CRC16_Check_Sum((uint8_t *)&data,sizeof(data));
//	 
//	USART1_Send((uint8_t *)&data,sizeof(data));  
}


//crc8 generator polynomial:G(x)=x8+x5+x4+1
const unsigned char CRC8_Init = 0xff;
const unsigned char CRC8_TAB[256] =
{
0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc, 0x23,
0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62, 0xbe, 0xe0,
0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff, 0x46, 0x18, 0xfa,
0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07, 0xdb, 0x85, 0x67,
0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a, 0x65, 0x3b, 0xd9, 0x87,
0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24, 0xf8, 0xa6, 0x44, 0x1a, 0x99,
0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd, 0x11,
0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50, 0xaf, 0xf1,
0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee, 0x32, 0x6c, 0x8e,
0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73, 0xca, 0x94, 0x76, 0x28,
0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b, 0x57, 0x09, 0xeb, 0xb5,
0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16, 0xe9, 0xb7, 0x55, 0x0b, 0x88,
0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8, 
0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};
unsigned  char  Get_CRC8_Check_Sum(unsigned  char  *pchMessage,unsigned  int
dwLength,unsigned char ucCRC8)
{
   unsigned char ucIndex;
   while (dwLength--)
    {
       ucIndex = ucCRC8^(*pchMessage++);
       ucCRC8 = CRC8_TAB[ucIndex];
    }
    return(ucCRC8);
}
/*
** Descriptions: CRC8 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
  unsigned char ucExpected = 0;
  if ((pchMessage == 0) || (dwLength <= 2)) return 0;
     ucExpected = Get_CRC8_Check_Sum (pchMessage, dwLength-1, CRC8_Init);
  return ( ucExpected == pchMessage[dwLength-1] );
}
/*
** Descriptions: append CRC8 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
  unsigned char ucCRC = 0;
  if ((pchMessage == 0) || (dwLength <= 2)) return;
     ucCRC = Get_CRC8_Check_Sum ( (unsigned char *)pchMessage, dwLength-1, CRC8_Init);
     pchMessage[dwLength-1] = ucCRC;
}
uint16_t CRC_INIT = 0xffff;
const uint16_t wCRC_Table[256] =
{
0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
} ;
/*
** Descriptions: CRC16 checksum function
** Input: Data to check,Stream length, initialized checksum
** Output: CRC checksum
*/
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC)
{
  uint8_t chData;
  if (pchMessage == NULL)
    {
      return 0xFFFF;
    }
  while(dwLength--)
   {
     chData = *pchMessage++;
     (wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff];
   }
  return wCRC;
}
/*
** Descriptions: CRC16 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
  uint16_t wExpected = 0;
  if ((pchMessage == NULL) || (dwLength <= 2))
    {
      return false;
    }
  wExpected = Get_CRC16_Check_Sum ( pchMessage, dwLength - 2, CRC_INIT);
  return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) ==
  pchMessage[dwLength - 1]);
}
/*
** Descriptions: append CRC16 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength)
{
  uint16_t wCRC = 0;
  if ((pchMessage == NULL) || (dwLength <= 2))
    {
      return;
    }
  wCRC = Get_CRC16_Check_Sum ( (uint8_t *)pchMessage, dwLength-2, CRC_INIT );
  pchMessage[dwLength-2] = (uint8_t)(wCRC & 0x00ff);
  pchMessage[dwLength-1] = (uint8_t)((wCRC >> 8)& 0x00ff);
}


#define send_max_len     200
#define custom_graphic_single_lenth 15
#define custom_graphic_double_lenth 30
#define custom_graphic_five_lenth 75
#define custom_character_lenth 45
#define custom_graphic_seven_lenth 105

//extern  Gimbal_Control_t gimbal_control;

uint8_t keyboard_C_state;
uint8_t keyboard_C_state_last;
uint8_t RISE3_state;
uint8_t RISE3_state_last;
void Draw_grahic(uint16_t cmd_id, uint16_t data_id, uint16_t tx_id, uint16_t rx_id)
{
	unsigned char CliendTxBuffer[send_max_len];
	static ext_draw_ui draw;
	static uint32_t alllength;
	static uint8_t seq=0;
	draw.head.sof = 0xA5;  
	draw.head.data_length=6+custom_graphic_seven_lenth; //画七个图形数据长??
	draw.head.seq=seq;
	memcpy(CliendTxBuffer, &draw.head, sizeof(frame_header_t));//写入帧头数据
	Append_CRC8_Check_Sum(CliendTxBuffer,sizeof(frame_header_t)); //写入帧头校验

	
	draw.cmd_id = cmd_id;  //机器人间交互ID 301
	
	draw.Client_Custom_ID.data_cmd_id= data_id; //约定的画图ID 0x101
	draw.Client_Custom_ID.sender_ID = tx_id;
	draw.Client_Custom_ID.receiver_ID = rx_id;
	
	/*自定义内??*/
/*
operate_tpye—??0：空操作??	1：增加；2：修改；3：删除；
graphic_tpye—??0：直线；	1：矩形；2：整圆；3：椭圆；4：圆弧；5：浮点数??6：整型数??7：字符；
						1920??960），1080??540??
		 —————??
200~880	|			 |	320~880			1340/1474
		|			 |					446
		|			 |
		|			 |
		 —————??
0??0
*/
	/*空接辅助线——左*/
	draw.grapic_data_struct[0].graphic_name[0]=21;
	draw.grapic_data_struct[0].operate_tpye=1;
	draw.grapic_data_struct[0].graphic_tpye=0;
	draw.grapic_data_struct[0].layer=2;
	draw.grapic_data_struct[0].color=1;
	draw.grapic_data_struct[0].width=4;
	draw.grapic_data_struct[0].radius=0;
	draw.grapic_data_struct[0].start_x=930;
	draw.grapic_data_struct[0].start_y=200;
	draw.grapic_data_struct[0].end_x=930;
	draw.grapic_data_struct[0].end_y=880;
	/*空接辅助线——右*/
	draw.grapic_data_struct[1].graphic_name[0]=22;
	draw.grapic_data_struct[1].operate_tpye=1;
	draw.grapic_data_struct[1].graphic_tpye=0;
	draw.grapic_data_struct[1].layer=2;
	draw.grapic_data_struct[1].color=1;
	draw.grapic_data_struct[1].width=4;
	draw.grapic_data_struct[1].start_x=990;
	draw.grapic_data_struct[1].start_y=200;
	draw.grapic_data_struct[1].end_x=990;
	draw.grapic_data_struct[1].end_y=880;
	
	draw.grapic_data_struct[2].graphic_name[0]=23;
	draw.grapic_data_struct[2].graphic_tpye=0;
	draw.grapic_data_struct[2].layer=2;
	draw.grapic_data_struct[2].width=4;
	
	draw.grapic_data_struct[3].graphic_name[0]=24;
	draw.grapic_data_struct[3].graphic_tpye=0;
	draw.grapic_data_struct[3].layer=2;
	draw.grapic_data_struct[3].width=4;
	
	// keyboard_C_state =get_keyboard_C_state();
	if(keyboard_C_state_last != keyboard_C_state)
	{
		if(keyboard_C_state == 1)
		{
			/*储矿位置??*/
			draw.grapic_data_struct[2].operate_tpye=2;
			draw.grapic_data_struct[2].color=4;
			draw.grapic_data_struct[2].start_x=50;
			draw.grapic_data_struct[2].start_y=600;
			draw.grapic_data_struct[2].end_x=150;
			draw.grapic_data_struct[2].end_y=600;

			/*储矿位置??*/
			draw.grapic_data_struct[3].operate_tpye=2;
			draw.grapic_data_struct[3].color=1;
			draw.grapic_data_struct[3].start_x=150;
			draw.grapic_data_struct[3].start_y=600;
			draw.grapic_data_struct[3].end_x=250;
			draw.grapic_data_struct[3].end_y=600;
		}
		else if(keyboard_C_state == 0)
		{
			/*储矿位置??*/
			draw.grapic_data_struct[2].operate_tpye=2;
			draw.grapic_data_struct[2].color=4;
			draw.grapic_data_struct[2].start_x=150;
			draw.grapic_data_struct[2].start_y=600;
			draw.grapic_data_struct[2].end_x=150;
			draw.grapic_data_struct[2].end_y=700;

			/*储矿位置??*/
			draw.grapic_data_struct[3].operate_tpye=2;
			draw.grapic_data_struct[3].color=1;
			draw.grapic_data_struct[3].start_x=150;
			draw.grapic_data_struct[3].start_y=600;
			draw.grapic_data_struct[3].end_x=250;
			draw.grapic_data_struct[3].end_y=600;
		}
		keyboard_C_state_last = keyboard_C_state;
	}
	else
	{
		if(keyboard_C_state == 1)
		{
			/*储矿位置??*/
			draw.grapic_data_struct[2].operate_tpye=1;
			draw.grapic_data_struct[2].color=4;
			draw.grapic_data_struct[2].start_x=50;
			draw.grapic_data_struct[2].start_y=600;
			draw.grapic_data_struct[2].end_x=150;
			draw.grapic_data_struct[2].end_y=600;

			/*储矿位置??*/
			draw.grapic_data_struct[3].operate_tpye=1;
			draw.grapic_data_struct[3].color=1;
			draw.grapic_data_struct[3].start_x=150;
			draw.grapic_data_struct[3].start_y=600;
			draw.grapic_data_struct[3].end_x=250;
			draw.grapic_data_struct[3].end_y=600;
		}
		else if(keyboard_C_state == 0)
		{
			/*储矿位置??*/
			draw.grapic_data_struct[2].operate_tpye=1;
			draw.grapic_data_struct[2].color=4;
			draw.grapic_data_struct[2].start_x=150;
			draw.grapic_data_struct[2].start_y=600;
			draw.grapic_data_struct[2].end_x=150;
			draw.grapic_data_struct[2].end_y=700;

			/*储矿位置??*/
			draw.grapic_data_struct[3].operate_tpye=1;
			draw.grapic_data_struct[3].color=1;
			draw.grapic_data_struct[3].start_x=150;
			draw.grapic_data_struct[3].start_y=600;
			draw.grapic_data_struct[3].end_x=250;
			draw.grapic_data_struct[3].end_y=600;
		}
	}
	/****************************************************************/
	draw.grapic_data_struct[4].graphic_name[0]=25;
	draw.grapic_data_struct[4].graphic_tpye=0;
	draw.grapic_data_struct[4].layer=2;
	draw.grapic_data_struct[4].width=4;
	
	// RISE3_state = get_RISE3_height();
	if( RISE3_state_last != RISE3_state)
	{
		if(RISE3_state == 0)
		{
			draw.grapic_data_struct[4].operate_tpye=2;
			draw.grapic_data_struct[4].color=1;
			draw.grapic_data_struct[4].start_x=50;
			draw.grapic_data_struct[4].start_y=400;
			draw.grapic_data_struct[4].end_x=250;
			draw.grapic_data_struct[4].end_y=400;
		}
		else if(RISE3_state == 1)
		{
			draw.grapic_data_struct[4].operate_tpye=2;
			draw.grapic_data_struct[4].color=1;
			draw.grapic_data_struct[4].start_x=250;
			draw.grapic_data_struct[4].start_y=400;
			draw.grapic_data_struct[4].end_x=250;
			draw.grapic_data_struct[4].end_y=600;
		}
		else if(RISE3_state == 2)
		{
			draw.grapic_data_struct[4].operate_tpye=2;
			draw.grapic_data_struct[4].color=1;
			draw.grapic_data_struct[4].start_x=250;
			draw.grapic_data_struct[4].start_y=400;
			draw.grapic_data_struct[4].end_x=450;
			draw.grapic_data_struct[4].end_y=400;
		}
		RISE3_state_last = RISE3_state;
	}
	else 
	{
		if(RISE3_state == 0)
		{
			draw.grapic_data_struct[4].operate_tpye=1;
			draw.grapic_data_struct[4].color=1;
			draw.grapic_data_struct[4].start_x=50;
			draw.grapic_data_struct[4].start_y=400;
			draw.grapic_data_struct[4].end_x=250;
			draw.grapic_data_struct[4].end_y=400;
		}
		else if(RISE3_state == 1)
		{
			draw.grapic_data_struct[4].operate_tpye=1;
			draw.grapic_data_struct[4].color=1;
			draw.grapic_data_struct[4].start_x=250;
			draw.grapic_data_struct[4].start_y=400;
			draw.grapic_data_struct[4].end_x=250;
			draw.grapic_data_struct[4].end_y=600;
		}
		else if(RISE3_state == 2)
		{
			draw.grapic_data_struct[4].operate_tpye=1;
			draw.grapic_data_struct[4].color=1;
			draw.grapic_data_struct[4].start_x=250;
			draw.grapic_data_struct[4].start_y=400;
			draw.grapic_data_struct[4].end_x=450;
			draw.grapic_data_struct[4].end_y=400;
		}
	}
	
//	draw.grapic_data_struct[4].graphic_name[0]=25;
//	draw.grapic_data_struct[4].operate_tpye=1;
//	draw.grapic_data_struct[4].graphic_tpye=0;
//	draw.grapic_data_struct[4].layer=2;
//	draw.grapic_data_struct[4].color=1;
//	draw.grapic_data_struct[4].width=4;
//	draw.grapic_data_struct[4].start_x=50;
//	draw.grapic_data_struct[4].start_y=730;
//	draw.grapic_data_struct[4].end_x=50;
//	draw.grapic_data_struct[4].end_y=650;
//	
//	draw.grapic_data_struct[5].graphic_name[0]=26;
//	draw.grapic_data_struct[5].operate_tpye=1;
//	draw.grapic_data_struct[5].graphic_tpye=0;
//	draw.grapic_data_struct[5].layer=2;
//	draw.grapic_data_struct[5].color=1;
//	draw.grapic_data_struct[5].width=4;
//	draw.grapic_data_struct[5].start_x=50;
//	draw.grapic_data_struct[5].start_y=652;
//	draw.grapic_data_struct[5].end_x=105;
//	draw.grapic_data_struct[5].end_y=652;
//	
//	draw.grapic_data_struct[6].graphic_name[0]=27;
//	draw.grapic_data_struct[6].operate_tpye=1;
//	draw.grapic_data_struct[6].graphic_tpye=1;
//	draw.grapic_data_struct[6].layer=2;
//	draw.grapic_data_struct[6].color=1;
//	draw.grapic_data_struct[6].width=4;
//	draw.grapic_data_struct[6].start_x=50;
//	draw.grapic_data_struct[6].start_y=600;
//	draw.grapic_data_struct[6].end_x=105;
//	draw.grapic_data_struct[6].end_y=545;
	
	memcpy(	
			CliendTxBuffer + 5, 
			(uint8_t*)&draw.cmd_id, 
			(sizeof(draw.cmd_id)+ sizeof(draw.Client_Custom_ID)+ sizeof(draw.grapic_data_struct))
		  );
	
	Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(draw));
	alllength = sizeof(draw);

  USART1_Send((uint8_t *)CliendTxBuffer,alllength);
	//for(i = 0; i< alllength; i++)
	//{
 //       HAL_UART_Transmit(&huart6, (uint8_t *)CliendTxBuffer[i], 1, 1000);
 //       while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET);

	//}

}

//void Draw_grahic_status_change(uint16_t cmd_id, uint16_t data_id, uint16_t tx_id, uint16_t rx_id) //显示改变之后的图??
//{
//	unsigned char CliendTxBuffer[send_max_len];
//  	static ext_draw_ui draw;
//	static u32 alllength, i;
//	static uint8_t seq=0;
//	draw.head.sof = 0xA5;  
//	draw.head.data_length=6+105; 
//	draw.head.seq=seq;
//	memcpy(CliendTxBuffer, &draw.head, sizeof(frame_header_t));//写入帧头数据
//	Append_CRC8_Check_Sum(CliendTxBuffer,sizeof(frame_header_t)); //写入帧头校验

//	
//	draw.cmd_id = cmd_id;  //机器人间交互ID 301
//	
//	draw.Client_Custom_ID.data_cmd_id= data_id; //约定的画图ID
//	draw.Client_Custom_ID.sender_ID = tx_id;
//	draw.Client_Custom_ID.receiver_ID = rx_id;
//	
//	keyboard_C_state =get_keyboard_C_state();
//	//自定义内??
//	if(keyboard_C_state == 1)
//	{
//		/*储矿位置??*/
//		draw.grapic_data_struct[2].graphic_name[0]=23;
//		draw.grapic_data_struct[2].operate_tpye=2;
//		draw.grapic_data_struct[2].graphic_tpye=0;
//		draw.grapic_data_struct[2].layer=2;
//		draw.grapic_data_struct[2].color=4;
//		draw.grapic_data_struct[2].width=4;
//		draw.grapic_data_struct[2].start_x=50;
//		draw.grapic_data_struct[2].start_y=600;
//		draw.grapic_data_struct[2].end_x=150;
//		draw.grapic_data_struct[2].end_y=600;

//		/*储矿位置??*/
//		draw.grapic_data_struct[3].graphic_name[0]=24;
//		draw.grapic_data_struct[3].operate_tpye=2;
//		draw.grapic_data_struct[3].graphic_tpye=0;
//		draw.grapic_data_struct[3].layer=2;
//		draw.grapic_data_struct[3].color=1;
//		draw.grapic_data_struct[3].width=4;
//		draw.grapic_data_struct[3].start_x=150;
//		draw.grapic_data_struct[3].start_y=600;
//		draw.grapic_data_struct[3].end_x=250;
//		draw.grapic_data_struct[3].end_y=600;
//		
//		memcpy(	
//				CliendTxBuffer + 5, 
//				(uint8_t*)&draw.cmd_id, 
//				(sizeof(draw.cmd_id)+ sizeof(draw.Client_Custom_ID)+ sizeof(draw.grapic_data_struct)));
//		
//		Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(draw));
//		alllength = sizeof(draw);

//		for(i = 0; i< alllength; i++)
//		{
//			USART_SendData(USART1,(uint16_t)CliendTxBuffer[i]);
//			while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);
//		}
//	}
//}
	uint8_t up_pump ;
	uint8_t down_pump ;
	uint8_t pump_ui_mode=0;/**/
	uint8_t pump_ui_mode_last=0;/**/
void Draw_grahic_char_1(uint16_t cmd_id, uint16_t data_id, uint16_t tx_id, uint16_t rx_id)
{
	unsigned char CliendTxBuffer[send_max_len];

	uint8_t text1[]="up: open   down: open";
	uint8_t text2[]="up: open   down: close";
	uint8_t text3[]="up: close  down: open";
	uint8_t text4[]="up: close  down: close";
		
	static ext_draw_ui_character draw;
	static uint32_t alllength;
	static uint8_t seq=0;
	draw.head.sof = 0xA5;  
	draw.head.data_length=6+45;
	draw.head.seq=seq;
	memcpy(CliendTxBuffer, &draw.head, sizeof(frame_header_t));//写入帧头数据
	Append_CRC8_Check_Sum(CliendTxBuffer,sizeof(frame_header_t)); //写入帧头校验

	draw.cmd_id = cmd_id;  //机器人间交互ID 301
	
	draw.Client_Custom_ID.data_cmd_id= data_id; //约定的画图ID
	draw.Client_Custom_ID.sender_ID = tx_id;
	draw.Client_Custom_ID.receiver_ID = rx_id;
	
	//自定义内??
	/*二级升降*/
	draw.grapic_data_struct.graphic_name[0]=10;
	draw.grapic_data_struct.operate_tpye=1;
	draw.grapic_data_struct.graphic_tpye=7;/*字符*/
	draw.grapic_data_struct.layer=3;
	draw.grapic_data_struct.color=6;
	draw.grapic_data_struct.start_angle=20;	/*字体大小*/
	draw.grapic_data_struct.end_angle=12;	/*字符长度*/
	draw.grapic_data_struct.width=2;
	draw.grapic_data_struct.start_x=1320;  
	draw.grapic_data_struct.start_y=650;  
	draw.grapic_data_struct.radius=0;	
	draw.grapic_data_struct.end_x=0;
	draw.grapic_data_struct.end_y=0;
	

	// up_pump = get_up_pump();
	// down_pump = get_down_pump();
	
		if(up_pump == 1 && down_pump == 1)
		{
			pump_ui_mode = 3;
		}
		else if(up_pump == 1 && down_pump == 0)
		{
			pump_ui_mode = 2;
		}
		else if(up_pump == 0 && down_pump == 1)
		{
			pump_ui_mode = 1;
		}
		else if(up_pump == 0 && down_pump == 0)
		{
			pump_ui_mode = 0;
		}

	if(pump_ui_mode_last!=pump_ui_mode)
	{
		if(pump_ui_mode==3)
		{
			memcpy(draw.data,&text1,sizeof(text1));
			pump_ui_mode_last = pump_ui_mode;
		}
		else if(pump_ui_mode==2)
		{
			memcpy(draw.data,&text2,sizeof(text2));
			pump_ui_mode_last = pump_ui_mode;
		}
		else if(pump_ui_mode==1)
		{
			memcpy(draw.data,&text3,sizeof(text3));
			pump_ui_mode_last = pump_ui_mode;
		}
		else if(pump_ui_mode==0)
		{
			memcpy(draw.data,&text4,sizeof(text4));
			pump_ui_mode_last = pump_ui_mode;
		}
		draw.grapic_data_struct.operate_tpye = 2;
	
	}
	else
	{
		if(pump_ui_mode==3)
		{
			memcpy(draw.data,&text1,sizeof(text1));
			pump_ui_mode_last = pump_ui_mode;
		}
		else if(pump_ui_mode==2)
		{
			memcpy(draw.data,&text2,sizeof(text2));
			pump_ui_mode_last = pump_ui_mode;
		}
		else if(pump_ui_mode==1)
		{
			memcpy(draw.data,&text3,sizeof(text3));
			pump_ui_mode_last = pump_ui_mode;
		}
		else if(pump_ui_mode==0)
		{
			memcpy(draw.data,&text4,sizeof(text4));
			pump_ui_mode_last = pump_ui_mode;
		}
		draw.grapic_data_struct.operate_tpye = 1;
		
	}
		

	
/*******************************************************************************************************************/
	memcpy(	
			CliendTxBuffer + 5, 
			(uint8_t*)&draw.cmd_id, 
			(sizeof(draw.cmd_id)+ sizeof(draw.Client_Custom_ID)+ sizeof(draw.grapic_data_struct)+sizeof(draw.data))
		  );
	
	Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(draw));

	alllength = sizeof(draw);

  USART1_Send((uint8_t *)CliendTxBuffer,alllength);
	//for(i = 0; i< alllength; i++)
	//{
 //       HAL_UART_Transmit(&huart6, (uint8_t *)CliendTxBuffer[i], 1, 1000);
 //       while(__HAL_UART_GET_FLAG(&huart6, UART_FLAG_TC) == RESET);
	//}
}

uint8_t RISE1_height =0;
uint8_t RISE2_height=0;
uint8_t RISE3_height=0;
uint8_t RISE1_height_last=0;
uint8_t RISE2_height_last=0;
uint8_t RISE3_height_last=0;

void Draw_grahic_char_2(uint16_t cmd_id, uint16_t data_id, uint16_t tx_id, uint16_t rx_id)
{
	unsigned char CliendTxBuffer[send_max_len];

	uint8_t text1[]="RISE1 -N- RISE2 -N- RISE3 DOWN";/*6-8*//*16-18*//*26-29*/
		
	static ext_draw_ui_character draw;
	static uint32_t alllength;//i
	static uint8_t seq=0;
	draw.head.sof = 0xA5;  
	draw.head.data_length=6+45;
	draw.head.seq=seq;
	memcpy(CliendTxBuffer, &draw.head, sizeof(frame_header_t));//写入帧头数据
	Append_CRC8_Check_Sum(CliendTxBuffer,sizeof(frame_header_t)); //写入帧头校验

	draw.cmd_id = cmd_id;  //机器人间交互ID 301
	
	draw.Client_Custom_ID.data_cmd_id= data_id; //约定的画图ID
	draw.Client_Custom_ID.sender_ID = tx_id;
	draw.Client_Custom_ID.receiver_ID = rx_id;
	
	//自定义内??
	/*二级升降*/
	draw.grapic_data_struct.graphic_name[0]=15;
	draw.grapic_data_struct.operate_tpye=1;
	draw.grapic_data_struct.graphic_tpye=7;/*字符*/
	draw.grapic_data_struct.layer=3;
	draw.grapic_data_struct.color=6;
	draw.grapic_data_struct.start_angle=20;	/*字体大小*/
	draw.grapic_data_struct.end_angle=12;	/*字符长度*/
	draw.grapic_data_struct.width=2;
	draw.grapic_data_struct.start_x=1320;  
	draw.grapic_data_struct.start_y=700;  
	draw.grapic_data_struct.radius=0;	
	draw.grapic_data_struct.end_x=0;
	draw.grapic_data_struct.end_y=0;
	

	// RISE1_height = get_RISE1_height();
	// RISE2_height = get_RISE2_height();
	// RISE3_height = get_RISE3_height();
	
	memcpy(draw.data,&text1,sizeof(text1));
/***************************************************/
	if(RISE1_height_last != RISE1_height)
	{
		if(RISE1_height == 0)
		{
			draw.data[6] = 'M';
			draw.data[7] = 'I';
			draw.data[8] = 'N';
			RISE1_height_last = RISE1_height;
		}
		else if(RISE1_height == 1)
		{
			draw.data[6] = 'M';
			draw.data[7] = 'A';
			draw.data[8] = 'X';
			RISE1_height_last = RISE1_height;
		}
		else if(RISE1_height == 4)
		{
			draw.data[6] = 'R';
			draw.data[7] = 'U';
			draw.data[8] = 'N';
			RISE1_height_last = RISE1_height;
		}
		draw.grapic_data_struct.operate_tpye = 2;
	}
	else 
	{
		if(RISE1_height == 0)
		{
			draw.data[6] = 'M';
			draw.data[7] = 'I';
			draw.data[8] = 'N';
		
		}
		else if(RISE1_height == 1)
		{
			draw.data[6] = 'M';
			draw.data[7] = 'A';
			draw.data[8] = 'X';
			
		}
		else if(RISE1_height == 4)
		{
			draw.data[6] = 'R';
			draw.data[7] = 'U';
			draw.data[8] = 'N';
			
		}
		draw.grapic_data_struct.operate_tpye = 1;
	}
/***************************************************/
	if(RISE2_height_last != RISE2_height)
	{
		if(RISE2_height == 0)
		{
			draw.data[16] = '-';
			draw.data[17] = '1';
			draw.data[18] = '-';
			RISE2_height_last = RISE2_height;
		}
		else if(RISE2_height == 1)
		{
			draw.data[16] = '-';
			draw.data[17] = '2';
			draw.data[18] = '-';
			RISE2_height_last = RISE2_height;
		}
		else if(RISE2_height == 2)
		{
			draw.data[16] = '-';
			draw.data[17] = '3';
			draw.data[18] = '-';
			RISE2_height_last = RISE2_height;
		}
		else if(RISE2_height == 3)
		{
			draw.data[16] = '-';
			draw.data[17] = '4';
			draw.data[18] = '-';
			RISE2_height_last = RISE2_height;
		}
		else if(RISE2_height == 6)
		{
			draw.data[16] = 'R';
			draw.data[17] = 'U';
			draw.data[18] = 'N';
			RISE2_height_last = RISE2_height;
		}
		draw.grapic_data_struct.operate_tpye = 2;
	}
	else 
	{
		if(RISE2_height == 0)
		{
			draw.data[16] = '-';
			draw.data[17] = '1';
			draw.data[18] = '-';
			
		}
		else if(RISE2_height == 1)
		{
			draw.data[16] = '-';
			draw.data[17] = '2';
			draw.data[18] = '-';
			
		}
		else if(RISE2_height == 2)
		{
			draw.data[16] = '-';
			draw.data[17] = '3';
			draw.data[18] = '-';
			
		}
		else if(RISE2_height == 3)
		{
			draw.data[16] = '-';
			draw.data[17] = '4';
			draw.data[18] = '-';
			
		}
		else if(RISE2_height == 6)
		{
			draw.data[16] = 'R';
			draw.data[17] = 'U';
			draw.data[18] = 'N';
			
		}
		draw.grapic_data_struct.operate_tpye = 1;
	}

/***************************************************/
	if(RISE3_height_last != RISE3_height)
	{
		if(RISE3_height == 0)
		{
			draw.data[26] = 'D';
			draw.data[27] = 'O';
			draw.data[28] = 'W';
			draw.data[29] = 'N';
			RISE3_height_last = RISE3_height;
		}
		else if(RISE3_height == 1)
		{
			draw.data[26] = '-';
			draw.data[27] = 'U';
			draw.data[28] = 'P';
			draw.data[29] = '-';
			RISE3_height_last = RISE3_height;
		}
		else if(RISE3_height == 2)
		{
			draw.data[26] = '-';
			draw.data[27] = 'M';
			draw.data[28] = 'A';
			draw.data[29] = 'X';
			RISE3_height_last = RISE3_height;
		}
		draw.grapic_data_struct.operate_tpye = 2;
	}
	else 
	{
		if(RISE3_height == 0)
		{
			draw.data[26] = 'D';
			draw.data[27] = 'O';
			draw.data[28] = 'W';
			draw.data[29] = 'N';
		
		}
		else if(RISE3_height == 1)
		{
			draw.data[26] = '-';
			draw.data[27] = 'U';
			draw.data[28] = 'P';
			draw.data[29] = '-';
		
		}
		else if(RISE3_height == 2)
		{
			draw.data[26] = '-';
			draw.data[27] = 'M';
			draw.data[28] = 'A';
			draw.data[29] = 'X';
		
		}
		draw.grapic_data_struct.operate_tpye = 1;
	}
/*******************************************************************************************************************/
	memcpy(	
			CliendTxBuffer + 5, 
			(uint8_t*)&draw.cmd_id, 
			(sizeof(draw.cmd_id)+ sizeof(draw.Client_Custom_ID)+ sizeof(draw.grapic_data_struct)+sizeof(draw.data))
		  );
	
	Append_CRC16_Check_Sum(CliendTxBuffer,sizeof(draw));

	alllength = sizeof(draw);

//	for(i = 0; i< alllength; i++)
//	{
//HAL_UART_Transmit(&huart6, (uint8_t *)CliendTxBuffer[i], 1, 1000);
//HAL_UART_Transmit 函数是通过轮询方式发送数据，即在发送数据时会阻塞程序的执行，直到数据发送完成或超时。这种方式简单易用，但是效率低下，会占用 CPU 的资源??
//HAL_UART_Transmit_DMA 函数是通过 DMA 方式发送数据，即在启动 DMA 传输后，程序可以继续执行其他任务，直?? DMA 传输完成后触发中断。这种方式复杂一些，需要配?? DMA 相关的参数和中断服务函数，但是效率高，可以释?? CPU 的资??
        USART1_Send((uint8_t *)CliendTxBuffer, 16);
    //    while(__HAL_UART_GET_FLAG(&huart6, UART_FLAG_TC) == RESET);
//	}
    

}
 

