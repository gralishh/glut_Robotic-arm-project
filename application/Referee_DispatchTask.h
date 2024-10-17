#ifndef REFEREE_DISPATCHTASK_H
#define REFEREE_DISPATCHTASK_H
#include "Referee.h"
#include "main.h"
ext_self_control_t *get_self_control_original_point(void);

void DispchRefereeTask(const void *parmas);
uint8_t GetEenmyColor(void);
uint8_t GetRobotLevel(void);
float Get_shouter_speed(void);
int Get_shouter_freq(void);
float GetRealPower(void);
float GetPowerBuffer(void);
Referee_Date *GetRefereeDataPoint(void);
void SendDataToClient(void);
void Send_UI_data(void);
void Draw_grahic(uint16_t cmd_id, uint16_t data_id, uint16_t tx_id, uint16_t rx_id);
void Draw_grahic_status(uint16_t cmd_id, uint16_t data_id, uint16_t tx_id, uint16_t rx_id);
void Draw_grahic_status_change(uint16_t cmd_id, uint16_t data_id, uint16_t tx_id, uint16_t rx_id);

void Draw_grahic_char_1(uint16_t cmd_id, uint16_t data_id, uint16_t tx_id, uint16_t rx_id);
void Draw_grahic_char_2(uint16_t cmd_id, uint16_t data_id, uint16_t tx_id, uint16_t rx_id);

#endif
