#include "PC_communication_task.h"
#include "MCU_communicaton_task.h"  
#include "CRC8_CRC16.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "bsp_usart.h"
visuals_need_data_t *visual_get;
visuals_tx_t visuals_tx;
void visual_send_task(void const *parmas)
{
    #if chassisControlBorad
    visual_get = (visuals_need_data_t *)get_Visual_need_data_point();   
    visuals_tx.header = 0xaa;
    visuals_tx.tailer = 0x70;
    while(1)
    {
        visuals_tx.data.identify_target_flag = 0;
        for (uint8_t i = 0; i < 4; i++)
        {
            visuals_tx.data.position1[i] = visual_get->motor_position1[i];
            visuals_tx.data.position2[i] = visual_get->motor_position2[i];
            visuals_tx.data.position3[i] = visual_get->motor_position3[i];
            visuals_tx.data.position4[i] = visual_get->motor_position4[i];
            visuals_tx.data.position5[i] = visual_get->motor_position5[i];
            visuals_tx.data.position6[i] = visual_get->motor_position6[i];
            visuals_tx.data.position7[i] = visual_get->motor_position7[i];
        }

        visuals_tx.CRC_16 = get_CRC16_check_sum((uint8_t *)&visuals_tx.data, 40 ,visuals_tx.CRC_16);
        USART1_Send((uint8_t *)&visuals_tx , sizeof(visuals_tx));
        vTaskDelay(33);
    }
    #endif
  
} 
uint32_t visual_receive_length;
visuals_rx_t Transmission_Visual;
static visuals_rx_data_t visuals_rx_data_classis;
const visuals_rx_data_t *get_visuals_rx_data_classis_point(void)
{
return &visuals_rx_data_classis;
}

const visuals_rx_t *get_visuals_rx_point(void)
{
    return &Transmission_Visual;
}
void visual_receive_task(void const *pvParameters) 
{
    #if chassisControlBorad
    while(1)
    {
        visual_receive:
        visual_receive_length = USART1_GetDataCount();
        
        if(visual_receive_length >= 10)
        {
            if(USART1_At(0) == 0xaa && USART1_At(43) == 0x70)
            {
				USART1_Recv((unsigned char *)&Transmission_Visual, sizeof(Transmission_Visual));
                Transmission_Visual_measure_visuals_rx(Transmission_Visual , &visuals_rx_data_classis);
                USART1_Drop(4096);
            }
            else
            {
                USART1_Drop(1);
                vTaskDelay(1);
                
                visual_receive_length = USART1_GetDataCount();
                if(visual_receive_length > 3000)
                {
                    USART1_Drop(4096);
                }
                
                goto visual_receive;
            }
        }
        else
        {
            USART1_Drop(1);
            for(;;)
            {
                visual_receive_length =  USART1_GetDataCount();
                if( visual_receive_length > 0)
                {
                    if(USART1_At(0) == 0xaa && USART1_At(43) == 0x80)
                    {
                        break;
                    }
                    else
                    {
                        USART1_Drop(1);
                        vTaskDelay(1);
                    }
                    
                    visual_receive_length =  USART1_GetDataCount();
                    if(visual_receive_length > 3000)
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
    vTaskDelay(15);
    }
    #endif
}
















