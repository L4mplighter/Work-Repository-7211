#include "main.h"

uint8_t TIMER_1ms = 0;
uint8_t tx_busy = 0;
uint16_t temp_update_cnt = 0;

XV7021_DataTpydef xv7021;
ADXL_DataTypdef adxl359;
float temp = 0;
int32_t debug_flag = 11;
uint8_t debug = 0x0F;

int main(void)
{
	CS_GPIO_Init();
	SPI13_Init();
    USART3_Init(921600);
    
    adxl_Init();
	
	xv7021_Init(&xv_handle_2);	
	TIM5_Init();
	
    while(1)
    {
        if(TIMER_1ms == 1)
        {
            TIMER_1ms = 0;
            temp_update_cnt++;
            if (temp_update_cnt >= 1000)
            {
                temp_update_cnt = 0;
				xv7021_GetSingleTemperature(&xv_handle_2, &temp);
//                xv7021_GetTemperatureData(&xv_handle_1, &xv_handle_2, &xv_handle_3, &xv7021);
//				xv7021_GetSingleTemperature(&xv_handle_2, &temp);
            }
//            xv7021_GetGyroData(&xv_handle_1, &xv_handle_2, &xv_handle_3, &xv7021);
			xv7021_GetSingleGyroData(&xv_handle_2, &xv7021.gyro[0]);
            adxl_GetData(&adxl359);
            if (tx_busy == 0)
            {
                tx_busy = 1;
                DataSend_43Byte();
            }
            
        }
    }
	
}

void DMA1_Channel2_IRQHandler()
{
    DMA_ClearFlag(DMA1_FLAG_TC2, DMA1);
    tx_busy = 0;
}

void TIM5_IRQHandler(void)
{
    if (TIM_GetIntStatus(TIM5, TIM_INT_UPDATE) != RESET)
    {
        TIM_ClrIntPendingBit(TIM5, TIM_INT_UPDATE); 
        TIMER_1ms = 1;
    }
}
