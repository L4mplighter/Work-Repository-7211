#include "usart.h"

extern XV7021_DataTpydef xv7021;
extern ADXL_DataTypdef adxl359;

uint8_t Txbuf[43] = {0};
uint16_t zero;
/**
 * PB10     USART3_TX
 * PB11     USART3_RX
 * DMA1_CH2 USART3
 */
void USART3_Init(uint32_t baudrate)
{
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_USART3, ENABLE);
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB, ENABLE);
    RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_DMA1, ENABLE);

    GPIO_InitType GPIO_InitStructure;
    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.Pin = GPIO_PIN_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = GPIO_PIN_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);

    DMA_InitType DMA_InitStructure;
    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.PeriphAddr = (USART3_BASE + 0x04);
    DMA_InitStructure.MemAddr = (uint32_t)Txbuf;
    DMA_InitStructure.Direction = DMA_DIR_PERIPH_DST;
    DMA_InitStructure.BufSize = sizeof(Txbuf);
    DMA_InitStructure.PeriphInc = DMA_PERIPH_INC_DISABLE;
    DMA_InitStructure.DMA_MemoryInc = DMA_MEM_INC_ENABLE;
    DMA_InitStructure.PeriphDataSize = DMA_PERIPH_DATA_SIZE_BYTE;
    DMA_InitStructure.MemDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.CircularMode = DMA_MODE_NORMAL;
    DMA_InitStructure.Priority = DMA_PRIORITY_VERY_HIGH;
    DMA_InitStructure.Mem2Mem = DMA_M2M_DISABLE;
    DMA_Init(DMA1_CH2, &DMA_InitStructure);

    DMA_ConfigInt(DMA1_CH2, DMA_INT_TXC, ENABLE);

    USART_InitType USART_InitStructure;
    USART_StructInit(&USART_InitStructure);
    USART_InitStructure.BaudRate = baudrate;
    USART_InitStructure.Mode = USART_MODE_RX | USART_MODE_TX;
    USART_InitStructure.Parity = USART_PE_NO;
    USART_InitStructure.StopBits = USART_STPB_1;
    USART_InitStructure.WordLength = USART_WL_8B;
    USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
    USART_Init(USART3, &USART_InitStructure);

    NVIC_InitType NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure);

    USART_EnableDMA(USART3, USART_DMAREQ_TX, ENABLE);
    // DMA_EnableChannel(DMA1_CH2, ENABLE);
    USART_Enable(USART3, ENABLE);
}

void USART3_DMA_Send(uint8_t* tx_buf, uint32_t length)
{
    DMA_EnableChannel(DMA1_CH2, DISABLE);
    DMA1_CH2->MADDR = (uint32_t)tx_buf;
    DMA_SetCurrDataCounter(DMA1_CH2, length);
    DMA_ClearFlag(DMA1_FLAG_TC2, DMA1);
    DMA_EnableChannel(DMA1_CH2, ENABLE);

}

void DataSend_43Byte(void)
{   
   uint16_t check_sum = 0;

   Txbuf[0] = 0xAA;
   Txbuf[1] = 0x55;
   memcpy(&Txbuf[2], &xv7021.gyro[0],4);
   memcpy(&Txbuf[6], &xv7021.gyro[1],4);
   memcpy(&Txbuf[10],&xv7021.gyro[2],4);
   memcpy(&Txbuf[14],&adxl359.acc[0],4);
   memcpy(&Txbuf[18],&adxl359.acc[1],4);
   memcpy(&Txbuf[22],&adxl359.acc[2],4);
   memcpy(&Txbuf[38],&zero, 4);
   for (int i=2; i<= 41; i++)
   {
       check_sum = check_sum + Txbuf[i];
   }
   Txbuf[42] = (uint8_t) (check_sum & 0x00FF);

   USART3_DMA_Send(Txbuf, 43);   
}
