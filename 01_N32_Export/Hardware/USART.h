#ifndef __USART_H__
#define __USART_H__

#include "n32g4fr.h"                    // Device header
#include "n32g4fr_rcc.h"
#include "n32g4fr_gpio.h"
#include "n32g4fr_usart.h"
#include "n32g4fr_dma.h"
#include <string.h>
#include "ADXL359.h"
#include "XV7021BB.h"

void USART3_Init(uint32_t baudrate);

void USART3_DMA_Send(uint8_t* tx_buf, uint32_t length);

void DataSend_43Byte(void);

#endif
