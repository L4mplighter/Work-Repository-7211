/*************************************/
/*           CREATED BY WJY          */
/*            2024/9/26              */
/*************************************/
#ifndef __ATSPI_H_
#define __ATSPI_H_
#include "at32f435_437.h"

uint16_t at_spi_transmit_receive16(spi_type* spi_x, uint16_t ptxdata);

uint8_t at_spi_transmit_receive8(spi_type* spi_x, uint8_t ptxdata);

#endif
