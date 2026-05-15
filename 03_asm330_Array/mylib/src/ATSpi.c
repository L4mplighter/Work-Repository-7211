/*************************************/
/*           CREATED BY WJY          */
/*            2024/9/26              */
/*************************************/
#include "ATSpi.h"

uint16_t at_spi_transmit_receive16(spi_type* spi_x, uint16_t ptxdata) {
    uint16_t prxdata = 0;
    while((spi_x->sts_bit.tdbe) == 0x00);
    spi_x->dt = ptxdata;
    while((spi_x->sts_bit.rdbf) == 0x00);
    prxdata = spi_x->dt;
    return prxdata;
}

uint8_t at_spi_transmit_receive8(spi_type* spi_x, uint8_t ptxdata) {
    uint8_t prxdata = 0;
    while((spi_x->sts_bit.tdbe) == 0x00);
    spi_x->dt = ptxdata;
    while((spi_x->sts_bit.rdbf) == 0x00);
    prxdata = spi_x->dt;
    return prxdata;
}



