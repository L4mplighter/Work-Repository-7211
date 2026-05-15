/*************************************/
/*           CREATED BY WJY          */
/*            2024/9/26              */
/*************************************/
#ifndef __ATUSART_H_
#define __ATUSART_H_
#include "at32f435_437.h"

void at_usart_transmit(usart_type *usart_x, uint8_t *data, uint16_t length);
void at_usart_receive(usart_type *usart_x, uint8_t *data, uint16_t length);

#endif
