/*************************************/
/*           CREATED BY WJY          */
/*            2024/9/26              */
/*************************************/
#include "ATUsart.h"

__INLINE void at_usart_transmit(usart_type *usart_x, uint8_t *data, uint16_t length) {
    int i = 0;
    for (i = 0; i < length; i++) {
        usart_x->dt = (data[i] & 0x01FF);
        while((usart_x->sts_bit.tdc) == 0x00);
        usart_x->sts_bit.tdc = 0x00;
    }
}

void at_usart_receive(usart_type *usart_x, uint8_t *data, uint16_t length) {

}
