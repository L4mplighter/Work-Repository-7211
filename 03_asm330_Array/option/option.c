/*************************************/
/*           CREATED BY WJY          */
/*            2024/10/29             */
/*************************************/
#include "option.h"
#include "ATFlash.h"
#include "stdio.h"
#include "ATUsart.h"
#include "string.h"
#include "at32f435_437_wk_config.h"

flash_status_type option_init(void) {
    flash_status_type res = FLASH_OPERATE_DONE;
    if(option_flag == *(uint32_t *)option_address) {
        option_read(&WG200G);
    } else {
        use_default(&WG200G);
        option_write(WG200G);
    }
    return res;
}

void option_read(optionINFO *opINFO) {
    opINFO->BandWidth = *(uint8_t *)BandWidth_address;
    opINFO->BdEnableFlag = *(uint8_t *)BdEnableFlag_address;
    opINFO->BdEnableFlag2 = *(uint8_t *)BdEnableFlag2_address;
    opINFO->Bias[0] = *(float *)Bias_address;
    opINFO->Bias[1] = *(float *)(Bias_address + 4);
    opINFO->Bias[2] = *(float *)(Bias_address + 8);
    opINFO->ComRate = *(uint32_t *)ComRate_address;
    opINFO->Drift[0] = *(float *)Drift_address;
    opINFO->Drift[1] = *(float *)(Drift_address + 4);
    opINFO->Drift[2] = *(float *)(Drift_address + 8);
    opINFO->GyrpRange = *(uint16_t *)GyrpRange_address;
    opINFO->OutputRatio = *(uint8_t *)OutputRatio_address;
    opINFO->Protocol = *(uint8_t *)Protocol_address;
    opINFO->RotationAngle[0] = *(float *)RotationAngle_address;
    opINFO->RotationAngle[1] = *(float *)(RotationAngle_address + 4);
    opINFO->RotationAngle[2] = *(float *)(RotationAngle_address + 8);
    opINFO->SampleFlag = *(uint8_t *)SampleFlag_address;
    opINFO->UartEnableFlag = *(uint8_t *)UartEnableFlag_address;
}

flash_status_type option_erase(void) {
    flash_status_type res = FLASH_OPERATE_DONE;
    flash_unlock();
    flash_sector_erase(option_address);
    res = flash_bank1_operation_status_get();
    flash_lock();
    return res;
}

flash_status_type option_write(optionINFO opINFO) {
    flash_status_type res = FLASH_OPERATE_DONE;
    option_erase();
    flash_unlock();
    if(FLASH_OPERATE_DONE != flash_word_program(option_address, option_flag)) return res;
    if(FLASH_OPERATE_DONE != flash_word_program(ComRate_address, WG200G.ComRate)) return res;
    if(FLASH_OPERATE_DONE != flash_word_program(Drift_address, WG200G.Drift[0])) return res;
    if(FLASH_OPERATE_DONE != flash_word_program((Drift_address + 4), WG200G.Drift[1])) return res;
    if(FLASH_OPERATE_DONE != flash_word_program((Drift_address + 8), WG200G.Drift[2])) return res;
    if(FLASH_OPERATE_DONE != flash_word_program(Bias_address, WG200G.Bias[0])) return res;
    if(FLASH_OPERATE_DONE != flash_word_program((Bias_address + 4), WG200G.Bias[1])) return res;
    if(FLASH_OPERATE_DONE != flash_word_program((Bias_address + 8), WG200G.Bias[2])) return res;
    if(FLASH_OPERATE_DONE != flash_word_program(RotationAngle_address, WG200G.RotationAngle[0])) return res;
    if(FLASH_OPERATE_DONE != flash_word_program((RotationAngle_address + 4), WG200G.RotationAngle[1])) return res;
    if(FLASH_OPERATE_DONE != flash_word_program((RotationAngle_address + 8), WG200G.RotationAngle[2])) return res;
    
    if(FLASH_OPERATE_DONE != flash_byte_program(Protocol_address, WG200G.Protocol)) return res;
    if(FLASH_OPERATE_DONE != flash_byte_program(BdEnableFlag_address, WG200G.BdEnableFlag)) return res;
    if(FLASH_OPERATE_DONE != flash_byte_program(BdEnableFlag2_address, WG200G.BdEnableFlag2)) return res;
    if(FLASH_OPERATE_DONE != flash_byte_program(UartEnableFlag_address, WG200G.UartEnableFlag)) return res;
    if(FLASH_OPERATE_DONE != flash_byte_program(SampleFlag_address, WG200G.SampleFlag)) return res;
    if(FLASH_OPERATE_DONE != flash_byte_program(OutputRatio_address, WG200G.OutputRatio)) return res;
    if(FLASH_OPERATE_DONE != flash_byte_program(BandWidth_address, WG200G.BandWidth)) return res;
    
    if(FLASH_OPERATE_DONE != flash_halfword_program(GyrpRange_address, WG200G.GyrpRange)) return res;
    flash_lock();
    return res;

}

void command_decode(const char *commandBuf, const uint8_t commandSize) {
    switch(commandSize) {
        case 6: 
            if(commandBuf[3] == 'E') {
                   set_uart_ed(1); 
                } else if(commandBuf[3] == 'D') {
                    set_uart_ed(0);
                } else if(commandBuf[3] == 'I') {
                    printOption(WG200G);
                } else if(commandBuf[3] == 'S') {
                    printSerialNumber();
                }
                break;
        case 7:
            if(commandBuf[6] == '1') {
                set_baud(115200);
            } else if(commandBuf[6] == '2') {
                set_baud(230400);
            } else if(commandBuf[6] == '9') {
                set_baud(921600);
            }
            break;
        case 10:
            if(commandBuf[3] == 'R') {
                settingRate = (uint8_t)(commandBuf[commandSize - 1] - '0');
                WG200G.OutputRatio = settingRate;
                hz200_cnt = 0;
                hz100_cnt = 0;
                hz50_cnt = 0;
            }
        default:
            break;
    }
    option_write(WG200G);
}

void set_baud(uint32_t baud) {
    WG200G.ComRate = baud;
    wk_usart1_init();
}

void set_output_rate(uint8_t OR) {
    WG200G.OutputRatio = OR;
}

void set_uart_ed(uint8_t UartEnableFlag) {
    WG200G.UartEnableFlag = UartEnableFlag;
}

void printOption(optionINFO opINFO) {
    char INFO[50] = {0x00};
    sprintf(INFO, "Protocol: %d\n", opINFO.Protocol);
    at_usart_transmit(USART1, (uint8_t *)INFO, strlen(INFO));
    memset(INFO, 0x00, sizeof(INFO));
    sprintf(INFO, "BdEnableFlag2: %d\n", opINFO.BdEnableFlag);
    at_usart_transmit(USART1, (uint8_t *)INFO, strlen(INFO));
    memset(INFO, 0x00, sizeof(INFO));
    sprintf(INFO, "BdEnableFlag2: %d\n", opINFO.BdEnableFlag2);
    at_usart_transmit(USART1, (uint8_t *)INFO, strlen(INFO));
    memset(INFO, 0x00, sizeof(INFO));
    sprintf(INFO, "UartEnableFlag: %d\n", opINFO.UartEnableFlag);
    at_usart_transmit(USART1, (uint8_t *)INFO, strlen(INFO));
    memset(INFO, 0x00, sizeof(INFO));
    sprintf(INFO, "SampleFlag: %d\n", opINFO.SampleFlag);
    at_usart_transmit(USART1, (uint8_t *)INFO, strlen(INFO));
    memset(INFO, 0x00, sizeof(INFO));
    sprintf(INFO, "OutputRatio: 0%d\n", opINFO.OutputRatio);
    at_usart_transmit(USART1, (uint8_t *)INFO, strlen(INFO));
    memset(INFO, 0x00, sizeof(INFO));
    sprintf(INFO, "ComRate: %d\n", opINFO.ComRate);
    at_usart_transmit(USART1, (uint8_t *)INFO, strlen(INFO));
    memset(INFO, 0x00, sizeof(INFO));
    sprintf(INFO, "Drift: %.6f %.6f %.6f\n", opINFO.Drift[0], opINFO.Drift[1], opINFO.Drift[2]);
    at_usart_transmit(USART1, (uint8_t *)INFO, strlen(INFO));
    memset(INFO, 0x00, sizeof(INFO));
    sprintf(INFO, "Bias: %.6f %.6f %.6f\n", opINFO.Bias[0], opINFO.Bias[1], opINFO.Bias[2]);
    at_usart_transmit(USART1, (uint8_t *)INFO, strlen(INFO));
    memset(INFO, 0x00, sizeof(INFO));
    sprintf(INFO, "RotationAngle: %.6f %.6f %.6f\n", opINFO.RotationAngle[0], opINFO.RotationAngle[1], opINFO.RotationAngle[2]);
    at_usart_transmit(USART1, (uint8_t *)INFO, strlen(INFO));
    memset(INFO, 0x00, sizeof(INFO));
    sprintf(INFO, "BandWidth: %d\n", opINFO.BandWidth);
    at_usart_transmit(USART1, (uint8_t *)INFO, strlen(INFO));
    memset(INFO, 0x00, sizeof(INFO));
    sprintf(INFO, "GyroRange: %d\n", opINFO.GyrpRange);
    at_usart_transmit(USART1, (uint8_t *)INFO, strlen(INFO));
    memset(INFO, 0x00, sizeof(INFO));
}

void printSerialNumber(void) {

}

void use_default(optionINFO *opINFO) {
    opINFO->Protocol = 2;
    opINFO->BdEnableFlag = 1;
    opINFO->BdEnableFlag2 = 1;
    opINFO->UartEnableFlag = 1;
    opINFO->SampleFlag = 0;
    opINFO->OutputRatio = 4;
    opINFO->ComRate = 230400;
    opINFO->Drift[0] = 0.000000;
    opINFO->Drift[1] = 0.000000;
    opINFO->Drift[2] = 0.000000;
    opINFO->Bias[0] = 0.000000;
    opINFO->Bias[1] = 0.000000;
    opINFO->Bias[2] = 0.000000;
    opINFO->RotationAngle[0] = 0.000000;
    opINFO->RotationAngle[1] = 0.000000;
    opINFO->RotationAngle[2] = 0.000000;
    opINFO->BandWidth = 50;
    opINFO->GyrpRange = 200;
}

