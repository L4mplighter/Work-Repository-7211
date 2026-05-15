/*************************************/
/*           CREATED BY WJY          */
/*            2024/10/29             */
/*************************************/
#ifndef __OPTION_H_
#define __OPTION_H_

#include "at32f435_437.h"

#define output_50hz 50
#define output_100hz 100
#define output_200hz 200

typedef struct {
    uint8_t Protocol;
    uint8_t BdEnableFlag;
    uint8_t BdEnableFlag2;
    uint8_t UartEnableFlag;
    uint8_t SampleFlag;
    uint8_t OutputRatio;
    uint32_t ComRate;
    float Drift[3];
    float Bias[3];
    float RotationAngle[3];
    uint8_t BandWidth;
    uint16_t GyrpRange;
} optionINFO;

extern optionINFO WG200G;
extern uint8_t settingRate;
extern uint8_t hz200_cnt;
extern uint8_t hz100_cnt;
extern uint8_t hz50_cnt;
flash_status_type option_init(void);
void option_read(optionINFO *opINFO);
flash_status_type option_erase(void);
flash_status_type option_write(optionINFO opINFO);
void set_baud(uint32_t baud);
void set_output_rate(uint8_t OR);
void set_uart_ed(uint8_t UartEnableFlag);
void use_default(optionINFO *opINFO);
void command_decode(const char *commandBuf, const uint8_t commandSize);
void printOption(optionINFO opINFO);
void printSerialNumber(void);
#endif


