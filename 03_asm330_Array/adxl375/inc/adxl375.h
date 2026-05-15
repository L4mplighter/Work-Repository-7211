/*************************************/
/*           CREATED BY WJY          */
/*            2024/9/29              */
/*************************************/
#ifndef __ADXL375_H_
#define __ADXL375_H_
#ifdef __cplusplus
extern "C" {
#endif
#include "at32f435_437.h"
#include "wk_system.h"
typedef struct {
    float  gyro[3];
    float  accel[3];
    float temp;
} __attribute__((packed)) spi_regs_data_t ;
#define         INIT_TIMEOUT                                    0xA0

#define         ADXL350_ID                                      0xE5

#define         ADXL375_REG_DEVICE_ID                           0x00
#define         ADXL375_REG_SHOCK_THRESHOLD                     0x1D
#define         ADXL375_REG_X_AXIS_OFFSET                       0x1E
#define         ADXL375_REG_Y_AXIS_OFFSET                       0x1F
#define         ADXL375_REG_Z_AXIS_OFFSET                       0x20
#define         ADXL375_REG_SHOCK_DURATION                      0x21
#define         ADXL375_REG_SHOCK_LATENCY                       0x22
#define         ADXL375_REG_SHOCK_WINDOW                        0x23
#define         ADXL375_REG_ACTIVITY_THRESHOLD                  0x24
#define         ADXL375_REG_INACTIVITY_THRESHOLD                0x25
#define         ADXL375_REG_INACTIVITY_TIME                     0x26
#define         ADXL375_REG_AXIS_ENABLE_OR_DISABLE_CONTROL      0x27
#define         ADXL375_REG_AXIS_SHOCK_SINGLE_OR_DOUBLE         0x2A
#define         ADXL375_REG_AXIS_SHOCK_SOURCE                   0x2B
#define         ADXL375_REG_DATA_RATE_AND_POWER_MODE_CONTROL    0x2C
#define         ADXL375_REG_POWER_SAVING_FEATURE_CONTROL        0x2D
#define         ADXL375_REG_INTERRUPT_ENABLE_CONTROL            0x2E
#define         ADXL375_REG_INTERRUPT_MAPPING_CONTROL           0x2F
#define         ADXL375_REG_INTERRUPT_SOURCE                    0x30
#define         ADXL375_REG_DATA_FORMAT_CONTROL                 0x31
#define         ADXL375_REG_X_AXIS_DATA_0                       0x32
#define         ADXL375_REG_X_AXIS_DATA_1                       0x33
#define         ADXL375_REG_Y_AXIS_DATA_0                       0x34
#define         ADXL375_REG_Y_AXIS_DATA_1                       0x35
#define         ADXL375_REG_Z_AXIS_DATA_0                       0x36
#define         ADXL375_REG_Z_AXIS_DATA_1                       0x37
#define         ADXL375_REG_FIFO_CONTROL                        0x38
#define         ADXL375_REG_FIFO_STATUS                         0x39

#define ADXL375_CS_H    (P_BOUT(12) = 1)
#define ADXL375_CS_L    (P_BOUT(12) = 0)

#define ADXL375_DELAY(ms)  (wk_delay_ms(ms))

uint8_t ADXL375_Read_Reg(uint8_t regAddr);

void ADXL375_Master_Init(void);

void ADXL375_Master_Read_Regs(void);

extern spi_regs_data_t  adxl375_regs_data;


#ifdef __cplusplus
}
#endif
#endif