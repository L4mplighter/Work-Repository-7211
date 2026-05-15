#ifndef __ASM330LHHX_H_
#define __ASM330LHHX_H_
#include "system.h"
#include "at32f435_437.h"
//#include "main.h"
typedef enum
{
  ASM_OK       = 0x00U,
  ASM_ERROR    = 0x01U,
  ASM_BUSY     = 0x02U,
  ASM_TIMEOUT  = 0x03U
} ASM_StatusTypeDef;

typedef struct {
    float acc[3];
    float gyro[3];
    float temp;
} ASM330DATAOP;

typedef struct {
    uint8_t out_temp_l;
    uint8_t out_temp_h;
    uint8_t outx_l_g;
    uint8_t outx_h_g;
    uint8_t outy_l_g;
    uint8_t outy_h_g;
    uint8_t outz_l_g;
    uint8_t outz_h_g;
    uint8_t outx_l_a;
    uint8_t outx_h_a;
    uint8_t outy_l_a;
    uint8_t outy_h_a;
    uint8_t outz_l_a;
    uint8_t outz_h_a;
} ASM330DATARAW;

#define SENSOR_NUM                      4

#define ASM330_ID                      0x6B
#define ASM330_SENSOR_1                 1
#define ASM330_SENSOR_2                 2
#define ASM330_SENSOR_3                 3
#define ASM330_SENSOR_4                 4
#define ASM330PORT                      SPI1
#define ASM330READ(reg)                 (reg | 0x80)
#define ASM330WRITE(reg)                (reg & 0x7F)
#define ASM330CS1_H                     (P_AOUT(1) = 1)
#define ASM330CS1_L                     (P_AOUT(1) = 0)
#define ASM330CS2_H                     (P_AOUT(2) = 1)
#define ASM330CS2_L                     (P_AOUT(2) = 0)
#define ASM330CS3_H                     (P_AOUT(3) = 1)
#define ASM330CS3_L                     (P_AOUT(3) = 0)
#define ASM330CS4_H                     (P_AOUT(4) = 1)
#define ASM330CS4_L                     (P_AOUT(4) = 0)
#define FUNC_CFG_ACCESS                 0x01
#define PIN_CTRL                        0x02
#define FIFO_CTRL1                      0x07
#define FIFO_CTRL2                      0x08
#define FIFO_CTRL3                      0x09
#define FIFO_CTRL4                      0x0A
#define COUNTER_BDR_REG1                0x0B
#define COUNTER_BDR_REG2                0x0C
#define INT1_CTRL                       0x0D
#define INT2_CTRL                       0x0E
#define WHO_AM_I                        0x0F
#define CTRL1_XL                        0x10
#define CTRL2_G                         0x11
#define CTRL3_C                         0x12
#define CTRL4_C                         0x13
#define CTRL5_C                         0x14
#define CTRL6_C                         0x15
#define CTRL7_G                         0x16
#define CTRL8_XL                        0x17
#define CTRL9_XL                        0x18
#define CTRL10_C                        0x19
#define ALL_INT_SRC                     0x1A
#define WAKE_UP_SRC                     0x1B
#define D6D_SRC                         0x1D
#define STATUS_REG                      0x1E
#define OUT_TEMP_L                      0x20
#define OUT_TEMP_H                      0x21
#define OUTX_L_G                        0x22
#define OUTX_H_G                        0x23
#define OUTY_L_G                        0x24
#define OUTY_H_G                        0x25
#define OUTZ_L_G                        0x26
#define OUTZ_H_G                        0x27
#define OUTX_L_XL                       0x28
#define OUTX_H_XL                       0x29
#define OUTY_L_XL                       0x2A
#define OUTY_H_XL                       0x2B
#define OUTZ_L_XL                       0x2C
#define OUTZ_H_XL                       0x2D
#define EMB_FUNC_STATUS_MAINPAGE        0x35
#define FSM_STATUS_A_MAINPAGE           0x36
#define FSM_STATUS_B_MAINPAGE           0x37
#define MLC_STATUS_MAINPAGE             0x38
#define STATUS_MASTER_MAINPAGE          0x39
#define FIFO_STATUS1                    0x3A
#define FIFO_STATUS2                    0x3B
#define TIMESTAMP0_REG                  0x40
#define TIMESTAMP1_REG                  0x41
#define TIMESTAMP2_REG                  0x42
#define TIMESTAMP3_REG                  0x43
#define INT_CFG0                        0x56
#define INT_CFG1                        0x58
#define THS_6D                          0x59
#define WAKE_UP_THS                     0x5B
#define WAKE_UP_DUR                     0x5C
#define FREE_FALL                       0x5D
#define MD1_CFG                         0x5E
#define MD2_CFG                         0x5F
#define I3C_BUS_AVB                     0x62
#define INTERNAL_FREQ_FINE              0x63
#define X_OFS_USR                       0x73
#define Y_OFS_USR                       0x74
#define Z_OFS_USR                       0x75
#define FIFO_DATA_OUT_TAG               0x78
#define FIFO_DATA_OUT_X_L               0x79
#define FIFO_DATA_OUT_X_H               0x7A
#define FIFO_DATA_OUT_Y_L               0x7B
#define FIFO_DATA_OUT_Y_H               0x7C
#define FIFO_DATA_OUT_Z_L               0x7D
#define FIFO_DATA_OUT_Z_H               0x7E

ASM_StatusTypeDef asm330lhhx_init(void);
void asm330lhhx_data(spi_type* spi_x, ASM330DATAOP *asmdata, uint8_t sensorNum);
#endif
