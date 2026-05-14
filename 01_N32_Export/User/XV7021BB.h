#ifndef __XV7021BB_H__
#define __XV7021BB_H__

#include "n32g4fr.h"
#include "Delay.h"

#define Dummy                   0xFF

#define XV7021_REG_STATUS       0x04
#define XV7021_REG_DATA_START   0x0A
#define XV7021_REG_DSP_CTRL1    0x01
#define XV7021_REG_DSP_CTRL2    0x02
#define XV7021_REG_DSP_CTRL3    0x03
#define XV7021_REG_MODE_CTRL    0x0B
#define XV7021_REG_TEMP_READ    0x08

#define XV7021_REG_I2C_OFF      0x1F

#define XV7021_CMD_SLEEP_IN     0x05
#define XV7021_CMD_SLEEP_OUT    0x06
#define XV7021_CMD_SW_RESET		0x09

#define XV7021_TEMP_BASE        25.0f
#define XV7021_TEMP_OFFSET      400.0f
#define XV7021_TEMP_SCALE       16.0f

#define XV7021_SCALE_FACTOR_24BIT  (1.0f / 17920.0f)

typedef struct {
    SPI_Module* SPIx;
    GPIO_Module* PORT;
    uint16_t PIN;
} XV7021_Handle_t;


typedef struct {
    float gyro[3];
    float temp[3];
} XV7021_DataTpydef;

void xv7021_WriteReg(XV7021_Handle_t* handle, uint8_t reg, uint8_t tx_buf);
void xv7021_CS_Control(XV7021_Handle_t* handle, Bit_OperateType level);
uint8_t xv7021_ReadReg(XV7021_Handle_t* handle, uint8_t reg);
void xv7021_SendCmd(XV7021_Handle_t* handle, uint8_t cmd);

int xv7021_Init(XV7021_Handle_t* handle);

void xv7021_GetSingleGyroData(XV7021_Handle_t* handle, float* data);

void xv7021_GetSingleTemperature(XV7021_Handle_t* handle, float* temperature);

void xv7021_GetGyroData(XV7021_Handle_t* handle_1, XV7021_Handle_t* handle_2, XV7021_Handle_t* handle_3, XV7021_DataTpydef* data);

void xv7021_GetTemperatureData(XV7021_Handle_t* handle_1, XV7021_Handle_t* handle_2, XV7021_Handle_t* handle_3, XV7021_DataTpydef* data);

#endif
