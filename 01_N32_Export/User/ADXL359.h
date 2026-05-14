#ifndef __ADXL359_H__
#define __ADXL359_H__

#include "n32g4fr.h"
#include "n32g4fr_gpio.h"
#include "Delay.h"

#define Dummy	0xFF
#define G_	9.8f

#define ADXL359_REG_DEVID_AD     0x00  
#define ADXL359_REG_DEVID_MST    0x01  
#define ADXL359_REG_PARTID       0x02  
#define ADXL359_REG_STATUS       0x04  
#define ADXL359_REG_FILTER       0x28  
#define ADXL359_REG_RANGE        0x2C  
#define ADXL359_REG_POWER_CTL    0x2D  
#define ADXL359_REG_RESET        0x2F  
#define ADXL359_REG_STATUS       0x04 
#define ADXL359_REG_TEMP2        0x06 
#define ADXL359_REG_XDATA3       0x08 

#define ADXL359_RESET_CMD        0x52
#define ADXL359_RANGE_40G        0x03

#define ADXL359_FILTER_HPF_OFF   (0x00 << 4)
#define ADXL359_FILTER_ODR_2000  0x01  
#define ADXL359_FILTER_SETTING   (ADXL359_FILTER_HPF_OFF | ADXL359_FILTER_ODR_2000)

#define ADXL359_POWER_MEASURE    0x00

#define ADXL359_SCALE_FACTOR_40G  0.000078f //  78ug/LSB

typedef struct 
{
    SPI_Module* SPIx;
    GPIO_Module* Port;
    uint16_t Pin;

} ADXL359_Handle_t;

typedef struct 
{
    // int32_t raw_x;
    // int32_t raw_y;
    // int32_t raw_z;
    // int32_t raw_temp;
    float acc[3];
    float temp;
}ADXL_DataTypdef;


void adxl_CS_Control(Bit_OperateType level);

int adxl_ReadReg(uint8_t reg, uint16_t size, uint8_t *rx_buf);

int adxl_WriteReg(uint8_t reg, uint16_t size, const uint8_t *tx_buf);

int adxl_Init(void);

int adxl_GetData(ADXL_DataTypdef* data);

#endif
