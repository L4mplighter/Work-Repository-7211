/*************************************/
/*           CREATED BY WJY          */
/*            2024/9/29              */
/*************************************/
#include "adxl375.h"
#include "system.h"
#include "ATSpi.h"
#include "wk_system.h"

spi_regs_data_t adxl375_regs_data;

uint8_t imu_data[6] = {0x00};

static uint8_t ADXL375_ReadWriteByte(uint8_t cmd) {
    uint8_t rev = 0;
//    ADXL375_CS_L;
    rev = at_spi_transmit_receive8(SPI2, cmd);
//    ADXL375_CS_H;
    return rev;
}

static void ADXL375_Write_Reg(uint8_t regAddr, uint8_t regData) {
    ADXL375_CS_L;
    ADXL375_ReadWriteByte(regAddr&0x7f);
    ADXL375_ReadWriteByte(regData);
    ADXL375_CS_H;
}

uint8_t ADXL375_Read_Reg(uint8_t regAddr) {
    uint8_t rev = 0;
    ADXL375_CS_L;
    ADXL375_ReadWriteByte(regAddr|0x80|0x40);
    rev = ADXL375_ReadWriteByte(0x00);
    ADXL375_CS_H;
    return rev;
}

uint8_t checkByte = 0;

void ADXL375_Master_Init(void) {
    ADXL375_CS_H;
    uint8_t deviceid = 0;
    ADXL375_CS_L;
    ADXL375_ReadWriteByte(ADXL375_REG_DEVICE_ID|0x80);
    deviceid = ADXL375_ReadWriteByte(0x00);
    ADXL375_CS_H;
    if(deviceid != ADXL350_ID) {
        return;
    }
    ADXL375_Write_Reg(ADXL375_REG_DATA_RATE_AND_POWER_MODE_CONTROL, 0b00001011);
    
    
    ADXL375_Write_Reg(ADXL375_REG_INTERRUPT_ENABLE_CONTROL, 0x00);
    
    
    ADXL375_Write_Reg(ADXL375_REG_POWER_SAVING_FEATURE_CONTROL, 0x08);

    
    ADXL375_Write_Reg(ADXL375_REG_DATA_FORMAT_CONTROL, 0b00001011);
    
}

void ADXL375_Master_Read_Regs(void) {
    ADXL375_CS_L;
    ADXL375_ReadWriteByte(0x32|0x80|0x40);
    for(int i = 0; i < 6; i++) {
        imu_data[i] = ADXL375_ReadWriteByte(0x00);
    }
    ADXL375_CS_H;
    adxl375_regs_data.accel[0] = (float)(((int16_t)(imu_data[1] << 8 | imu_data[0]) * 49 / 1000.0f));
    adxl375_regs_data.accel[1] = (float)(((int16_t)(imu_data[3] << 8 | imu_data[2]) * 49 / 1000.0f));
    adxl375_regs_data.accel[2] = (float)(((int16_t)(imu_data[5] << 8 | imu_data[4]) * 49 / 1000.0f));
}

