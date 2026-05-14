#include "XV7021BB.h"

void xv7021_CS_Control(XV7021_Handle_t* handle, Bit_OperateType level) 
{
    GPIO_WriteBit(handle->PORT, handle->PIN, level);
}

static uint8_t xv7021_SPI_ReadWrite(SPI_Module* SPIx, uint8_t tx_data) 
{
    while (SPI_I2S_GetStatus(SPIx, SPI_I2S_TE_FLAG) == RESET);
    SPI_I2S_TransmitData(SPIx, tx_data);
    
    while (SPI_I2S_GetStatus(SPIx, SPI_I2S_RNE_FLAG) == RESET);
    return (uint8_t)SPI_I2S_ReceiveData(SPIx);
}

void xv7021_WriteReg(XV7021_Handle_t* handle, uint8_t reg, uint8_t tx_buf) 
{
    xv7021_CS_Control(handle, Bit_RESET);
    xv7021_SPI_ReadWrite(handle->SPIx, reg & 0x7F);
    xv7021_SPI_ReadWrite(handle->SPIx, tx_buf);
    xv7021_CS_Control(handle, Bit_SET);
}

uint8_t xv7021_ReadReg(XV7021_Handle_t* handle, uint8_t reg) 
{
    uint8_t rx_buf;
    xv7021_CS_Control(handle, Bit_RESET);
    xv7021_SPI_ReadWrite(handle->SPIx, reg | 0x80);
    rx_buf = xv7021_SPI_ReadWrite(handle->SPIx, 0x00);
    xv7021_CS_Control(handle, Bit_SET);
    return rx_buf;
}

void xv7021_SendCmd(XV7021_Handle_t* handle, uint8_t cmd) 
{
    xv7021_CS_Control(handle, Bit_RESET);
    xv7021_SPI_ReadWrite(handle->SPIx, cmd);
    xv7021_CS_Control(handle, Bit_SET);
}

int xv7021_Init(XV7021_Handle_t* handle)
{
    uint8_t temp;
    uint8_t buf[2];
	
    xv7021_SendCmd(handle, 0x09); 
    Delay_ms(50); 

    xv7021_WriteReg(handle, XV7021_REG_I2C_OFF, 0x00); 
    Delay_ms(10); 


    xv7021_SendCmd(handle, 0x05); 
    Delay_ms(10);

    
    xv7021_CS_Control(handle, Bit_RESET);
    xv7021_SPI_ReadWrite(handle->SPIx, 0x88); 
    buf[0] = xv7021_SPI_ReadWrite(handle->SPIx, 0x00); 
    buf[1] = xv7021_SPI_ReadWrite(handle->SPIx, 0x00); 
    xv7021_CS_Control(handle, Bit_SET);

    temp = xv7021_ReadReg(handle, XV7021_REG_DSP_CTRL2);
    temp &= 0x80; 
    temp |= 0x4C; 
    xv7021_WriteReg(handle, XV7021_REG_DSP_CTRL2, temp);
    Delay_ms(10);	
	
	xv7021_WriteReg(handle, XV7021_REG_DSP_CTRL3, 0x02);
	Delay_ms(10);
	
    temp = xv7021_ReadReg(handle, XV7021_REG_MODE_CTRL);
    temp |= 0x04; 
    xv7021_WriteReg(handle, XV7021_REG_MODE_CTRL, temp);
    Delay_ms(10);	

    xv7021_SendCmd(handle, 0x06); 
    
    Delay_ms(100); 

    
    xv7021_CS_Control(handle, Bit_RESET);
    xv7021_SPI_ReadWrite(handle->SPIx, 0x88); 
    buf[0] = xv7021_SPI_ReadWrite(handle->SPIx, 0x00); 
    buf[1] = xv7021_SPI_ReadWrite(handle->SPIx, 0x00); 
    xv7021_CS_Control(handle, Bit_SET);
    
    return 0;
}

void xv7021_GetSingleGyroData(XV7021_Handle_t* handle, float* gyro_data)
{
    uint8_t buf[3];
    int32_t raw_24bit;

    xv7021_CS_Control(handle, Bit_RESET);
    xv7021_SPI_ReadWrite(handle->SPIx, XV7021_REG_DATA_START | 0x80);
    buf[0] = xv7021_SPI_ReadWrite(handle->SPIx, 0x00); // 高字节
    buf[1] = xv7021_SPI_ReadWrite(handle->SPIx, 0x00); // 中字节
    buf[2] = xv7021_SPI_ReadWrite(handle->SPIx, 0x00); // 低字节
    xv7021_CS_Control(handle, Bit_SET);

    raw_24bit = ((int32_t)buf[0] << 16) | ((int32_t)buf[1] << 8) | (int32_t)buf[2];
    
    if (raw_24bit & 0x00800000) {
        raw_24bit |= 0xFF000000;
    }
    
    *gyro_data = (float)raw_24bit * XV7021_SCALE_FACTOR_24BIT;
}

void xv7021_GetSingleTemperature(XV7021_Handle_t* handle, float* temperature)
{
    uint8_t buf[2];
	int16_t raw_12bit;

    xv7021_CS_Control(handle, Bit_RESET);
//    for(volatile int i=0; i<20; i++);
//	Delay_ms(3);
    xv7021_SPI_ReadWrite(handle->SPIx, XV7021_REG_TEMP_READ | 0x80); 
    

    buf[0] = xv7021_SPI_ReadWrite(handle->SPIx, 0x00); 
    buf[1] = xv7021_SPI_ReadWrite(handle->SPIx, 0x00); 
    
    xv7021_CS_Control(handle, Bit_SET);

    raw_12bit = (int16_t)((uint16_t)buf[0] << 8 | (uint16_t)buf[1]) >> 4;

    *temperature = 25.0f + ((float)raw_12bit - 400.0f) / 16.0f;
}

void xv7021_GetGyroData(XV7021_Handle_t* handle_1, XV7021_Handle_t* handle_2, XV7021_Handle_t* handle_3, XV7021_DataTpydef* data)
{
    xv7021_GetSingleGyroData(handle_1, &data->gyro[0]);
    xv7021_GetSingleGyroData(handle_2, &data->gyro[1]);
    xv7021_GetSingleGyroData(handle_3, &data->gyro[2]);
}

void xv7021_GetTemperatureData(XV7021_Handle_t* handle_1, XV7021_Handle_t* handle_2, XV7021_Handle_t* handle_3, XV7021_DataTpydef* data)
{
    xv7021_GetSingleTemperature(handle_1, &data->temp[0]);
    xv7021_GetSingleTemperature(handle_2, &data->temp[1]);
    xv7021_GetSingleTemperature(handle_3, &data->temp[2]);
}
