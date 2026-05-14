#include "ADXL359.h"

ADXL359_Handle_t adxl_handle = {SPI1, GPIOA, GPIO_PIN_4};

void adxl_CS_Control(Bit_OperateType level)
{
    GPIO_WriteBit(adxl_handle.Port, adxl_handle.Pin, level);
}

static uint8_t adxl_SPI_ReadWriteByte(uint8_t tx_data)
{
    while(SPI_I2S_GetStatus(adxl_handle.SPIx, SPI_I2S_TE_FLAG) != SET);
    SPI_I2S_TransmitData(adxl_handle.SPIx, (uint16_t)tx_data);

    while(SPI_I2S_GetStatus(adxl_handle.SPIx, SPI_I2S_RNE_FLAG) != SET);
    return (uint8_t)SPI_I2S_ReceiveData(adxl_handle.SPIx);
}

int adxl_ReadReg(uint8_t reg, uint16_t size, uint8_t *rx_buf)
{
    uint16_t idx;
    uint8_t cmd_byte = (reg << 1) | 0x01;
    adxl_CS_Control(Bit_RESET);
    adxl_SPI_ReadWriteByte(cmd_byte);
    for (idx = 0; idx < size; idx++) 
    {
        rx_buf[idx] = adxl_SPI_ReadWriteByte(Dummy);
    }
    adxl_CS_Control(Bit_SET);

    return 0;
}

int adxl_WriteReg(uint8_t reg, uint16_t size, const uint8_t *tx_buf)
{
    uint16_t idx;
    uint8_t cmd_byte = (reg << 1) | 0x00;
    adxl_CS_Control(Bit_RESET);
    adxl_SPI_ReadWriteByte(cmd_byte);
    for (idx = 0; idx < size; idx++) 
    {
        adxl_SPI_ReadWriteByte(tx_buf[idx]);
    }
    adxl_CS_Control(Bit_SET);

    return 0;
}

int adxl_Init(void)
{
    uint8_t tx_buf = 0;
    uint8_t rx_buf = 0;

    tx_buf = ADXL359_RESET_CMD;
    adxl_WriteReg(ADXL359_REG_RESET, 1, &tx_buf);

    Delay_ms(15);

    adxl_ReadReg(ADXL359_REG_DEVID_AD, 1, &rx_buf);
    if (rx_buf != 0xAD) return -1;

    adxl_ReadReg(ADXL359_REG_PARTID, 1, &rx_buf);    
    if (rx_buf != 0xE9) return -1;

    tx_buf = ADXL359_RANGE_40G;
    adxl_WriteReg(ADXL359_REG_RANGE, 1, &tx_buf);

    tx_buf = ADXL359_FILTER_SETTING;
    adxl_WriteReg(ADXL359_REG_FILTER, 1, &tx_buf);    

    tx_buf = ADXL359_POWER_MEASURE;
    adxl_WriteReg(ADXL359_REG_POWER_CTL, 1, &tx_buf);

    Delay_ms(15);

    return 0;
}

int adxl_GetData(ADXL_DataTypdef* data)
{
    uint8_t buf[9] = {0};
    uint32_t temp_val;
    int32_t raw_val;

    adxl_ReadReg(ADXL359_REG_XDATA3, 9, buf); 
    
    temp_val = ((uint32_t)buf[0] << 12) | ((uint32_t)buf[1] << 4) | (buf[2] >> 4);
    if (temp_val & 0x00080000) temp_val |= 0xFFF00000;
    
    raw_val = (int32_t)temp_val; 
    data->acc[0] = (float)raw_val * ADXL359_SCALE_FACTOR_40G * G_; 

    temp_val = ((uint32_t)buf[3] << 12) | ((uint32_t)buf[4] << 4) | (buf[5] >> 4);
    if (temp_val & 0x00080000) temp_val |= 0xFFF00000;
    
    raw_val = (int32_t)temp_val;
    data->acc[1] = (float)raw_val * ADXL359_SCALE_FACTOR_40G * G_;

    temp_val = ((uint32_t)buf[6] << 12) | ((uint32_t)buf[7] << 4) | (buf[8] >> 4);
    if (temp_val & 0x00080000) temp_val |= 0xFFF00000;
    
    raw_val = (int32_t)temp_val;
    data->acc[2] = (float)raw_val * ADXL359_SCALE_FACTOR_40G * G_;

    return 0;
}
