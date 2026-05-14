#include "MMC5983.h"

MMC5983_HandleTypeDef MMC5983_handle = {
    .SPIhandler = &hspi1,
    .CS_GPIOport = GPIOC,
    .CS_GPIOpin = GPIO_PIN_4,
    .INT_CTRL_0_Reg = 0x00,
    .INT_CTRL_1_Reg = 0x00,
    .INT_CTRL_2_Reg = 0x00,
    .INT_CTRL_3_Reg = 0x00
};

MMC5983_InitStructureTypeDef MMC_InitStructure = {
    .BandWidth = MMC_BW_0400,
    .OperationRate = MMC_SET_0500,
    .OutputRate = MMC_ODR_0100,
    .OperationMode = 0x00
};

void disableCS_MMC5983(GPIO_TypeDef *CS_GPIOport, uint16_t CS_GPIOpin)
{
  HAL_GPIO_WritePin(CS_GPIOport, CS_GPIOpin, GPIO_PIN_SET);
}

void enableCS_MMC5983(GPIO_TypeDef *CS_GPIOport, uint16_t CS_GPIOpin)
{
  HAL_GPIO_WritePin(CS_GPIOport, CS_GPIOpin, GPIO_PIN_RESET);
}

MMC5983_Error_TypeDef MMC5983_SingleRegister_Read(MMC5983_HandleTypeDef *MMC5983_Handler, uint8_t MMC5983_Register_Addr, uint8_t *pData)
{
	uint8_t SPITransmitData_MMC;
	SPITransmitData_MMC = (MMC_Read_Command | MMC5983_Register_Addr);

	enableCS_MMC5983(MMC5983_Handler->CS_GPIOport, MMC5983_Handler->CS_GPIOpin);
	if(HAL_SPI_Transmit(MMC5983_Handler->SPIhandler, &SPITransmitData_MMC, 1, HAL_MAX_DELAY) != HAL_OK)
	{
		disableCS_MMC5983(MMC5983_Handler->CS_GPIOport, MMC5983_Handler->CS_GPIOpin);
		return MMC_HAL_ERROR;
	}
	if(HAL_SPI_Receive(MMC5983_Handler->SPIhandler, pData, 1, HAL_MAX_DELAY) != HAL_OK)
	{
		disableCS_MMC5983(MMC5983_Handler->CS_GPIOport, MMC5983_Handler->CS_GPIOpin);
		return MMC_HAL_ERROR;
	}
	disableCS_MMC5983(MMC5983_Handler->CS_GPIOport, MMC5983_Handler->CS_GPIOpin);

	return MMC_NO_ERROR;
}

MMC5983_Error_TypeDef MMC5983_SingleRegister_Write(MMC5983_HandleTypeDef *MMC5983_Handler, uint8_t MMC5983_Register_Addr, uint8_t MMC5983_Write_Data)
{
	uint8_t SPITransmitData_MMC;
	SPITransmitData_MMC = (MMC_Write_command | MMC5983_Register_Addr);

	enableCS_MMC5983(MMC5983_Handler->CS_GPIOport, MMC5983_Handler->CS_GPIOpin);
	if(HAL_SPI_Transmit(MMC5983_Handler->SPIhandler, &SPITransmitData_MMC, 1, HAL_MAX_DELAY) != HAL_OK)
	{
		disableCS_MMC5983(MMC5983_Handler->CS_GPIOport, MMC5983_Handler->CS_GPIOpin);
		return MMC_HAL_ERROR;
	}
	if(HAL_SPI_Transmit(MMC5983_Handler->SPIhandler, &MMC5983_Write_Data, 1, HAL_MAX_DELAY) != HAL_OK)
	{
		disableCS_MMC5983(MMC5983_Handler->CS_GPIOport, MMC5983_Handler->CS_GPIOpin);
		return MMC_HAL_ERROR;
	}
	disableCS_MMC5983(MMC5983_Handler->CS_GPIOport, MMC5983_Handler->CS_GPIOpin);

	return MMC_NO_ERROR;
}

MMC5983_Error_TypeDef MMC5983_SW_Reset(MMC5983_HandleTypeDef *MMC5983_Handler)
{

	if(MMC5983_SingleRegister_Write(MMC5983_Handler, MMC_INTERNAL_CONTROL_1, MMC5983_SW_Reset_Mask) != MMC_NO_ERROR)
	{
		return MMC_HAL_ERROR;
	}
	HAL_Delay(15);

	MMC5983_Handler->INT_CTRL_0_Reg = 0x00;
	MMC5983_Handler->INT_CTRL_1_Reg = 0x00;
	MMC5983_Handler->INT_CTRL_2_Reg = 0x00;
	MMC5983_Handler->INT_CTRL_3_Reg = 0x00;
	return (MMC_NO_ERROR);
}

MMC5983_Error_TypeDef MMC5983_Set_Bandwidth(MMC5983_HandleTypeDef *MMC5983_Handler, uint8_t MMC5983_BW_Rate){

	MMC5983_Handler->INT_CTRL_1_Reg = MMC5983_Handler->INT_CTRL_1_Reg | MMC5983_BW_Rate;

	if(MMC5983_SingleRegister_Write(MMC5983_Handler, MMC_INTERNAL_CONTROL_1, MMC5983_Handler->INT_CTRL_1_Reg) != MMC_NO_ERROR)
		return MMC_ERROR;

	return MMC_NO_ERROR;
}


MMC5983_Error_TypeDef MMC5983_Set_Continuous_Measurement(MMC5983_HandleTypeDef *MMC5983_Handler)
{

	MMC5983_Handler->INT_CTRL_2_Reg = MMC5983_Handler->INT_CTRL_2_Reg | MMC_MEASUREMODE;

	if(MMC5983_SingleRegister_Write(MMC5983_Handler, MMC_INTERNAL_CONTROL_2, MMC5983_Handler->INT_CTRL_2_Reg) != MMC_NO_ERROR)
		return MMC_ERROR;
	else
		return MMC_NO_ERROR;
}

MMC5983_Error_TypeDef MMC5983_Set_Output_DataRate(MMC5983_HandleTypeDef *MMC5983_Handler, uint8_t MMC5983_Rate)
{

	MMC5983_Handler->INT_CTRL_2_Reg = MMC5983_Handler->INT_CTRL_2_Reg | MMC5983_Rate;

	if(MMC5983_SingleRegister_Write(MMC5983_Handler, MMC_INTERNAL_CONTROL_2, MMC5983_Handler->INT_CTRL_2_Reg) != MMC_NO_ERROR)
		return MMC_ERROR;
	else
		return MMC_NO_ERROR;
}

MMC5983_Error_TypeDef MMC5983_Enable_Auto_Set_Reset(MMC5983_HandleTypeDef *MMC5983_Handler)
{

	MMC5983_Handler->INT_CTRL_0_Reg = MMC5983_Handler->INT_CTRL_0_Reg | MMC_AUTO_SET_RESET_ENABLE;

	if(MMC5983_SingleRegister_Write(MMC5983_Handler, MMC_INTERNAL_CONTROL_0, MMC5983_Handler->INT_CTRL_0_Reg) != MMC_NO_ERROR)
		return MMC_ERROR;
	else
		return MMC_NO_ERROR;
}

MMC5983_Error_TypeDef MMC5983_Set_Operation_Rate(MMC5983_HandleTypeDef *MMC5983_Handler, uint8_t MMC5983_Set_Rate)
{

	MMC5983_Handler->INT_CTRL_2_Reg = MMC5983_Handler->INT_CTRL_2_Reg | MMC5983_Set_Rate | MMC_SET_ENABLE;

	if(MMC5983_SingleRegister_Write(MMC5983_Handler, MMC_INTERNAL_CONTROL_2, MMC5983_Handler->INT_CTRL_2_Reg) != MMC_NO_ERROR)
		return MMC_ERROR;
	else
		return MMC_NO_ERROR;
}

// MMC5983_Error_TypeDef MMC5983_Init(MMC5983_InitStructureTypeDef *initstructure, MMC5983_HandleTypeDef *handler)
// {
//     MMC5983_SW_Reset(handler);
//     MMC5983_Set_Bandwidth(handler, initstructure->BandWidth);
//     MMC5983_Set_Operation_Rate(handler, initstructure->OperationRate);
//     MMC5983_Enable_Auto_Set_Reset(handler);
//     MMC5983_Set_Output_DataRate(handler, initstructure->OutputRate);
//     MMC5983_Set_Continuous_Measurement(handler);
// }

MMC5983_Error_TypeDef MMC5983_Init(void)
{
    MMC5983_Error_TypeDef status = MMC_NO_ERROR;

    status = MMC5983_SW_Reset(&MMC5983_handle);
    if (status != MMC_NO_ERROR) return status; 

    status = MMC5983_Set_Bandwidth(&MMC5983_handle, MMC_InitStructure.BandWidth);
    if (status != MMC_NO_ERROR) return status;

    status = MMC5983_Set_Operation_Rate(&MMC5983_handle, MMC_InitStructure.OperationRate);
    if (status != MMC_NO_ERROR) return status;

    status = MMC5983_Enable_Auto_Set_Reset(&MMC5983_handle);
    if (status != MMC_NO_ERROR) return status;

    status = MMC5983_Set_Output_DataRate(&MMC5983_handle, MMC_InitStructure.OutputRate);
    if (status != MMC_NO_ERROR) return status;

    status = MMC5983_Set_Continuous_Measurement(&MMC5983_handle);
    if (status != MMC_NO_ERROR) return status;

    return MMC_NO_ERROR;
}

MMC5983_Error_TypeDef MMC5983_Data_Read(MMC5983_Data_TypeDef *MMC5983_Data)
{
	uint8_t SPITransmitData_MMC;
	uint8_t MMC_Data_Reply[7];

	SPITransmitData_MMC = (MMC_Read_Command | MMC_DATA_ADDRESS_INIT);


	enableCS_MMC5983(MMC5983_handle.CS_GPIOport, MMC5983_handle.CS_GPIOpin);
	if(HAL_SPI_Transmit(MMC5983_handle.SPIhandler, &SPITransmitData_MMC, 1, HAL_MAX_DELAY) != HAL_OK)
	{
		disableCS_MMC5983(MMC5983_handle.CS_GPIOport, MMC5983_handle.CS_GPIOpin);
		return MMC_HAL_ERROR;
	}
	if(HAL_SPI_Receive(MMC5983_handle.SPIhandler, MMC_Data_Reply, 6, HAL_MAX_DELAY) != HAL_OK)
	{
		disableCS_MMC5983(MMC5983_handle.CS_GPIOport, MMC5983_handle.CS_GPIOpin);
		return MMC_HAL_ERROR;
	}
	disableCS_MMC5983(MMC5983_handle.CS_GPIOport, MMC5983_handle.CS_GPIOpin);
	
	uint16_t raw_x = (uint16_t)((MMC_Data_Reply[0] << 8) | MMC_Data_Reply[1]);
    uint16_t raw_y = (uint16_t)((MMC_Data_Reply[2] << 8) | MMC_Data_Reply[3]);
    uint16_t raw_z = (uint16_t)((MMC_Data_Reply[4] << 8) | MMC_Data_Reply[5]);

    MMC5983_Data->axes.DX = ((float)raw_x - 32768.0f) / 4096.0f;
    MMC5983_Data->axes.DY = ((float)raw_y - 32768.0f) / 4096.0f;
    MMC5983_Data->axes.DZ = ((float)raw_z - 32768.0f) / 4096.0f;


	return MMC_NO_ERROR;
}

MMC5983_Error_TypeDef MMC5983_ID_Verify(void)
{
	uint8_t ID_Receive=0;

	MMC5983_Error_TypeDef Verify_Status = MMC5983_SingleRegister_Read(&MMC5983_handle, MMC_ID_REG, &ID_Receive);
	if ( (ID_Receive==MMC_DEVICE_ID) && (Verify_Status==MMC_NO_ERROR) )
		return MMC_NO_ERROR;
	else
		return MMC_ID_VERIFY_ERROR;
}
