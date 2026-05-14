#ifndef __MMC5983_H__
#define __MMC5983_H__

#include "spi.h"
#include "stm32h7xx_hal.h"

// --- Registers ---
#define MMC_DATA_ADDRESS_INIT				0x00
#define MMC_DATAX0							0x00
#define MMC_DATAX1							0x01
#define MMC_DATAY0							0x02
#define MMC_DATAY1 							0x03
#define MMC_DATAZ0							0x04
#define MMC_DATAZ1							0x05
#define MMC_DATAXYZ2						0x06
#define MMC_DATAT							0x07

#define MMC_STATUS							0x08
#define MMC_INTERNAL_CONTROL_0				0x09
#define MMC_INTERNAL_CONTROL_1				0x0A
#define MMC_INTERNAL_CONTROL_2				0x0B
#define MMC_INTERNAL_CONTROL_3				0x0C
#define MMC_ID_REG							0x2F

// --- Device ID ---
#define MMC_DEVICE_ID						0x30

// --- Device Output Data Rate ---
#define MMC_ODR_0001		0x01
#define MMC_ODR_0010		0x02
#define MMC_ODR_0020		0x03
#define MMC_ODR_0050		0x04
#define MMC_ODR_0100		0x05
#define MMC_ODR_0200		0x06
#define MMC_ODR_1000		0x07

// --- Device BW Rate ---
#define MMC_BW_0100			0x00
#define MMC_BW_0200			0x01
#define MMC_BW_0400			0x02
#define MMC_BW_0800			0x03

// --- Device SET Operation Rate ---
#define MMC_SET_0001		0x00
#define MMC_SET_0025		0x10
#define MMC_SET_0075		0x20
#define MMC_SET_0100		0x30
#define MMC_SET_0250		0x40
#define MMC_SET_0500		0x50
#define MMC_SET_1000		0x60
#define MMC_SET_2000		0x70

// --- Operation Modes ---
#define MMC_MEASUREMODE 			0x08
#define MMC_SET_ENABLE				0x80
#define MMC_AUTO_SET_RESET_ENABLE	0x20

// --- Interrupt Map ---
#define MMC_INT_DATA_READY			0x04

// --- SPI Commands ---
#define MMC_Read_Command			0x80
#define MMC_Read_Multiple_Command	0xC0
#define MMC_Write_command			0x00

// --- Reset Command ---
#define MMC5983_SW_Reset_Mask		0x80

// --- SPI Timeout (ms) ---
#define MMC_Timeout	10

// --- Error Status Enum ---
typedef enum {
	MMC_NO_ERROR 			=	0x00U, 	/**< No error */
	MMC_ERROR	  			=	0x01U, 	/**< Generic error */
	MMC_HAL_ERROR			= 	0x02U, 	/**< HAL communication error */
	MMC_INV_REG_DATA_READ	= 	0x03U, 	/**< Invalid data read from register */
	MMC_ID_VERIFY_ERROR		=	0x04U, 	/**< Device ID verification failed */
    
} MMC5983_Error_TypeDef;

typedef struct {
	SPI_HandleTypeDef *SPIhandler; 	/**< Pointer to SPI handler */
	GPIO_TypeDef *CS_GPIOport;     	/**< GPIO port for chip select */
	uint16_t CS_GPIOpin;			/**< GPIO pin number for chip select */
	uint8_t INT_CTRL_0_Reg;         /**< Interrupt control register 0 content */
	uint8_t INT_CTRL_1_Reg;         /**< Interrupt control register 1 content */
	uint8_t INT_CTRL_2_Reg;         /**< Interrupt control register 2 content */
	uint8_t INT_CTRL_3_Reg;         /**< Interrupt control register 3 content */
} MMC5983_HandleTypeDef;

typedef struct{
    uint8_t OperationRate;
    uint8_t OperationMode;
    uint8_t BandWidth;
    uint8_t OutputRate;
} MMC5983_InitStructureTypeDef;

typedef union {
    struct {
        float DX;
        float DY;
        float DZ;
    } axes;
    uint8_t raw_data[6];
}MMC5983_Data_TypeDef;

MMC5983_Error_TypeDef MMC5983_SW_Reset(MMC5983_HandleTypeDef *);

MMC5983_Error_TypeDef MMC5983_Set_Bandwidth(MMC5983_HandleTypeDef *, uint8_t);

MMC5983_Error_TypeDef MMC5983_Set_Operation_Rate(MMC5983_HandleTypeDef *, uint8_t);

MMC5983_Error_TypeDef MMC5983_Enable_Auto_Set_Reset(MMC5983_HandleTypeDef *);

MMC5983_Error_TypeDef MMC5983_Set_Output_DataRate(MMC5983_HandleTypeDef *, uint8_t);

MMC5983_Error_TypeDef MMC5983_Set_Continuous_Measurement(MMC5983_HandleTypeDef *);

MMC5983_Error_TypeDef MMC5983_SingleRegister_Read(MMC5983_HandleTypeDef *, uint8_t, uint8_t *);

MMC5983_Error_TypeDef MMC5983_SingleRegister_Write(MMC5983_HandleTypeDef *, uint8_t, uint8_t);

void enableCS_MMC5983(GPIO_TypeDef *, uint16_t);

void disableCS_MMC5983(GPIO_TypeDef *, uint16_t);

// MMC5983_Error_TypeDef MMC5983_Init(MMC5983_InitStructureTypeDef *initstructure, MMC5983_HandleTypeDef *handler);
MMC5983_Error_TypeDef MMC5983_Init(void);
MMC5983_Error_TypeDef MMC5983_Data_Read(MMC5983_Data_TypeDef *MMC5983_Data);
MMC5983_Error_TypeDef MMC5983_ID_Verify(void);

#endif
