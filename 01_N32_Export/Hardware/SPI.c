#include "spi.h"

/**
 *  时代的眼泪(^._.^)
 *  SPI1    APB2    72MHzMax
 *  PA5     SPI1_SCK    AF
 *  PA6     SPI1_MISO   FLOAT
 *  PA7     SPI1_MOSI   AF
 *  
 *  SPI3    APB1    36MHzMax
 *  PB3     SPI3_SCK    AF
 *  PB4     SPI3_MISO   FLOAT    
 *  PB5     SPI3_MOSI   AF
 */
void SPI13_Init(void)
{
	RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO, ENABLE);
	GPIO_ConfigPinRemap(GPIO_RMP_SW_JTAG_SW_ENABLE, ENABLE);
	
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA, ENABLE);
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB, ENABLE);
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_SPI1, ENABLE);
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_SPI3, ENABLE);

    //SPI引脚配置
    GPIO_InitType GPIO_InitStructure;
    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.Pin = GPIO_PIN_5 | GPIO_PIN_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = GPIO_PIN_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = GPIO_PIN_3 | GPIO_PIN_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);
    
    GPIO_InitStructure.Pin = GPIO_PIN_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);    

    //SPI1、3配置
    SPI_InitType SPI_InitStructure;
    SPI_InitStruct(&SPI_InitStructure);
    SPI_InitStructure.SpiMode = SPI_MODE_MASTER;
    SPI_InitStructure.BaudRatePres = SPI_BR_PRESCALER_4;
    SPI_InitStructure.DataDirection = SPI_DIR_DOUBLELINE_FULLDUPLEX;
    SPI_InitStructure.NSS = SPI_NSS_SOFT;
    SPI_InitStructure.CLKPHA = SPI_CLKPHA_SECOND_EDGE;
    SPI_InitStructure.CLKPOL = SPI_CLKPOL_HIGH;
    SPI_InitStructure.CRCPoly = 7;
    SPI_InitStructure.DataLen = SPI_DATA_SIZE_8BITS;
    SPI_InitStructure.FirstBit = SPI_FB_MSB;

    SPI_Init(SPI3, &SPI_InitStructure);

    SPI_InitStructure.BaudRatePres = SPI_BR_PRESCALER_8;
	SPI_InitStructure.CLKPHA = SPI_CLKPHA_FIRST_EDGE;
    SPI_InitStructure.CLKPOL = SPI_CLKPOL_LOW;
    SPI_Init(SPI1, &SPI_InitStructure);

    SPI_Enable(SPI1, ENABLE);
    SPI_Enable(SPI3, ENABLE);
}
