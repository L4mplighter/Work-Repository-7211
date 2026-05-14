#include "gpio.h"

/**
	PA0		CS3_7021
	PA15	CS1_7021
	PA12	CS2_7021
	PA4		CS_359
*/

void CS_GPIO_Init(void)
{
	RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA, ENABLE);
	
	GPIO_InitType GPIO_InitStructure;
	GPIO_InitStruct(&GPIO_InitStructure);
	GPIO_InitStructure.Pin = GPIO_PIN_0 | GPIO_PIN_4 | GPIO_PIN_12 | GPIO_PIN_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOA, GPIO_PIN_0 | GPIO_PIN_4 | GPIO_PIN_12 | GPIO_PIN_15);
}
