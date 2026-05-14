#ifndef __MAIN_H__
#define __MAIN_H__

// Created By LYJ
#include "n32g4fr.h"
#include <stdio.h>
#include <string.h>
#include "USART.h"
#include "GPIO.h"
#include "SPI.h"
#include "TIM.h"
#include "ADXL359.h"
#include "XV7021BB.h"

XV7021_Handle_t xv_handle_1 = {SPI3, GPIOA, GPIO_PIN_15};
XV7021_Handle_t xv_handle_2 = {SPI3, GPIOA, GPIO_PIN_12};
XV7021_Handle_t xv_handle_3 = {SPI3, GPIOA, GPIO_PIN_0 };

#endif
