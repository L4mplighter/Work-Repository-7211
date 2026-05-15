#ifndef __MYFLASH_H_
#define __MYFLASH_H_

#ifdef STM32F4XX
#include "stm32f4xx_hal.h"
#else
#include "at32f435_437.h"
#endif

/*flash information macro*/
#define     flash_address       (uint32_t)0x08000000
#define     flash_size          (uint32_t)0x00030000 //定义写入的flash大小为192KB

/*sector map*/
/*bank 1*/
/*area 0*/
#define     sector0_address     (uint32_t)0x08000000
#define     sector1_address     (uint32_t)0x08001000
#define     sector2_address     (uint32_t)0x08002000
#define     sector3_address     (uint32_t)0x08003000
/*area 1*/
#define     sector4_address     (uint32_t)0x08004000
#define     sector5_address     (uint32_t)0x08005000
#define     sector6_address     (uint32_t)0x08006000
#define     sector7_address     (uint32_t)0x08007000
/*area 2*/
#define     sector8_address     (uint32_t)0x08008000
#define     sector9_address     (uint32_t)0x08009000
#define     sector10_address    (uint32_t)0x0800A000
#define     sector11_address   (uint32_t)0x0800B000
/*area 3*/
#define     sector12_address   (uint32_t)0x0800C000
#define     sector13_address   (uint32_t)0x0800D000
#define     sector14_address   (uint32_t)0x0800E000
#define     sector15_address   (uint32_t)0x0800F000
/*bank 2*/
/*area 4*/
#define     sector16_address   (uint32_t)0x08010000
#define     sector17_address   (uint32_t)0x08011000
#define     sector18_address   (uint32_t)0x08012000
#define     sector19_address   (uint32_t)0x08013000
/*area 5*/
#define     sector20_address   (uint32_t)0x08014000
#define     sector21_address   (uint32_t)0x08015000
#define     sector22_address   (uint32_t)0x08016000
#define     sector23_address   (uint32_t)0x08017000
/*area 6*/
#define     sector24_address   (uint32_t)0x08018000
#define     sector25_address   (uint32_t)0x08019000
#define     sector26_address   (uint32_t)0x0801A000
#define     sector27_address   (uint32_t)0x0801B000
/*area 7*/
#define     sector28_address   (uint32_t)0x0801C000
#define     sector29_address   (uint32_t)0x0801D000
#define     sector30_address   (uint32_t)0x0801E000
#define     sector31_address   (uint32_t)0x0801F000
#define     sector32_address   (uint32_t)0x08020000
#define     sector33_address   (uint32_t)0x08021000
#define     sector34_address   (uint32_t)0x08022000
#define     sector35_address   (uint32_t)0x08023000
#define     sector36_address   (uint32_t)0x08024000
#define     sector37_address   (uint32_t)0x08025000
#define     sector38_address   (uint32_t)0x08026000
#define     sector39_address   (uint32_t)0x08027000
#define     sector40_address   (uint32_t)0x08028000
#define     sector41_address   (uint32_t)0x08029000
#define     sector42_address   (uint32_t)0x0802A000
#define     sector43_address   (uint32_t)0x0802B000
#define     sector44_address   (uint32_t)0x0802C000
#define     sector45_address   (uint32_t)0x0802D000
#define     sector46_address   (uint32_t)0x0802E000
#define     sector47_address   (uint32_t)0x0802F000
#define     sector48_address   (uint32_t)0x08030000
#define     sector49_address   (uint32_t)0x08031000


/*application information macro*/
#define     application_address (uint32_t)sector9_address
#define     user_size          (uint32_t)(sector48_address - application_address)

/* procedure flag */
#define		flag_address		sector48_address
#define		pro_flag			(uint32_t)0x72116666

/* option data */
#define     option_address              sector49_address
#define     option_flag                 (uint32_t)0x72117211
#define     Protocol_address            (sector49_address + 4)
#define     BdEnableFlag_address        (sector49_address + 5)
#define     BdEnableFlag2_address       (sector49_address + 6)
#define     UartEnableFlag_address      (sector49_address + 7)
#define     SampleFlag_address          (sector49_address + 8)
#define     OutputRatio_address         (sector49_address + 9)
#define     ComRate_address             (sector49_address + 10)
#define     Drift_address               (sector49_address + 14)
#define     Bias_address                (sector49_address + 26)
#define     RotationAngle_address       (sector49_address + 38)
#define     BandWidth_address           (sector49_address + 50)
#define     GyrpRange_address           (sector49_address + 51)

typedef  void (*iapfun)(void);              //����һ���������͵Ĳ���.
/*function defined */
void flash_init(void);
static uint32_t sector_location(const uint32_t address);
flash_status_type flash_program(uint32_t address,uint32_t const * data,const uint32_t len);
flash_status_type flash_erase(uint32_t address,uint32_t size);
void iap_load_app(uint32_t appxaddr);
#endif
