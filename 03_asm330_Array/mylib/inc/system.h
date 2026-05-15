#ifndef __SYSTEM_H_
#define __SYSTEM_H_
#include "at32f435_437_gpio.h"
#include "at32f435_437_wk_config.h"
/************************/
/* 位带操作宏函数 */
#define BITBAND(addr, bitnum) (addr&0xF0000000) + 0x2000000 + ((addr - (addr&0xF0000000)) << 5) + (bitnum << 2)
#define MEMADDR(addr) *((volatile unsigned long *)(addr)) 
#define PINADDR(addr, bitnum) MEMADDR(BITBAND(addr, bitnum))
/************************/

/************************/
/*  GPIOx 输入数据寄存器 */
#define GPIOA_IDT_addr (GPIOA_BASE+0x10)
#define GPIOB_IDT_addr (GPIOB_BASE+0x10)
#define GPIOC_IDT_addr (GPIOC_BASE+0x10)
#define GPIOD_IDT_addr (GPIOD_BASE+0x10)
#define GPIOE_IDT_addr (GPIOE_BASE+0x10)
#define GPIOF_IDT_addr (GPIOF_BASE+0x10)
#define GPIOG_IDT_addr (GPIOG_BASE+0x10)
#define GPIOH_IDT_addr (GPIOH_BASE+0x10)
/************************/

/************************/
/*  GPIOx 输出数据寄存器 */
#define GPIOA_ODT_addr (GPIOA_BASE+0x14)
#define GPIOB_ODT_addr (GPIOB_BASE+0x14)
#define GPIOC_ODT_addr (GPIOC_BASE+0x14)
#define GPIOD_ODT_addr (GPIOD_BASE+0x14)
#define GPIOE_ODT_addr (GPIOE_BASE+0x14)
#define GPIOF_ODT_addr (GPIOF_BASE+0x14)
#define GPIOG_ODT_addr (GPIOG_BASE+0x14)
#define GPIOH_ODT_addr (GPIOH_BASE+0x14)
/************************/

/************************/
/* BIT操作宏函数 */
#define P_AOUT(n) PINADDR(GPIOA_ODT_addr, n)
#define P_AIN(n) PINADDR(GPIOA_IDT_addr, n)

#define P_BOUT(n) PINADDR(GPIOB_ODT_addr, n)
#define P_BIN(n) PINADDR(GPIOB_IDT_addr, n)

#define P_COUT(n) PINADDR(GPIOC_ODT_addr, n)
#define P_CIN(n) PINADDR(GPIOC_IDT_addr, n)

#define P_DOUT(n) PINADDR(GPIOD_ODT_addr, n)
#define P_DIN(n) PINADDR(GPIOD_IDT_addr, n)

#define P_EOUT(n) PINADDR(GPIOE_ODT_addr, n)
#define P_EIN(n) PINADDR(GPIOE_IDT_addr, n)

#define P_FOUT(n) PINADDR(GPIOF_ODT_addr, n)
#define P_FIN(n) PINADDR(GPIOF_IDT_addr, n)

#define P_GOUT(n) PINADDR(GPIOG_ODT_addr, n)
#define P_GIN(n) PINADDR(GPIOG_IDT_addr, n)

#define P_HOUT(n) PINADDR(GPIOH_ODT_addr, n)
#define P_HIN(n) PINADDR(GPIOH_IDT_addr, n)
/************************/

/************************/
/* 根据滴答定时器实现AT_Delay() */
extern  volatile uint16_t tickValue;
extern void AT_Delay(uint16_t ms);



/************************/
#endif
