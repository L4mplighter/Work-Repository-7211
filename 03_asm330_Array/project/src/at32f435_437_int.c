/* add user code begin Header */
/**
  **************************************************************************
  * @file     at32f435_437_int.c
  * @brief    main interrupt service routines.
  **************************************************************************
  *                       Copyright notice & Disclaimer
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */
/* add user code end Header */

/* includes ------------------------------------------------------------------*/
#include "at32f435_437_int.h"
#include "wk_system.h"
#include "jump2app.h"
#include "string.h"
#include "option.h"
#include "math.h"
/* private includes ----------------------------------------------------------*/
/* add user code begin private includes */
uint8_t timer5ms = 0;
uint8_t timer10ms = 0;
uint16_t timerCnt = 0;

char settingBuf[20] = {0x00};
char commandBuf_[13] = {0x00};
uint8_t commandSize_ = 0;
uint8_t settingFlag = 0;
uint8_t settingCnt = 0;
const uint8_t outputRate50hz = 4;
const uint8_t outputRate100hz = 2;
const uint8_t outputRate200hz = 1;
uint8_t settingRate = 0;
/* add user code end private includes */

/* private typedef -----------------------------------------------------------*/
/* add user code begin private typedef */

/* add user code end private typedef */

/* private define ------------------------------------------------------------*/
/* add user code begin private define */

/* add user code end private define */

/* private macro -------------------------------------------------------------*/
/* add user code begin private macro */

/* add user code end private macro */

/* private variables ---------------------------------------------------------*/
/* add user code begin private variables */

/* add user code end private variables */

/* private function prototypes --------------------------------------------*/
/* add user code begin function prototypes */

/* add user code end function prototypes */

/* private user code ---------------------------------------------------------*/
/* add user code begin 0 */

/* add user code end 0 */

/* external variables ---------------------------------------------------------*/
/* add user code begin external variables */

/* add user code end external variables */

/**
  * @brief  this function handles nmi exception.
  * @param  none
  * @retval none
  */
void NMI_Handler(void)
{
  /* add user code begin NonMaskableInt_IRQ 0 */

  /* add user code end NonMaskableInt_IRQ 0 */

  /* add user code begin NonMaskableInt_IRQ 1 */

  /* add user code end NonMaskableInt_IRQ 1 */
}

/**
  * @brief  this function handles hard fault exception.
  * @param  none
  * @retval none
  */
void HardFault_Handler(void)
{
  /* add user code begin HardFault_IRQ 0 */

  /* add user code end HardFault_IRQ 0 */
  /* go to infinite loop when hard fault exception occurs */
  while (1)
  {
    /* add user code begin W1_HardFault_IRQ 0 */

    /* add user code end W1_HardFault_IRQ 0 */
  }
}

/**
  * @brief  this function handles memory manage exception.
  * @param  none
  * @retval none
  */
void MemManage_Handler(void)
{
  /* add user code begin MemoryManagement_IRQ 0 */

  /* add user code end MemoryManagement_IRQ 0 */
  /* go to infinite loop when memory manage exception occurs */
  while (1)
  {
    /* add user code begin W1_MemoryManagement_IRQ 0 */

    /* add user code end W1_MemoryManagement_IRQ 0 */
  }
}

/**
  * @brief  this function handles bus fault exception.
  * @param  none
  * @retval none
  */
void BusFault_Handler(void)
{
  /* add user code begin BusFault_IRQ 0 */

  /* add user code end BusFault_IRQ 0 */
  /* go to infinite loop when bus fault exception occurs */
  while (1)
  {
    /* add user code begin W1_BusFault_IRQ 0 */

    /* add user code end W1_BusFault_IRQ 0 */
  }
}

/**
  * @brief  this function handles usage fault exception.
  * @param  none
  * @retval none
  */
void UsageFault_Handler(void)
{
  /* add user code begin UsageFault_IRQ 0 */

  /* add user code end UsageFault_IRQ 0 */
  /* go to infinite loop when usage fault exception occurs */
  while (1)
  {
    /* add user code begin W1_UsageFault_IRQ 0 */

    /* add user code end W1_UsageFault_IRQ 0 */
  }
}

/**
  * @brief  this function handles svcall exception.
  * @param  none
  * @retval none
  */
void SVC_Handler(void)
{
  /* add user code begin SVCall_IRQ 0 */

  /* add user code end SVCall_IRQ 0 */
  /* add user code begin SVCall_IRQ 1 */

  /* add user code end SVCall_IRQ 1 */
}

/**
  * @brief  this function handles debug monitor exception.
  * @param  none
  * @retval none
  */
void DebugMon_Handler(void)
{
  /* add user code begin DebugMonitor_IRQ 0 */

  /* add user code end DebugMonitor_IRQ 0 */
  /* add user code begin DebugMonitor_IRQ 1 */

  /* add user code end DebugMonitor_IRQ 1 */
}

/**
  * @brief  this function handles pendsv_handler exception.
  * @param  none
  * @retval none
  */
void PendSV_Handler(void)
{
  /* add user code begin PendSV_IRQ 0 */

  /* add user code end PendSV_IRQ 0 */
  /* add user code begin PendSV_IRQ 1 */

  /* add user code end PendSV_IRQ 1 */
}

/**
  * @brief  this function handles systick handler.
  * @param  none
  * @retval none
  */
void SysTick_Handler(void)
{
  /* add user code begin SysTick_IRQ 0 */
   
  /* add user code end SysTick_IRQ 0 */

  wk_timebase_handler();
  timerCnt=(unsigned short)((timerCnt + 1) % 1000); 
  /* add user code begin SysTick_IRQ 1 */
  
    if(timerCnt %5== 0) 
	 {
        //timerCnt = 0;
        timer5ms = 1;
    }
	 if(timerCnt %10== 0)
	 {
	  timer10ms = 1;
	 }
  /* add user code end SysTick_IRQ 1 */
}

/* add user code begin 1 */
  uint8_t buff_uart1[64]={0};
  uint8_t rcmd_flag=0;
  static uint8_t buff_head[2]={0};
  static uint8_t buff_count=0;
  static uint8_t srcount=0;
  static uint8_t cmd_flag=0;
  static uint16_t check=0;
  static uint32_t rbegintime=0;
  
/* add user code end 1 */
float utc_time=0;
void USART1_IRQHandler(void)
{
  /* add user code begin USART1_IRQ 0 */
    if (SET == usart_interrupt_flag_get(USART1, USART_RDBF_FLAG)) 
	 {
        usart_flag_clear(USART1, USART_RDBF_FLAG);
        unsigned char buf = (unsigned char)(USART1->dt_bit.dt);
        jumpToApp(buf);//bootloader
		  
        if(settingFlag == 1 || (char)buf == '$') //200g…Ë÷√√¸¡Ó
		  {
            settingFlag = 1;
            settingBuf[settingCnt++] = (char)buf;
            if(settingBuf[settingCnt - 1] == 0x0A) 
				{
                commandSize_ = settingCnt - 2;
                memcpy(commandBuf_, settingBuf, commandSize_);
                settingFlag = 0;
                settingCnt = 0;
                memset(settingBuf, 0x00, sizeof(settingBuf));
                command_decode(commandBuf_, commandSize_);
            }
        }
		  
//		  //eb 90  begin
//
		  if(cmd_flag==1)
		  {
		    if(buff_count==2)
			 {
			    srcount=buf+3;
				 buff_uart1[2]=buf;
				 buff_count++;
				 check=0;
			 }
			 else if(buff_count<srcount)
			 {
			    buff_uart1[buff_count]=buf;
				 buff_count++;
				 check=check+buf;
			 }
			 else if(buff_count==srcount)
			 {
			     buff_uart1[buff_count]=buf;
				  cmd_flag=0;
				  buff_count=0;
				  if((uint8_t)(check&0xff)==buf)
				  {
				    rcmd_flag=1;
				  }
			 }
		  }
		  
		  buff_head[0]=buff_head[1];
		  buff_head[1]=buf;
		  if(buff_head[0]==0xeb&&buff_head[1]==0x90)
		  {
		    if(cmd_flag==0)
			 {
		    cmd_flag=1;
			 buff_count=2;
			 rbegintime=wk_timebase_get();
			 }
			 else
			 {
			   if(wk_timebase_get()-rbegintime>2)
				{
				  rbegintime=wk_timebase_get();
				  buff_count=2;
				}
			 }
		  }
		 //eb 90 end   
    }
  /* add user code end USART1_IRQ 0 */
  /* add user code begin USART1_IRQ 1 */

  /* add user code end USART1_IRQ 1 */
}
float pre_pps=0.0; //ms
static uint8_t pps_time=0;
uint32_t ir_rise_pre=0;
uint32_t ir_rise_now=0;
void EXINT0_IRQHandler(void)
{
  /* add user code begin EXINT0_IRQ 0 */
     if(exint_interrupt_flag_get(EXINT_LINE_0))
	  {
	     ir_rise_now=wk_timebase_get();
	  }
//   if(exint_interrupt_flag_get(EXINT_LINE_0))
//	{
//	   if(pps_time<2)
//		{
//		   pps_time++;
//			return;
//		}
//		else if(pps_time==2)
//		{
//		   pps_time++;
//			pre_pps=get_systick_time_ms();
//			return;
//		}
//		float a=get_systick_time_ms();
//		pre_pps=a;
//		float b=a-pre_pps;
//		float d=roundf(b);
//		float c=b-d;
//		if(b>0.9&&b<1.1)
//		{
//	      pre_pps=a;
//			utc_time=utc_time+1.0;
//			return ;
//		}
//	   if(b>1.05&&fabs(c)<0.1)
//		{
//	      pre_pps=a;
//			utc_time=utc_time+d;
//			return ;
//		}
//	}
  /* add user code end EXINT0_IRQ 0 */
  /* add user code begin EXINT0_IRQ 1 */

  /* add user code end EXINT0_IRQ 1 */
}
