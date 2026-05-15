/* add user code begin Header */
/**
  **************************************************************************
  * @file     main.c
  * @brief    main program
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

/* Includes ------------------------------------------------------------------*/
#include "at32f435_437_wk_config.h"
#include "wk_system.h"

/* private includes ----------------------------------------------------------*/
/* add user code begin private includes */
#include "ATSpi.h"
#include "asm330lhhx.h"
#include "string.h"
#include "ATUsart.h"
#include "at32f435_437_int.h"
#include "errorcor.h"
#include "adxl375.h"
#include "option.h"
#include "tempCalibration.h"
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
extern  uint8_t timer10ms;
extern  uint8_t buff_uart1[64];
extern  uint8_t rcmd_flag;
extern uint32_t ir_rise_now;
uint16_t imu_simple_time=0;
ASM330DATAOP asm330_data1_;
ASM330DATAOP asm330_data2_;
ASM330DATAOP asm330_data3_;
ASM330DATAOP asm330_data4_;
float accx_[SENSOR_NUM] = {0.0};
float accy_[SENSOR_NUM] = {0.0};
float accz_[SENSOR_NUM] = {0.0};
float gyrox_[SENSOR_NUM] = {0.0};
float gyroy_[SENSOR_NUM] = {0.0};
float gyroz_[SENSOR_NUM] = {0.0};
uint8_t hz_5_cnt = 0;

float accx_5[25] = {0.0};
float accy_5[25] = {0.0};
float accz_5[25] = {0.0};
float gyrox_5[25] = {0.0};
float gyroy_5[25] = {0.0};
float gyroz_5[25] = {0.0};

float accx_50_[5] = {0.0};
float accy_50_[5] = {0.0};
float accz_50_[5] = {0.0};
float gyrox_50_[5] = {0.0};
float gyroy_50_[5] = {0.0};
float gyroz_50_[5] = {0.0};
//gyro就不比例因子融合了。
float mean_cofe[24]={
0.25,	0.25,	0.25,	0.25,
0.25,	0.25,	0.25,	0.25,
0.25,	0.25,	0.25,	0.25,
0.0245290310001870,0.00783754960468403,0.722448599607240,0.245184819787889,   // 0.337495811970225	0.192064596826833	0.271174938491673	0.199264652711269 以前的y轴
0.337495811970225,	0.192064596826833,0.271174938491673,0.199264652711269,   //  0.0245290310001870	0.00783754960468403	0.722448599607240	0.245184819787889 以前的x轴
0.137098691420348, 0.151546349342972,	 0.0272218592653002,	0.684133099971379};

uint8_t timeCnt = 0;
ASM330DATAOP asm330_data_mean_, asm330_data_mean_5;

float mean(float* data, int meanNum) {
    float buf = 0.0f;
    for (int i = 0; i < meanNum; i++)
        buf += data[i];
    return buf / (meanNum * 1.0f);
}
float mul1x4(float* a, float *b) {

    return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]+a[3]*b[3];
}
uint8_t ch_buf[256] = {0x00};
//#define test
void dataSend(void)	{
    #ifndef test
    //200G格式
    
    uint16_t check_sum = 0;
    float f_acc_[3] = {0.0};
////    f_acc_[0] = asm330_data_mean_.acc[0] / 9.8f;
////    f_acc_[1] = asm330_data_mean_.acc[1] / 9.8f;
////    f_acc_[2] = asm330_data_mean_.acc[2] / 9.8f;
//    f_acc_[0] = asm330_data_mean_.acc[0];
//    f_acc_[1] = asm330_data_mean_.acc[1];
//    f_acc_[2] = asm330_data_mean_.acc[2];
    ch_buf[0] = 0xAA;
    ch_buf[1] = 0x55;
    memcpy(&ch_buf[2], asm330_data_mean_.gyro,4);			    //gyro_[0]
    memcpy(&ch_buf[6], &asm330_data_mean_.gyro[1],4);	        //gyro_[1]
    memcpy(&ch_buf[10],&asm330_data_mean_.gyro[2],4);         //gyro_[2]
    memcpy(&ch_buf[14],asm330_data_mean_.acc,4);			    //acc_[0]
    memcpy(&ch_buf[18],&asm330_data_mean_.acc[1],4);         //acc_[1]
    memcpy(&ch_buf[22],&asm330_data_mean_.acc[2],4);         //acc_[2]
    memcpy(&ch_buf[38],&asm330_data1_.temp,4);         //acc_[2]
    for (int i=2; i<= 41; i++)
    {
     check_sum = check_sum + ch_buf[i];
    }
    ch_buf[42] = (uint8_t) (check_sum & 0x00FF);
    //at_usart_transmit(USART1, ch_buf, 43);
   dma_channel_enable(DMA1_CHANNEL1, FALSE);
   wk_dma_channel_config(DMA1_CHANNEL1, 
                        (uint32_t)&USART1->dt, 
                        (uint32_t)ch_buf, 
                        43);
  dma_channel_enable(DMA1_CHANNEL1, TRUE);
    #else

	int t = 0;
	static int num_send = 0;
	num_send++;

	ch_buf[0] = 0xEB;
	ch_buf[1] = 0x90;

	ch_buf[2] = 0x08;
	ch_buf[3] = 0x65;//226个字节长度
	
	float aa =num_send*1.0;
	
//	float gx,gy,gz,ax,ay,az;
//	gx = (imuCal.gyroloter[0].gyrox + imuCal.gyroloter[1].gyrox +imuCal.gyroloter[2].gyrox + imuCal.gyroloter[3].gyrox) / 4.0;
//	gy = (imuCal.gyroloter[0].gyroy + imuCal.gyroloter[1].gyroy +imuCal.gyroloter[2].gyroy + imuCal.gyroloter[3].gyroy) / 4.0;
//	gz = (imuCal.gyroloter[0].gyroz + imuCal.gyroloter[1].gyroz +imuCal.gyroloter[2].gyroz + imuCal.gyroloter[3].gyroz) / 4.0;
//	ax = (imuCal.acceloter[0].accx + imuCal.acceloter[1].accx +imuCal.acceloter[2].accx + imuCal.acceloter[3].accx) / 4.0;
//	ay = (imuCal.acceloter[0].accy + imuCal.acceloter[1].accy +imuCal.acceloter[2].accy + imuCal.acceloter[3].accy) / 4.0;
//	az = (imuCal.acceloter[0].accz + imuCal.acceloter[1].accz +imuCal.acceloter[2].accz + imuCal.acceloter[3].accz) / 4.0;

	memcpy(&ch_buf[4], &asm330_data1_.gyro[0],4);				//陀螺	
	memcpy(&ch_buf[8], &asm330_data1_.gyro[1],4);
	memcpy(&ch_buf[12],&asm330_data1_.gyro[2],4);
	memcpy(&ch_buf[16],&asm330_data1_.acc[0] ,4);			
	memcpy(&ch_buf[20],&asm330_data1_.acc[1] ,4);
	memcpy(&ch_buf[24],&asm330_data1_.acc[2] ,4);
	
	memcpy(&ch_buf[28],&asm330_data2_.gyro[0],4);					
	memcpy(&ch_buf[32],&asm330_data2_.gyro[1],4);
	memcpy(&ch_buf[36],&asm330_data2_.gyro[2],4);
	memcpy(&ch_buf[40],&asm330_data2_.acc[0] ,4);					
	memcpy(&ch_buf[44],&asm330_data2_.acc[1] ,4);
	memcpy(&ch_buf[48],&asm330_data2_.acc[2] ,4);
												 
	memcpy(&ch_buf[52],&asm330_data3_.gyro[0],4);					
	memcpy(&ch_buf[56],&asm330_data3_.gyro[1],4);
	memcpy(&ch_buf[60],&asm330_data3_.gyro[2],4);
	memcpy(&ch_buf[64],&asm330_data3_.acc[0] ,4);					
	memcpy(&ch_buf[68],&asm330_data3_.acc[1] ,4);
	memcpy(&ch_buf[72],&asm330_data3_.acc[2] ,4);
	
	memcpy(&ch_buf[76],&asm330_data4_.gyro[0],4);					
	memcpy(&ch_buf[80],&asm330_data4_.gyro[1],4);
	memcpy(&ch_buf[84],&asm330_data4_.gyro[2],4);
	memcpy(&ch_buf[88],&asm330_data4_.acc[0] ,4);					
	memcpy(&ch_buf[92],&asm330_data4_.acc[1] ,4);
	memcpy(&ch_buf[96],&asm330_data4_.acc[2] ,4);


	memcpy(&ch_buf[100], &asm330_data_mean_.gyro[0],4);				//陀螺	
	memcpy(&ch_buf[104], &asm330_data_mean_.gyro[1],4);
	memcpy(&ch_buf[108],&asm330_data_mean_.gyro[2],4);
	memcpy(&ch_buf[112],&asm330_data_mean_.acc[0],4);			
	memcpy(&ch_buf[116],&asm330_data_mean_.acc[1],4);
	memcpy(&ch_buf[120],&asm330_data_mean_.acc[2],4);
//	
//	memcpy(&ch_buf[28],&imu.gyroloter[1].gyrox,4);					
//	memcpy(&ch_buf[32],&imu.gyroloter[1].gyroy,4);
//	memcpy(&ch_buf[36],&imu.gyroloter[1].gyroz,4);
//	memcpy(&ch_buf[40],&imu.acceloter[1].accx,4);					
//	memcpy(&ch_buf[44],&imu.acceloter[1].accy,4);
//	memcpy(&ch_buf[48],&imu.acceloter[1].accz,4);
//												 
//	memcpy(&ch_buf[52],&imu.gyroloter[2].gyrox,4);					
//	memcpy(&ch_buf[56],&imu.gyroloter[2].gyroy,4);
//	memcpy(&ch_buf[60],&imu.gyroloter[2].gyroz,4);
//	memcpy(&ch_buf[64],&imu.acceloter[2].accx,4);					
//	memcpy(&ch_buf[68],&imu.acceloter[2].accy,4);
//	memcpy(&ch_buf[72],&imu.acceloter[2].accz,4);
//	
//	memcpy(&ch_buf[76],&imu.gyroloter[3].gyrox,4);					
//	memcpy(&ch_buf[80],&imu.gyroloter[3].gyroy,4);
//	memcpy(&ch_buf[84],&imu.gyroloter[3].gyroz,4);
//	memcpy(&ch_buf[88],&imu.acceloter[3].accx,4);					
//	memcpy(&ch_buf[92],&imu.acceloter[3].accy,4);
//	memcpy(&ch_buf[96],&imu.acceloter[3].accz,4);
											
//	memcpy(&ch_buf[100],&gyro_[0],4);					
//	memcpy(&ch_buf[104],&gyro_[1],4);
//	memcpy(&ch_buf[108],&gyro_[2],4);
//	memcpy(&ch_buf[112],&acc_[0],4);					
//	memcpy(&ch_buf[116],&acc_[1],4);
//	memcpy(&ch_buf[120],&acc_[2],4);
//	

//												 
//	memcpy(&ch_buf[124],&adxl375_regs_data.gyro[0],4);							
//	memcpy(&ch_buf[128],&adxl375_regs_data.gyro[1],4);
//	memcpy(&ch_buf[132],&adxl375_regs_data.gyro[2],4);
	memcpy(&ch_buf[136],&adxl375_regs_data.accel[0],4);							
	memcpy(&ch_buf[140],&adxl375_regs_data.accel[1],4);
	memcpy(&ch_buf[144],&adxl375_regs_data.accel[2],4);
//												 
//	memcpy(&ch_buf[148],&imu.gyroloter[6].gyrox,4);					
//	memcpy(&ch_buf[152],&imu.gyroloter[6].gyroy,4);
//	memcpy(&ch_buf[156],&imu.gyroloter[6].gyroz,4);
//	memcpy(&ch_buf[160],&imu.acceloter[6].accx,4);						
//	memcpy(&ch_buf[164],&imu.acceloter[6].accy,4);
//	memcpy(&ch_buf[168],&imu.acceloter[6].accz,4);
//	
//	memcpy(&ch_buf[172],&imu.gyroloter[7].gyrox,4);					
//	memcpy(&ch_buf[176],&imu.gyroloter[7].gyroy,4);
//	memcpy(&ch_buf[180],&imu.gyroloter[7].gyroz,4);
//	memcpy(&ch_buf[184],&imu.acceloter[7].accx,4);						
//	memcpy(&ch_buf[188],&imu.acceloter[7].accy,4);
//	memcpy(&ch_buf[192],&imu.acceloter[7].accz,4);
//	
//	memcpy(&ch_buf[196],&imu.gyroloter[8].gyrox,4);					
//	memcpy(&ch_buf[200],&imu.gyroloter[8].gyroy,4);
//	memcpy(&ch_buf[204],&imu.gyroloter[8].gyroz,4);
//	memcpy(&ch_buf[208],&imu.acceloter[8].accx,4);						
//	memcpy(&ch_buf[212],&imu.acceloter[8].accy,4);
	memcpy(&ch_buf[216],&aa,4);
	
	memcpy(&ch_buf[220],&asm330_data1_.temp,4);
	
	uint16_t check_sum;
	check_sum=0;
	for (int i=2; i < 224; i++)
	{
	 check_sum = check_sum + ch_buf[i];
	}
	ch_buf[224] = check_sum & 0x00FF;
	ch_buf[225] = (check_sum>>8) & 0x00FF;
   dma_channel_enable(DMA1_CHANNEL1, FALSE);
   wk_dma_channel_config(DMA1_CHANNEL1, 
                        (uint32_t)&USART1->dt, 
                        (uint32_t)ch_buf, 
                        226);
  dma_channel_enable(DMA1_CHANNEL1, TRUE);
    //at_usart_transmit(USART1, ch_buf, 226);
    #endif
}
union inttochar
{
   uint32_t i;
	char c[4];
};

void dataSend_22(void)
{
  ch_buf[0] = 0xAA;
  ch_buf[1] = 0x55;
  static uint8_t fromframe=0;
  uint8_t nn=2;
  union inttochar b;
  b.i=0;
  if(fromframe==0)
  {
    nn=2;
	 
  }else
  {
    nn=22;
  }
  for(int n=0;n<=2;n++)
  {
    b.i=(uint32_t)(fabs(asm330_data_mean_.gyro[n])*8388608/1000.0);
	 if(b.i>8388608)
	           b.i=8388608;
	 if(asm330_data_mean_.gyro[n]<0)
	           b.i=0xffffff-b.i;
	 memcpy(&ch_buf[nn+n*3],b.c,3);
	 b.i=(uint32_t)(fabs(asm330_data_mean_.acc[n])*8388608/16.0);
	 if(b.i>8388608)
	           b.i=8388608;
	 if(asm330_data_mean_.acc[n]<0)
	           b.i=0xffffff-b.i;
	 memcpy(&ch_buf[nn+(n+3)*3],b.c,3);
  }
  if(fromframe==0)
  {
  memcpy(&ch_buf[20],&imu_simple_time,2);
  fromframe=1;
  }
  else
  {
   uint16_t sum=0;
   for(int n=2;n<=39;n++)
	{
     sum=sum+ch_buf[n];
	}
	memcpy(&ch_buf[40],&sum,2);
  dma_channel_enable(DMA1_CHANNEL1, FALSE);
  wk_dma_channel_config(DMA1_CHANNEL1, 
                        (uint32_t)&USART1->dt, 
                        (uint32_t)ch_buf, 
                        42);
  dma_channel_enable(DMA1_CHANNEL1, TRUE);
  fromframe=0;
  }


}
optionINFO WG200G;
uint8_t hz200_cnt = 0;
uint8_t hz100_cnt = 0;
uint8_t hz50_cnt = 0;
uint32_t delat_tick=0;
uint32_t delat_time=0;
/* add user code end 0 */

/**
  * @brief main function.
  * @param  none
  * @retval none
  */
int main(void)
{
  /* add user code begin 1 */
    //SCB->VTOR = FLASH_BASE | 0x9000;
	//__enable_irq();
  option_init();
  /* add user code end 1 */

  /* system clock config. */
  wk_system_clock_config();

  /* config periph clock. */
  wk_periph_clock_config();

  /* nvic config. */
  wk_nvic_config();

  /* timebase config. */
  wk_timebase_init();

  /* init dma1 channel1 */
  wk_dma1_channel1_init();
  /* config dma channel transfer parameter */
  /* user need to modify define values DMAx_CHANNELy_XXX_BASE_ADDR and DMAx_CHANNELy_BUFFER_SIZE in at32xxx_wk_config.h */
  wk_dma_channel_config(DMA1_CHANNEL1, 
                        (uint32_t)&USART1->dt, 
                        DMA1_CHANNEL1_MEMORY_BASE_ADDR, 
                        DMA1_CHANNEL1_BUFFER_SIZE);
  dma_channel_enable(DMA1_CHANNEL1, TRUE);
  
  wk_dma_channel_config(DMA1_CHANNEL2, 
                        (uint32_t)&USART6->dt, 
                        DMA1_CHANNEL2_MEMORY_BASE_ADDR, 
                        DMA1_CHANNEL2_BUFFER_SIZE);
  dma_channel_enable(DMA1_CHANNEL2, TRUE);

  /* init usart1 function. */
  wk_usart1_init();

  /* init spi1 function. */
  wk_spi1_init();

  /* init gpio function. */
  wk_gpio_config();

  /* add user code begin 2 */
  wk_exint_config();
  exint_interrupt_enable(EXINT_LINE_0,TRUE);
  asm330lhhx_init();
  ADXL375_Master_Init();
  /* add user code end 2 */

  while(1)
  {
    /* add user code begin 3 */

//	 if(buff_uart1[3]==0x01&&rcmd_flag==1)
//	 {
//	     rcmd_flag=0;
//		  dataSend();
//	 }
	 if (timer5ms == 1) 
	 {
    #ifndef test
    float TBias[6] = {0.0f};
	 timer5ms = 0;
	 //delat_tick=wk_timebase_get()-delat_time;
	 delat_time=(wk_timebase_get()-ir_rise_now); 

    asm330lhhx_data(ASM330PORT, &asm330_data1_, ASM330_SENSOR_1);
    asm330lhhx_data(ASM330PORT, &asm330_data2_, ASM330_SENSOR_2);
    asm330lhhx_data(ASM330PORT, &asm330_data3_, ASM330_SENSOR_3);
    asm330lhhx_data(ASM330PORT, &asm330_data4_, ASM330_SENSOR_4);
    //ADXL375_Master_Read_Regs();
    errorcor();
//    tempCalibration(asm330_data1_.temp, TBias); 
    if(timer10ms!=1)
	 {
	 imu_simple_time=delat_time%1000;
    accx_[0] = asm330_data1_.acc[0];    accx_[1] = asm330_data2_.acc[0];   accx_[2] = asm330_data3_.acc[0]; accx_[3] = asm330_data4_.acc[0];
    accy_[0] = asm330_data1_.acc[1];    accy_[1] = asm330_data2_.acc[1];   accy_[2] = asm330_data3_.acc[1]; accy_[3] = asm330_data4_.acc[1];
    accz_[0] = asm330_data1_.acc[2];    accz_[1] = asm330_data2_.acc[2];   accz_[2] = asm330_data3_.acc[2]; accz_[3] = asm330_data4_.acc[2];
    gyrox_[0] = asm330_data1_.gyro[0];  gyrox_[1] = asm330_data2_.gyro[0]; gyrox_[2] = asm330_data3_.gyro[0];gyrox_[3] = asm330_data4_.gyro[0];
    gyroy_[0] = asm330_data1_.gyro[1];  gyroy_[1] = asm330_data2_.gyro[1]; gyroy_[2] = asm330_data3_.gyro[1];gyroy_[3] = asm330_data4_.gyro[1];
    gyroz_[0] = asm330_data1_.gyro[2];  gyroz_[1] = asm330_data2_.gyro[2]; gyroz_[2] = asm330_data3_.gyro[2];gyroz_[3] = asm330_data4_.gyro[2];
    }
	 else if(timer10ms==1)
	 {
	 accx_[0] = asm330_data1_.acc[0]+accx_[0];    accx_[1] = asm330_data2_.acc[0]+ accx_[1];   accx_[2] = asm330_data3_.acc[0]+accx_[2]; accx_[3] = asm330_data4_.acc[0]+accx_[3];
    accy_[0] = asm330_data1_.acc[1]+accy_[0];    accy_[1] = asm330_data2_.acc[1]+ accy_[1];   accy_[2] = asm330_data3_.acc[1]+accy_[2]; accy_[3] = asm330_data4_.acc[1]+accy_[3];
    accz_[0] = asm330_data1_.acc[2]+accz_[0];    accz_[1] = asm330_data2_.acc[2]+ accz_[1];   accz_[2] = asm330_data3_.acc[2]+accz_[2]; accz_[3] = asm330_data4_.acc[2]+accz_[3];
	 gyrox_[0] = asm330_data1_.gyro[0]+gyrox_[0];    gyrox_[1] = asm330_data2_.gyro[0]+ gyrox_[1];   gyrox_[2] = asm330_data3_.gyro[0]+gyrox_[2]; gyrox_[3] = asm330_data4_.gyro[0]+gyrox_[3];
    gyroy_[0] = asm330_data1_.gyro[1]+gyroy_[0];    gyroy_[1] = asm330_data2_.gyro[1]+ gyroy_[1];   gyroy_[2] = asm330_data3_.gyro[1]+gyroy_[2]; gyroy_[3] = asm330_data4_.gyro[1]+gyroy_[3];
    gyroz_[0] = asm330_data1_.gyro[2]+gyroz_[0];    gyroz_[1] = asm330_data2_.gyro[2]+ gyroz_[1];   gyroz_[2] = asm330_data3_.gyro[2]+gyroz_[2]; gyroz_[3] = asm330_data4_.gyro[2]+gyroz_[3];
	 asm330_data_mean_.acc[0] = mul1x4(accx_,&mean_cofe[12])/2 - TBias[3];//mean(accx_, SENSOR_NUM) - TBias[3];         
    asm330_data_mean_.acc[1] = mul1x4(accy_,&mean_cofe[16])/2 - TBias[4];         
    asm330_data_mean_.acc[2] = mul1x4(accz_,&mean_cofe[20])/2 - TBias[5];         
    asm330_data_mean_.gyro[0] = mul1x4(gyrox_,&mean_cofe[0])/2- TBias[0];
    asm330_data_mean_.gyro[1] = mul1x4(gyroy_,&mean_cofe[4])/2- TBias[1];
    asm330_data_mean_.gyro[2] = mul1x4(gyroz_,&mean_cofe[8])/2- TBias[2];

	 dataSend_22();
	 timer10ms=0;
	 }

	 
	 //errorcor_1();
    
//    if (hz_5_cnt < 25) 
//	 {
//        accx_5[hz_5_cnt] =    asm330_data_mean_.acc[0];  
//        accy_5[hz_5_cnt] =    asm330_data_mean_.acc[1]; 
//        accz_5[hz_5_cnt] =    asm330_data_mean_.acc[2];  
//        gyrox_5[hz_5_cnt] =   asm330_data_mean_.gyro[0]; 
//        gyroy_5[hz_5_cnt] =   asm330_data_mean_.gyro[1]; 
//        gyroz_5[hz_5_cnt++] = asm330_data_mean_.gyro[2]; 
//    }
//    if(hz_5_cnt == 25) 
//	 {
//        hz_5_cnt = 0;
//        asm330_data_mean_5.acc[0] = mean(accx_5, 25);  
//        asm330_data_mean_5.acc[1] = mean(accy_5, 25); 
//        asm330_data_mean_5.acc[2] = mean(accz_5, 25);  
//        asm330_data_mean_5.gyro[0] = mean(gyrox_5, 25); 
//        asm330_data_mean_5.gyro[1] = mean(gyroy_5, 25); 
//        asm330_data_mean_5.gyro[2] = mean(gyroz_5, 25); 
//    }
    #else
      asm330lhhx_data(ASM330PORT, &asm330_data1_, ASM330_SENSOR_1);
      asm330lhhx_data(ASM330PORT, &asm330_data2_, ASM330_SENSOR_2);
      asm330lhhx_data(ASM330PORT, &asm330_data3_, ASM330_SENSOR_3);
      asm330lhhx_data(ASM330PORT, &asm330_data4_, ASM330_SENSOR_4);
		errorcor();
     #endif
        
        //dataSend();
        
    }
    
    /* add user code end 3 */
  }
}
