#ifndef ERRORCOR_H
#define ERRORCOR_H
#include "asm330lhhx.h"
void SixPosition_cor(float Ax,float Ay,float Az,float* ax,float* ay,float* az, int imu_num);
void gyro_calibration(float gyr_x, float gyr_y, float gyr_z, float *Gx, float *Gy, float *Gz, int imu_num);
float getA(float arcs[3][3], int n);
void getAStart(float arcs[3][3], int n, float ans[3][3]);
void Axial_Decoupling(float K_cail[18],int pattren_num);
void errorcor();
void errorcor_1();
void Senddata_A5(void);
void SystemClock_Config(void);
void Senddata(void);
void mag_correct(float Mag_x,float Mag_y,float Mag_z,float* mag_x,float* mag_y,float* mag_z);
 extern unsigned char TIMER_5ms,
											TIMER_10ms,
											TIMER_20ms,
											TIMER_40ms,
											TIMER_50ms,
											TIMER_100ms,
											TIMER_200ms,
											TIMER_500ms,
											TIMER_1s;
extern float acc_614[3], gyro_[3], acc_375[3];
extern float Init_flag , Pre_Init_flag ;
extern float Acc_x_614, Acc_y_614, Acc_z_614, Gyr_x, Gyr_y, Gyr_z, Acc_x_375, Acc_y_375, Acc_z_375;
extern ASM330DATAOP asm330_data1_;
extern ASM330DATAOP asm330_data2_;
extern ASM330DATAOP asm330_data3_;
extern ASM330DATAOP asm330_data4_;
extern ASM330DATAOP asm330_data_mean_;
#endif