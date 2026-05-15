#ifndef __TEMPCALIBRATION_H_
#define __TEMPCALIBRATION_H_
#include "at32f435_437.h"
#include "asm330lhhx.h"

extern ASM330DATAOP asm330_data_mean_;

typedef struct  {
    float k[6];
    float b[6];
    float topTemp;
    float bottomTemp;
} TEMPCALIBRATION;

void tempCalibration(float temp, float *tbias);


#endif