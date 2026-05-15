#include "tempCalibration.h"
#include "tempcalibrationData.h"

void tempCalibration(float temp, float *tbias) {
    int num = 0;
    if (temp_minus_inf_35.topTemp >= temp) {
        num = 0;
    } else if (temp_minus_35_25.topTemp >= temp && temp_minus_35_25.bottomTemp <= temp) {
        num = 1;
    } else if (temp_minus_25_15.topTemp >= temp && temp_minus_25_15.bottomTemp <= temp) {
        num = 2;
    } else if (temp_minus_15_5.topTemp >= temp && temp_minus_15_5.bottomTemp <= temp) {
        num = 3;
    } else if (temp_minus_5_5.topTemp >= temp && temp_minus_5_5.bottomTemp <= temp) {
        num = 4;
    } else if (temp_5_15.topTemp >= temp && temp_5_15.bottomTemp <= temp) {
        num = 5;
    } else if (temp_15_25.topTemp >= temp && temp_15_25.bottomTemp <= temp) {
        num = 6;
    } else if (temp_25_35.topTemp >= temp && temp_25_35.bottomTemp <= temp) {
        num = 7;
    } else if (temp_35_45.topTemp >= temp && temp_35_45.bottomTemp <= temp) {
        num = 8;
    } else if (temp_45_55.topTemp >= temp && temp_45_55.bottomTemp <= temp) {
        num = 9;
    } else if (temp_55_65.topTemp >= temp && temp_55_65.bottomTemp <= temp) {
        num = 10;
    } else if (temp_65_75.topTemp >= temp && temp_65_75.bottomTemp <= temp) {
        num = 11;
    } else if (temp_75_inf.topTemp >= temp && temp_75_inf.bottomTemp <= temp) {
        num = 12;
    }

    for (int i = 0; i < 6; i++) {
        tbias[i] = ((TEMPCALIBRATION *)tempCaliBuff[num])->k[i] * temp + ((TEMPCALIBRATION *)tempCaliBuff[num])->b[i];
    };
}

