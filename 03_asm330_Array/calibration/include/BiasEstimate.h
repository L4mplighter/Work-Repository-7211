//
// Created by Roman on 2024/3/23.
//

#ifndef ATTITUDE_BIASESTIMATE_H
#define ATTITUDE_BIASESTIMATE_H

#ifdef __cplusplus
extern "C"{
#endif

#include <math.h>
#include "SensorData.h"

#define ZERO_EPS         1.0e-6

extern float real_gx_bias_; // unit:deg/s
extern float real_gy_bias_;
extern float real_gz_bias_;
extern float real_ax_bias_; // unit:g
extern float real_ay_bias_;
extern float real_az_bias_;

void calibrationImuData(ImuDataI imudata, float tempature);

void temperatureCompensateBias(float temperature, int imuNumber);

void ImuBiasCompensate(ImuDataI imudata, float temperature, int imuNumber);

void calcSensorVar(const float * gyro, float *var);

void imuBiasDataBase(int imuNumber);

int findIndex(float curTempature);

#ifdef __cplusplus
}
#endif

#endif //ATTITUDE_BIASESTIMATE_H
