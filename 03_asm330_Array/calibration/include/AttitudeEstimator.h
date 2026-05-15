// Copyright (c) 2023

#ifndef ATTITUDE_ESTIMATOR_EKF_H_
#define ATTITUDE_ESTIMATOR_EKF_H_

#ifdef __cplusplus
extern "C"{
#endif

#include <string.h>
#include <math.h>
#include <stdio.h>

#include "matrix_ext.h"

#define PI              3.141592
#define RAD2DEG         (180.0/PI)
#define DEG2RAD         (PI/180.0)
#define ZERO_EPS         1.0e-6
#define BIASVAR          2e-5
#define GYROSTATIC       2e-2
#define STATICOUNT       50

extern float roll_, pitch_, yaw_ ;                      // 实时姿态角
extern float gx_bias_;
extern float gy_bias_;
extern float gz_bias_;
extern float var_;
extern float version_;

//---------------------------------------------------------------------------------------------------
// Function declarations

void attitudeEKF(float gx, float gy, float gz, float ax, float ay, float az, float dt, float gz_scale);
void mahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt);
void quaternion2Euler(float *quat, float *euler);
void euler2Quaternion(float *euler, float *quat);
void quaternion2RotationMatrix(float *quat, float *mat);
void calcGyroBias(const float * gyro,const float * acc,  const float var);
void calcSensorVar(const float * gyro, float *var);
void initParameters();
void predictionUpdate(
        float * predict_x,
        float * predict_P,
        const float * gyro);

void accObservationUpdate(
        const float * predict_x,
        const float * predict_P,
        const float * z);

void attitude(
        float * quat,
        const float dt,
        const float * acc,
        const float * gyro,
        float * euler);

void setProcessNoize(const float process_noize);
void setObservationNoize(const float observation_noize);
void calcSensorVar(const float *data, float *var);
void calcGyroBias(const float * gyro,const float * acc,  const float var);
void quaternion2RotationMatrix(float * quat, float * mat);
void initEKFParameters();
void zuptDetection();
void getVersion();

#ifdef __cplusplus
}
#endif

#endif  // ATTITUDE_ESTIMATOR_EKF_H_