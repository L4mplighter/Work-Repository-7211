// Copyright (c) 2023

#ifndef _SensorData_h_
#define _SensorData_h_

typedef struct tagImuDataIns
{
	float timestamp;  // unit: us
	float dt;         // unit: second, resoluton: 0.001s
	float gx;
	float gy;
	float gz;
	float ax;
	float ay;
	float az;
	int isUpdate;
} ImuDataI;

#endif  // _SensorData_h_

