#include "BiasEstimate.h"

float ax_sum_ = 0.0;
float ay_sum_ = 0.0;
float az_sum_ = 0.0;
int bias_acc_count_ = 0;

//extern float lock_ax_;
//extern float lock_ay_;
//extern float lock_az_;
//extern float pre_lock_ax_;
//extern float pre_lock_ay_;
//extern float pre_lock_az_;
//extern float ax_send_;
//extern float ay_send_;
//extern float az_send_;
//extern float TEMP_BUF_N;
//extern float ax_sum;
//extern float ay_sum;
//extern float az_sum;
//extern int bias_acc_count;

void AccLock(ImuDataI imudata)
{

	static float ax_bias_ = 0.0;
	static float ay_bias_ = 0.0;
	static float az_bias_ = 0.0;
	static float ax_temp = 0.0;
	static float ay_temp = 0.0;
	static float az_temp = 0.0;
	static float ax_mean = 0.0;
	static float ay_mean = 0.0;
	static float az_mean = 0.0;
	static float RandomNumax = 0.0;
	static float RandomNumay = 0.0;
	static float RandomNumaz = 0.0;
	
	bias_acc_count_++;
	ax_sum_ += imudata.ax;
	ay_sum_ += imudata.ay;
	az_sum_ += imudata.az;
	ax_temp = imudata.ax;	
	ay_temp = imudata.ay;
	az_temp = imudata.az;
	ax_mean = ax_sum_ / bias_acc_count_;
	ay_mean = ay_sum_ / bias_acc_count_;
	az_mean = az_sum_ / bias_acc_count_;
	RandomNumax = ax_temp - ax_mean; 
	RandomNumay = ay_temp - ay_mean;
	RandomNumaz = az_temp - az_mean;

	if(bias_acc_count_ < 100)
	{
		ax_send_ = imudata.ax;
		ay_send_ = imudata.ay;
		az_send_ = imudata.az;
	}
	if(bias_acc_count_ == 100)
	{
		lock_ax_ = ax_sum_ / bias_acc_count_;
		lock_ay_ = ay_sum_ / bias_acc_count_;
		lock_az_ = az_sum_ / bias_acc_count_;
		
			if(fabs(pre_lock_ax_-lock_ax_)<=0.2 && fabs(pre_lock_ay_-lock_ay_)<=0.2 && fabs(pre_lock_az_-lock_az_)<=0.2)
   {
		lock_ax_ = pre_lock_ax_;
		lock_ay_ = pre_lock_ay_;
		lock_az_ = pre_lock_az_;
	 }
	}
	if(bias_acc_count_ > 100){
			ax_send_ = lock_ax_- TEMP_BUF_N / 3000000 + RandomNumax / 3;
			ay_send_ = lock_ay_+ TEMP_BUF_N / 3000000 + RandomNumay / 3;
			az_send_ = lock_az_+ TEMP_BUF_N / 3000000 + RandomNumaz / 3;
	}

//		if(fabs(gyro_norm) > 1)
//		{
//		bias_acc_count_ = 0;
//		ax_sum_ = 0.0;
//		ay_sum_ = 0.0;
//		az_sum_ = 0.0;
//		ax_send_ =  imudata.ax;
//		ay_send_ =	imudata.ay;
//		az_send_ =	imudata.az;
//		}

}
