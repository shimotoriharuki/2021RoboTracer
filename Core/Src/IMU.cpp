/*
 * IMU.cpp
 *
 *  Created on: Jun 27, 2021
 *      Author: Haruki Shimotori
 */


#include "IMU.hpp"
#include "ICM_20648.h"
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include <vector>

#define PI 3.1415926535

IMU::IMU() : xa_(0), ya_(0), za_(0), xg_(0), yg_(0), zg_(0), offset_(0)
{

}

void IMU::init()
{
	uint16_t who_i_am;
	who_i_am = IMU_init();
	printf("who i am: %d\n", who_i_am);

}

void IMU::updateValues()
{
	read_gyro_data();
	read_accel_data();

	xa_ = xa;
	ya_ = ya;
	za_ = za;
	xg_ = xg;
	yg_ = yg;
	zg_ = zg;

}

double IMU::getOmega()
{
	double corrected_zg = double(zg_) - offset_;
	return -(corrected_zg / 16.4) * PI / 180;
}

void IMU::calibration()
{
	HAL_Delay(1000);

	int16_t num = 2000;
	double zg_vals[num];
	for(uint16_t i = 0; i < num; i++){
		zg_vals[i] = double(zg_);
		HAL_Delay(2);
	}

	float sum;
	for(const auto &v : zg_vals){
		sum += v;
	}

	offset_ = sum / num;
}

double IMU::getOffsetVal()
{
	return offset_;
}
