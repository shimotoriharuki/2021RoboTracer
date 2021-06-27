/*
 * IMU.cpp
 *
 *  Created on: Jun 27, 2021
 *      Author: under
 */


#include "IMU.hpp"
#include "ICM_20648.h"
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include <vector>

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
	xa_ = xa;
	ya_ = ya;
	za_ = za;
	xg_ = xg;
	yg_ = yg;
	zg_ = zg;

}

float IMU::getOmega()
{
	return zg_ - offset_;
}

void IMU::calibration()
{
	led.fullColor('G');

	std::vector<float> zg_vals;
	for(uint16_t i = 0; i < 1000; i++){
		zg_vals.push_back(zg_);
		HAL_Delay(2);
	}

	float sum;
	for(const auto &v : zg_vals){
		sum += v;
	}

	offset_ = sum / zg_vals.size();

	led.fullColor('B');
}

float IMU::getOffsetVal()
{
	return offset_;
}
