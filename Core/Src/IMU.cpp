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

uint16_t mon_zg;

IMU::IMU() : array_idx(0), offset_(0)
{
	for(uint16_t i = 0; i < STORE_NUM; i++){
		xa_store_[i] = 0;
		ya_store_[i] = 0;
		za_store_[i] = 0;
		xg_store_[i] = 0;
		yg_store_[i] = 0;
		zg_store_[i] = 0;
	}

}

void IMU::init()
{
	uint16_t who_i_am;
	who_i_am = IMU_init();
	printf("who i am: %d\n", who_i_am);

}

void IMU::storeValues()
{
	read_gyro_data();
	//read_accel_data();

	xa_store_[array_idx] = xa;
	ya_store_[array_idx] = ya;
	za_store_[array_idx] = za;
	xg_store_[array_idx] = xg;
	yg_store_[array_idx] = yg;
	zg_store_[array_idx] = zg;

	if(array_idx >= STORE_NUM) array_idx = 0;

}
void IMU::updateValues()
{
	// heap value
	float temp_val[10];
	for(uint8_t i = 0; i < STORE_NUM; i++){
		temp_val[i] = zg_store_[i];
	}

	// sort
	for(uint8_t i = 0; i < STORE_NUM; i++){
		for (uint8_t j = i+1; j < STORE_NUM; j++) {
			if(temp_val[i] < temp_val[j]){
				float tmp = temp_val[j];
				temp_val[j] = temp_val[i];
				temp_val[i] = tmp;
			}
		}
	}

	zg_ = temp_val[2];
	mon_zg = zg_;

}

float IMU::getOmega()
{
	float corrected_zg = float(zg_) - offset_;
	return -(corrected_zg / 16.4) * PI / 180;
}

void IMU::calibration()
{
	HAL_Delay(1000);

	int16_t num = 2000;
	float zg_vals[num];
	for(uint16_t i = 0; i < num; i++){
		zg_vals[i] = float(zg_);
		HAL_Delay(2);
	}

	float sum;
	for(const auto &v : zg_vals){
		sum += v;
	}

	offset_ = sum / num;
}

float IMU::getOffsetVal()
{
	return offset_;
}
