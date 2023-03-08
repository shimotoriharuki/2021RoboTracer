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
#include "AQM0802.h"

#define PI 3.1415926535
#define DELTA_T 0.001

float mon_zg, mon_omega;

IMU::IMU() : xa_(0), ya_(0), za_(0), xg_(0), yg_(0), zg_(0), omega_(0), offset_(0), constant_distance_theta_(0)
{

}

void IMU::init()
{
	uint16_t who_i_am;
	who_i_am = IMU_init();

	lcd_clear();
	lcd_locate(0,0);
	lcd_printf("IMUstatus");
	lcd_locate(0,1);
	lcd_printf("%d", who_i_am);

	HAL_Delay(500);

}

void IMU::updateValues()
{
	read_gyro_data();
	//read_accel_data();

	static int16_t pre_zg;
	zg_ = int((R_IMU)*(zg) + (1.0 - (R_IMU))* (pre_zg)); // lowpath filter

	pre_zg = zg_;
	mon_zg= zg_;

	float corrected_zg = float(zg_) - offset_;
	omega_ = -(corrected_zg / 16.4) * PI / 180;
	mon_omega = omega_;


	float delta_theta_ = omega_ * DELTA_T;
	constant_distance_theta_= constant_distance_theta_ + delta_theta_;

}

float IMU::getOmega()
{
	return omega_;
}

void IMU::calibration()
{
	HAL_Delay(800);

	lcd_clear();
	lcd_locate(0,0);
	lcd_printf("IMU     ");
	lcd_locate(0,1);
	lcd_printf("Calib   ");

	int16_t num = 2000;
	float zg_vals[num];
	for(uint16_t i = 0; i < num; i++){
		zg_vals[i] = float(zg_);
		HAL_Delay(1);
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

double IMU::getConstantDistanceTheta()
{
	return constant_distance_theta_;
}

void IMU::clearConstantDistanceTheta()
{
	constant_distance_theta_ = 0;
}
