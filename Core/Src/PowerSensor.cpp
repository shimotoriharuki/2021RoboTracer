/*
 * PoserSensor.cpp
 *
 *  Created on: Jun 27, 2021
 *      Author: Haruki Shimotori
 */



#include "PowerSensor.hpp"
#include "INA260.h"
#include "Macro.h"

#define LOW_VOLTAGE_THRESHOLD 7.4

float monitor_voltage;

void PowerSensor::init()
{
	INA260_init(CURRENT_VOLTAGE_SENSOR_ADRESS_LEFT);
	INA260_init(CURRENT_VOLTAGE_SENSOR_ADRESS_RIGHT);
}

void PowerSensor::updateValues()
{
	//current_l_ = INA260_read(0x01, CURRENT_VOLTAGE_SENSOR_ADRESS_LEFT) * 0.00125;
	//current_r_ = INA260_read(0x01, CURRENT_VOLTAGE_SENSOR_ADRESS_RIGHT) * 0.00125;
	buttery_voltage_ = INA260_read(0x02, CURRENT_VOLTAGE_SENSOR_ADRESS_LEFT) * 0.00125;

	monitor_voltage = buttery_voltage_;
}

void PowerSensor::getCurrentValue(float &left, float &right)
{
	left = current_l_;
	right = current_r_;
}

float PowerSensor::getButteryVoltage()
{
	return buttery_voltage_;

}

bool PowerSensor::butteryCheck()
{
	static uint16_t cnt;
	bool ret = false;

	if(buttery_voltage_ < LOW_VOLTAGE_THRESHOLD) cnt++;
	else cnt = 0;

	if(cnt >= 1) {
		ret = true;
		cnt = 1;
	}

	return ret;
}



