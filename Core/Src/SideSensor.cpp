/*
 * SideSensor.cpp
 *
 *  Created on: 2021/06/11
 *      Author: under
 */

#include "SideSensor.hpp"

SideSensor::SideSensor()
{

}


void SideSensor::updateStatus(uint16_t gpio_pin)
{
	static bool white_flag1 = false;
	static bool white_flag2 = false;

	if (gpio_pin == GPIO_PIN_2 && white_flag1 == false){
		status_ |= 0x01;
		white_flag1 = true;
	}
	else if(gpio_pin == GPIO_PIN_2 && white_flag1 == true){
		status_ ^= 0x01;
		white_flag1 = false;

	}

	if (gpio_pin == GPIO_PIN_8 && white_flag2 == false){
		status_ |= 0x02;
		white_flag2 = true;
	}
	else if(gpio_pin == GPIO_PIN_8 && white_flag2 == true){
		status_ ^= 0x02;
		white_flag2 = false;
	}

}

uint16_t SideSensor::status()
{
	return status_;
}



