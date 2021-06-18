/*
 * LineSensor.h
 *
 *  Created on: 2021/06/09
 *      Author: Haruki Shimotori
 */

#ifndef INC_LINESENSOR_HPP_
#define INC_LINESENSOR_HPP_

#include "stm32f4xx_hal.h"
#include "Macro.h"
#include "LED.hpp"
#include "Joystick.hpp"
#include "RotarySwitch.hpp"


class LineSensor {

private:
	uint16_t analog_val_[AD_DATA_SIZE];
	float sensor_coefficient_[AD_DATA_SIZE];

	LED led_;
	JoyStick joy_stick_;
	RotarySwitch rotary_switch_;


public:

	uint16_t sensor[AD_DATA_SIZE];
	uint16_t max_values[AD_DATA_SIZE];
	uint16_t min_values[AD_DATA_SIZE];

	LineSensor();
	void ADCStart();
	void updateSensorvaluses();
	void calibration();

};

#endif /* INC_LINESENSOR_HPP_ */
