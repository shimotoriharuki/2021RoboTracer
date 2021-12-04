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
#include <vector>

#define R_LINESENSE 0.05

class LineSensor {

private:
	uint16_t analog_val_[AD_DATA_SIZE];
	float store_vals_[10][AD_DATA_SIZE];
	float sensor_coefficient_[AD_DATA_SIZE];
	float offset_values_[AD_DATA_SIZE];

	LED led_;
	JoyStick joy_stick_;
	RotarySwitch rotary_switch_;


public:

	float sensor[AD_DATA_SIZE];

	LineSensor();
	void ADCStart();
	void storeSensorValues();
	void updateSensorValues();
	void calibration();
	void printSensorValues();
	bool emergencyStop();

};

#endif /* INC_LINESENSOR_HPP_ */
