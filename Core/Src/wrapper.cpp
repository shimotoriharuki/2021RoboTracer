/*
 * wrapper.cpp
 *
 *  Created on: Jun 9, 2021
 *      Author: under
 */

#include "wrapper.hpp"
#include <stdio.h>

#include "LineSensor.hpp"
#include "SideSensor.hpp"
#include "Joystick.hpp"
#include "RotarySwitch.hpp"
#include "Motor.hpp"

LineSensor line_sensor;
SideSensor side_sensor;
JoyStick joy_stick;
RotarySwitch rotary_switch;
Motor motor;

void cppInit(void)
{
	line_sensor.ADCStart();
	motor.init();

}

void cppFlip(void)
{
	line_sensor.updateSensorvaluses();
	motor.motorCtrl();

}

void cppExit(uint16_t gpio_pin)
{
	side_sensor.updateStatus(gpio_pin);

}

void cppLoop(void)
{
	printf("cpp loop test\n");
	printf("cpp AD %d\n", line_sensor.sensor[0]);
	printf("cpp side: %d\n", side_sensor.status());
	printf("cpp joystick: %d\n", joy_stick.getValue());
	printf("cpp joystick: %d\n", joy_stick.getValue());
	printf("cpp rotaryswitch: %d\n", rotary_switch.getValue());

	motor.setRatio(0, 1.0);


	HAL_Delay(1000);

	//motor.setRatio(0, -0.5);

	HAL_Delay(1000);




}




