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

LineSensor line_sensor;
SideSensor side_sensor;

void cppInit(void)
{
	line_sensor.ADCStart();

}

void cppFlip(void)
{
	line_sensor.updateSensorvaluses();
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
}




