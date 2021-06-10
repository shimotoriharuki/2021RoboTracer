/*
 * wrapper.cpp
 *
 *  Created on: Jun 9, 2021
 *      Author: under
 */

#include "wrapper.hpp"
#include <stdio.h>
#include "LineSensor.hpp"

LineSensor line_sensor;

void cppInit(void)
{
	line_sensor.ADCStart();

}
void cppLoop(void)
{
	printf("cpp loop test\n");
	printf("cpp AD %d\n", line_sensor.sensor[0]);
}

void cppFlip(void)
{
	line_sensor.updateSensorvaluses();
}

/*
//LineSensor.hpp
void ADCStart(void) {
    LineSensor instance;

    instance.ADCStart();
}
void updateSensorvaluses(void) {
    LineSensor instance;

    instance.updateSensorvaluses();
}
*/



