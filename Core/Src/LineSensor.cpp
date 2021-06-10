/*
 * LineSensor.cpp
 *
 *  Created on: 2021/06/09
 *      Author: under
 */

#include <stdio.h>
#include <LineSensor.hpp>
//#include "stm32f4xx_hal.h"
#include "G_variables.h"
//#include "Macro.h"

//int analog[AD_DATA_SIZE];

void LineSensor::ADCStart()
{
	printf("class test\n");
	HAL_ADC_Start_DMA(&hadc2, (uint32_t *) analog_val_, 14);
}

void LineSensor::updateSensorvaluses()
{
	sensor[0] = analog_val_[0];
	sensor[1] = analog_val_[1];
	sensor[2] = analog_val_[2];
	sensor[3] = analog_val_[3];
	sensor[4] = analog_val_[4];
	sensor[5] = analog_val_[5];
	sensor[6] = analog_val_[6];
	sensor[7] = analog_val_[7];
	sensor[8] = analog_val_[8];
	sensor[9] = analog_val_[9];
	sensor[10] = analog_val_[10];
	sensor[11] = analog_val_[11];
	sensor[12] = analog_val_[12];
	sensor[13] = analog_val_[13];

}



