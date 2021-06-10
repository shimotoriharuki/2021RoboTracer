/*
 * LineSensor.cpp
 *
 *  Created on: 2021/06/09
 *      Author: under
 */

#include <stdio.h>
#include <LineSensor.hpp>
#include "stm32f4xx_hal.h"
#include "G_variables.h"
#include "Macro.h"

int analog[AD_DATA_SIZE];

void LineSensor::ADCStart()
{
	printf("class test\n");
	HAL_ADC_Start_DMA(&hadc2, (uint32_t *) analog, 14);
}
void LineSensor::updateSensorvaluses()
{


}



