/*
 * LineSensor.cpp
 *
 *  Created on: 2021/06/09
 *      Author: Haruki Shimotori
 */

#include <stdio.h>
#include <LineSensor.hpp>
#include "G_variables.h"

LineSensor::LineSensor()
{
	for(auto &av : analog_val_){
		av = 0;
	}
	for(auto &s : sensor){
		s = 0;
	}
	for(auto &m : offset_values){
		m = 0;
	}
	for(auto &s : sensor_coefficient_){
		s = 1;
	}

}

void LineSensor::ADCStart()
{
	HAL_ADC_Start_DMA(&hadc2, (uint32_t *) analog_val_, 14);
}

void LineSensor::updateSensorvaluses()
{
	for(uint8_t i = 0; i < AD_DATA_SIZE; i++){
		sensor[i] = (analog_val_[i] - offset_values[i]) * sensor_coefficient_[i];
	}

}

void LineSensor::calibration()
{
	HAL_Delay(100);

	uint16_t max_values[AD_DATA_SIZE];
	uint16_t min_values[AD_DATA_SIZE];

	for(uint16_t i = 0; i < AD_DATA_SIZE; i++){
		max_values[i] = sensor[i];
		min_values[i] = sensor[i];
	}

	while(joy_stick_.getValue() != JOY_C){

		for(uint16_t i = 0; i < AD_DATA_SIZE; i++){
			if(max_values[i] < sensor[i]){
				max_values[i] = sensor[i];
			}
			else if(min_values[i] > sensor[i]){
				min_values[i] = sensor[i];
			}
		}

		if(rotary_switch_.getValue() == 0){
			led_.LR(-1, 1);

		}
		else{
			led_.LR(-1, 0);

		}
	}

	for(const auto &m : max_values){
		printf("%d, ", m);
	}
		printf("\n");
	for(const auto &m : min_values){
		printf("%d, ", m);
	}
		printf("\n");


	for(uint16_t i = 0; i < AD_DATA_SIZE; i++){
		sensor_coefficient_[i] = 1000 / (max_values[i] - min_values[i]);
	}
	for(uint16_t i = 0; i < AD_DATA_SIZE; i++){
		offset_values[i] = min_values[i];
	}

}

void LineSensor::printSensorValues(){
	printf("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n", sensor[0], sensor[1], sensor[2], sensor[3], sensor[4], sensor[5], sensor[6], sensor[7], sensor[8], sensor[9], sensor[10], sensor[11], sensor[12], sensor[13]);
}



