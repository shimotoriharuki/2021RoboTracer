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
	sensor[0] = analog_val_[0] * sensor_coefficient_[0];
	sensor[1] = analog_val_[1] * sensor_coefficient_[1];
	sensor[2] = analog_val_[2] * sensor_coefficient_[2];
	sensor[3] = analog_val_[3] * sensor_coefficient_[3];
	sensor[4] = analog_val_[4] * sensor_coefficient_[4];
	sensor[5] = analog_val_[5] * sensor_coefficient_[5];
	sensor[6] = analog_val_[6] * sensor_coefficient_[6];
	sensor[7] = analog_val_[7] * sensor_coefficient_[7];
	sensor[8] = analog_val_[8] * sensor_coefficient_[8];
	sensor[9] = analog_val_[9] * sensor_coefficient_[9];
	sensor[10] = analog_val_[10] * sensor_coefficient_[10];
	sensor[11] = analog_val_[11] * sensor_coefficient_[11];
	sensor[12] = analog_val_[12] * sensor_coefficient_[12];
	sensor[13] = analog_val_[13] * sensor_coefficient_[13];

}

void LineSensor::calibration()
{
	HAL_Delay(100);

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

}

void LineSensor::printSensorValues(){
	printf("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n", sensor[0], sensor[1], sensor[2], sensor[3], sensor[4], sensor[5], sensor[6], sensor[7], sensor[8], sensor[9], sensor[10], sensor[11], sensor[12], sensor[13]);
}



