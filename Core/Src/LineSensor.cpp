/*
 * LineSensor.cpp
 *
 *  Created on: 2021/06/09
 *      Author: Haruki Shimotori
 */

#include <stdio.h>
#include <LineSensor.hpp>
//#include "stm32f4xx_hal.h"
#include "G_variables.h"
//#include "Macro.h"

//int analog[AD_DATA_SIZE];

LineSensor::LineSensor()
{
	for(auto &av : analog_val_){
		av = 0;
	}
	for(auto &s : sensor){
		s = 0;
	}

}

void LineSensor::ADCStart()
{
	//printf("class test\n");
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

void LineSensor::calibration()
{
	//uint16_t max_values[AD_DATA_SIZE];
	//uint16_t min_values[AD_DATA_SIZE];
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


}



