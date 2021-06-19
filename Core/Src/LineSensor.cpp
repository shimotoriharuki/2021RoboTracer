/*
 * LineSensor.cpp
 *
 *  Created on: 2021/06/09
 *      Author: Haruki Shimotori
 */

#include <stdio.h>
#include <LineSensor.hpp>
#include <algorithm>
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

void LineSensor::storeSensorValues()
{
	static uint8_t cnt = 0;

	for(int i = 0; i < AD_DATA_SIZE; i++){
		store_vals_[cnt][i] = analog_val_[i];
	}

	cnt++;
	if(cnt >= 10) cnt = 0;


}
void LineSensor::updateSensorvaluses()
{
	uint16_t temp_val[10];

	for(uint8_t ad_cnt = 0; ad_cnt < AD_DATA_SIZE; ad_cnt++){
		for(uint8_t store_cnt = 0; store_cnt < 10; store_cnt++){
			temp_val[store_cnt] = store_vals_[store_cnt][ad_cnt];
		}

		std::sort(temp_val, temp_val + AD_DATA_SIZE);
		sensor[ad_cnt] = temp_val[5];
	}


	for(uint8_t store_cnt = 0; store_cnt < 10; store_cnt++){
		for(uint8_t ad_cnt = 0; ad_cnt < AD_DATA_SIZE; ad_cnt++){
			printf("%d\n", store_vals_[store_cnt][ad_cnt]);
		}
		printf("\n");
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
	printf("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", sensor[0], sensor[1], sensor[2], sensor[3], sensor[4], sensor[5], sensor[6], sensor[7], sensor[8], sensor[9], sensor[10], sensor[11], sensor[12], sensor[13]);
}



