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
#include "Macro.h"
#include "AQM0802.h"

float mon_sens, mon_sens_lpf;

float mon_sens0;
float mon_sens1;
float mon_sens2;
float mon_sens3;
float mon_sens4;
float mon_sens5;
float mon_sens6;
float mon_sens7;
float mon_sens8;
float mon_sens9;
float mon_sens10;
float mon_sens11;
float mon_sens12;

LineSensor::LineSensor()
{
	for(auto &av : analog_val_){
		av = 0;
	}

	for(auto &s : sensor){
		s = 0;
	}

	for(auto &m : offset_values_){
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
		store_vals_[cnt][i] = sensor_coefficient_[i] * (analog_val_[i] - offset_values_[i]) ;
		//store_vals_[cnt][i] = float(analog_val_[i]) ;
	}

	cnt++;
	if(cnt >= 10) cnt = 0;


}
void LineSensor::updateSensorValues()
{
	float temp_val[10];

	for(uint8_t ad_cnt = 0; ad_cnt < AD_DATA_SIZE; ad_cnt++){
		for(uint8_t store_cnt = 0; store_cnt < 10; store_cnt++){
			temp_val[store_cnt] = store_vals_[store_cnt][ad_cnt];
		}

		// sort
		for(uint8_t i = 0; i < 10; i++){
			for (uint8_t j = i+1; j < 10; j++) {
				if(temp_val[i] < temp_val[j]){
					float tmp = temp_val[j];
					temp_val[j] = temp_val[i];
					temp_val[i] = tmp;
				}
			}
		}

		sensor[ad_cnt] = temp_val[5];
	}

}

void LineSensor::calibration()
{
	HAL_Delay(100);

	lcd_clear();
	lcd_locate(0,0);
	lcd_printf("LineSens");
	lcd_locate(0,1);
	lcd_printf("Calib   ");


	float max_values[AD_DATA_SIZE];
	float min_values[AD_DATA_SIZE];

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

	/*
	for(const auto &m : max_values){
		printf("%f, ", m);
	}
		printf("\n");
	for(const auto &m : min_values){
		printf("%f, ", m);
	}
		printf("\n");
	*/

	for(uint16_t i = 0; i < AD_DATA_SIZE; i++){
		sensor_coefficient_[i] = 1000 / (max_values[i] - min_values[i]);
	}
	for(uint16_t i = 0; i < AD_DATA_SIZE; i++){
		offset_values_[i] = min_values[i];
	}


}

void LineSensor::printSensorValues()
{
	//printf("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", sensor[0], sensor[1], sensor[2], sensor[3], sensor[4], sensor[5], sensor[6], sensor[7], sensor[8], sensor[9], sensor[10], sensor[11], sensor[12], sensor[13]);
}

bool LineSensor::emergencyStop()
{
	uint16_t out_cnt = 0;
	static uint16_t cnt = 0;
	static bool flag = false;

	/*
	for(const auto & s : sensor){
		if(s >= 550) out_cnt++;
	}
	*/
	for(uint16_t i = 3; i <= 10; i++){
		if(sensor[i] >= 550) out_cnt++;
	}

	if(out_cnt >= 8){
		cnt++;
	}
	else{
		cnt = 0;
	}

	if(cnt >= 100){
		flag = true;
	}
	else flag = false;

	if(cnt >= 10000) cnt = 10000;

	return flag;

}
