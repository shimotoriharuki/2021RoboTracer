/*
 * DownForceUnit.cpp
 *
 *  Created on: Nov 8, 2022
 *      Author: under
 */

#include "DownForceUnit.hpp"
#include "G_variables.h"

#define MAX_COUTNER_PERIOD 8999
#define LIMIT_RATIO 0.5

//---private---//
uint16_t DownForceUnit::getCounterPeriod(float ratio)
{
	return uint16_t(MAX_COUTNER_PERIOD * ratio);
}

//---public---//
DownForceUnit::DownForceUnit(){}

void DownForceUnit::init()
{
	HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
}

void DownForceUnit::on(float L, float R)
{
	if(L > LIMIT_RATIO) L = LIMIT_RATIO;
	else if(L < 0) L = 0;

	if(R > LIMIT_RATIO) R = LIMIT_RATIO;
	else if(R < 0) R = 0;


	__HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, getCounterPeriod(L));
	__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, getCounterPeriod(R));
}

void DownForceUnit::off()
{
	__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 0);
}
