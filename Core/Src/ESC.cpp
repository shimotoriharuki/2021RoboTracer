/*
 * ESC.cpp
 *
 *  Created on: Feb 4, 2022
 *      Author: under
 */

#include "ESC.hpp"
#include "G_variables.h"

#define ESC_MIN_TIM3 1889
#define ESC_MAX_TIM3 3779
#define ESC_MIN_TIM10_11 3770
#define ESC_MAX_TIM10_11 7559

//---private---//

uint16_t ESC::getCounterPeriodTIM3(float ratio)
{
	return int((ESC_MAX_TIM3 - ESC_MIN_TIM3) * ratio + ESC_MIN_TIM3);
}

uint16_t ESC::getCounterPeriodTIM10_11(float ratio)
{
	return int((ESC_MAX_TIM10_11 - ESC_MIN_TIM10_11) * ratio + ESC_MIN_TIM10_11);
}

//---public---//
ESC::ESC()
{

}

void ESC::init()
{
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ESC_MIN_TIM3);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ESC_MIN_TIM3);
	__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, ESC_MIN_TIM10_11);
	__HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, ESC_MIN_TIM10_11);
}

void ESC::on(float FL, float FR, float RL, float RR)
{
	if(FL > 1.0) FL = 1.0;
	else if(FL < 0) FL = 0;

	if(FR > 1.0) FR = 1.0;
	else if(FR < 0) FR = 0;

	if(RL > 1.0) RL = 1.0;
	else if(RL < 0) RL = 0;

	if(RR > 1.0) RR = 1.0;
	else if(RR < 0) RR = 0;

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, getCounterPeriodTIM3(FL));
	__HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, getCounterPeriodTIM10_11(FR));
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, getCounterPeriodTIM3(RL));
	__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, getCounterPeriodTIM10_11(RR));
}

void ESC::off()
{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ESC_MIN_TIM3);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ESC_MIN_TIM3);
	__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, ESC_MIN_TIM10_11);
	__HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, ESC_MIN_TIM10_11);
}
