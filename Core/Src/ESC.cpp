/*
 * ESC.cpp
 *
 *  Created on: Feb 4, 2022
 *      Author: under
 */

#include "ESC.hpp"
#include "G_variables.h"

#define ESC_MAX 84
#define ESC_MIN 42

ESC::ESC()
{

}

void ESC::init()
{
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
}

void ESC::on()
{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ESC_MIN);
}

void ESC::off()
{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);

}
