/*
 * ECU.cpp
 *
 *  Created on: Feb 4, 2022
 *      Author: under
 */

#include "ECU.hpp"
#include "G_variables.h"

ECU::ECU()
{
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
}

void ECU::on()
{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 500);
}

void ECU::off()
{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 500);

}
