/*
 * BZ.cpp
 *
 *  Created on: 2021/12/14
 *      Author: under
 */

#include "BZ.hpp"
#include "G_variables.h"

BZ::BZ()
{
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
}

void BZ::on()
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, int(MAX_COUNTER_PERIOD_TIM1/2));
}

void BZ::off()
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
}


