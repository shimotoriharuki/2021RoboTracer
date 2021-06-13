/*
 * Motor.cpp
 *
 *  Created on: Jun 11, 2021
 *      Author: Haruki Shimotori
 */



#include "Motor.hpp"
#include "G_variables.h"


Motor::Motor() : temp_left_counter_period_(0), temp_right_counter_period_(0){}

void Motor::init()
{
	//PWM start
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

}

void Motor::motorCtrl()
{
	uint16_t left_counter_period, right_counter_period;

	if(temp_left_counter_period_ < 0) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_SET);
		left_counter_period = -1 * temp_left_counter_period_;
	}
	else{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET);
		left_counter_period = temp_left_counter_period_;
	}

	if(temp_right_counter_period_ < 0) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
		right_counter_period = -1 * temp_right_counter_period_;
	}
	else{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
		right_counter_period = temp_right_counter_period_;
	}

	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, left_counter_period);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, right_counter_period);
}

void Motor::setRatio(double left_ratio, double right_ratio)
{
	if(left_ratio > 1) left_ratio = 1;
	else if(left_ratio < -1) left_ratio = -1;
	if(right_ratio > 1) right_ratio = 1;
	else if(right_ratio < -1) right_ratio = -1;

	temp_left_counter_period_ = (int)((double)MAX_COUNTER_PERIOD * left_ratio);
	temp_right_counter_period_ = (int)((double)MAX_COUNTER_PERIOD * right_ratio);

}
