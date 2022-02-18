/*
 * Motor.hpp
 *
 *  Created on: Jun 11, 2021
 *      Author: Haruki Shimotori
 */

#ifndef INC_MOTOR_HPP_
#define INC_MOTOR_HPP_

#include "stm32f4xx_hal.h"

#define MAX_COUNTER_PERIOD 1800

class Motor{
private:

	//void speedCtrl();
	int16_t temp_left_counter_period_, temp_right_counter_period_;

public:

	Motor();
	void init();
	void motorCtrl(); //call by timer interrupt in
	void setRatio(double, double);
	int16_t getLeftCounterPeriod();
	int16_t getRightCounterPeriod();

};



#endif /* INC_MOTOR_HPP_ */
