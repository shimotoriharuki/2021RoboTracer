/*
 * BZ.hpp
 *
 *  Created on: 2021/12/14
 *      Author: under
 */

#ifndef INC_BZ_HPP_
#define INC_BZ_HPP_

#include "stm32f4xx_hal.h"
#define MAX_COUNTER_PERIOD_TIM1 1800

class BZ{
private:

public:
	BZ();
	void on();
	void off();

};



#endif /* INC_BZ_HPP_ */
