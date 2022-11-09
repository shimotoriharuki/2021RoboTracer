/*
 * DownForceUnit.hpp
 *
 *  Created on: Nov 8, 2022
 *      Author: under
 */

#ifndef INC_DOWNFORCEUNIT_HPP_
#define INC_DOWNFORCEUNIT_HPP_

#include "stm32f4xx_hal.h"

class DownForceUnit{
private:
	uint16_t getCounterPeriod(float);

public:
	DownForceUnit();
	void init();
	void on(float, float);
	void off();

};




#endif /* INC_DOWNFORCEUNIT_HPP_ */
