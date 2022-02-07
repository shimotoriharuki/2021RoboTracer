/*
 * ESC.hpp
 *
 *  Created on: Feb 4, 2022
 *      Author: under
 */

#ifndef INC_ESC_HPP_
#define INC_ESC_HPP_

#include "stm32f4xx_hal.h"

class ESC{
private:

	uint16_t getCounterPeriodTIM3(float);
	uint16_t getCounterPeriodTIM10_11(float);

public:
	ESC();
	void init();
	void on(float, float, float, float);
	void off();

};



#endif /* INC_ESC_HPP_ */
