/*
 * SideSensor.cpp
 *
 *  Created on: 2021/06/11
 *      Author: under
 */

#ifndef INC_SIDESENSOR_CPP_
#define INC_SIDESENSOR_CPP_

#include "stm32f4xx_hal.h"

class SideSensor{

private:

	uint16_t status_;

public:

	SideSensor();
	void updateStatus(uint16_t);
	uint16_t status();

};


#endif /* INC_SIDESENSOR_CPP_ */