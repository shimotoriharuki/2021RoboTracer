/*
 * LineSensor.h
 *
 *  Created on: 2021/06/09
 *      Author: under
 */

#ifndef INC_LINESENSOR_HPP_
#define INC_LINESENSOR_HPP_u

#include "stm32f4xx_hal.h"
#include "Macro.h"

class LineSensor {

public:

	uint16_t sensor[AD_DATA_SIZE];
	void ADCStart();
	void updateSensorvaluses();


};



#endif /* INC_LINESENSOR_HPP_ */
