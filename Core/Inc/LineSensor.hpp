/*
 * LineSensor.h
 *
 *  Created on: 2021/06/09
 *      Author: under
 */

#ifndef INC_LINESENSOR_HPP_
#define INC_LINESENSOR_HPP_

#include "stm32f4xx_hal.h"
#include "Macro.h"


class LineSensor {

private:
	uint16_t analog_val_[AD_DATA_SIZE];

public:

	uint16_t sensor[AD_DATA_SIZE];

	LineSensor();
	void ADCStart();
	void updateSensorvaluses();

};

#endif /* INC_LINESENSOR_HPP_ */
