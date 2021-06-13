/*
 * LED.hpp
 *
 *  Created on: Jun 13, 2021
 *      Author: under
 */

#ifndef INC_LED_HPP_
#define INC_LED_HPP_

#include "stm32f4xx_hal.h"

class LED{
private:

public:
	void fullColor(char);
	void LR(int8_t, int8_t);

};



#endif /* INC_LED_HPP_ */
