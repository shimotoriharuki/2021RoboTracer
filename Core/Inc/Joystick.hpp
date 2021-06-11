/*
 * Joystick.hpp
 *
 *  Created on: Jun 11, 2021
 *      Author: under
 */

#ifndef INC_JOYSTICK_HPP_
#define INC_JOYSTICK_HPP_

#include "stm32f4xx_hal.h"

class JoyStick{

private:

public:

	JoyStick();
	uint16_t getValue();

};



#endif /* INC_JOYSTICK_HPP_ */
