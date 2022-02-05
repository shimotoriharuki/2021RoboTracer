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

public:
	ESC();
	void init();
	void on();
	void off();

};



#endif /* INC_ESC_HPP_ */
