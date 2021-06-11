/*
 * RotarySwitch.cpp
 *
 *  Created on: Jun 11, 2021
 *      Author: under
 */

#include "RotarySwitch.hpp"


uint16_t RotarySwitch::getValue()
{
	uint16_t ret_value = 0;

	if(!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_1))	ret_value |= 0x01;
	if(!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_3))	ret_value |= 0x02;
	if(!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_4))	ret_value |= 0x04;
	if(!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_7))	ret_value |= 0x08;

	return ret_value;

}
