/*
 * Joystick.cpp
 *
 *  Created on: Jun 11, 2021
 *      Author: under
 */

#include "Joystick.hpp"
//#include "stm32f4xx_hal.h"

JoyStick::JoyStick()
{

}

uint16_t JoyStick::getValue()
{
	uint16_t ret_value = 0;

	if(!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_10)) ret_value |= 0x01;
	if(!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_0)) ret_value |= 0x02;
	if(!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_8)) ret_value |= 0x04;
	if(!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2)) ret_value |= 0x08;
	if(!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_7)) ret_value |= 0x10;

	return ret_value;
}



