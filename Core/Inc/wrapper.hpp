/*
 * wrapper.hpp
 *
 *  Created on: Jun 9, 2021
 *      Author: Haruki Shimotori
 */

#ifndef INC_WRAPPER_HPP_
#define INC_WRAPPER_HPP_

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

void batteryLowMode(void);
void cppInit(void);
void cppLoop(void);
void cppFlip1ms(void);
void cppFlip100ns(void);
void cppFlip10ms(void);
void cppExit(uint16_t);
void parameterSettingMode(void);

#ifdef __cplusplus
};
#endif



#endif /* INC_WRAPPER_HPP_ */
