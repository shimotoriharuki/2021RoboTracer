/*
 * wrapper.hpp
 *
 *  Created on: Jun 9, 2021
 *      Author: under
 */

#ifndef INC_WRAPPER_HPP_
#define INC_WRAPPER_HPP_

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

void cppInit(void);
void cppLoop(void);
void cppFlip(void);
void cppExit(uint16_t);
//LineSensor.hpp
/*
void ADCStart(void);
void updateSensorvaluses(void);
*/
#ifdef __cplusplus
};
#endif



#endif /* INC_WRAPPER_HPP_ */
