/*
 * INA260.h
 *
 *  Created on: Jun 26, 2021
 *      Author: under
 */

#ifndef SRC_INA260_H_
#define SRC_INA260_H_

#include "main.h"

extern I2C_HandleTypeDef hi2c2;

unsigned short INA260_read(uint8_t, uint8_t);
void INA260_write(uint8_t,uint8_t,uint8_t, uint8_t);
void INA260_init(uint8_t);



#endif /* SRC_INA260_H_ */
