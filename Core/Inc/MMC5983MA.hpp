/*
 * MMC5983MA.hpp
 *
 *  Created on: 2023/04/23
 *      Author: under
 */

#ifndef INC_MMC5983MA_HPP_
#define INC_MMC5983MA_HPP_

#include "main.h"

class MMC5983MA{
private:
	I2C_HandleTypeDef hi2c1;

	void send(uint8_t, uint8_t *, uint16_t);
	void read(uint8_t, uint8_t *, uint16_t);

public:
	MMC5983MA();
	void start();
	void stop();
	void updateData();
	void getData();

};


#endif /* INC_MMC5983MA_HPP_ */
