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

	void send(uint8_t *, uint16_t);
	void receive(uint8_t *, uint16_t);


public:
	MMC5983MA();

	void write(const uint8_t, uint8_t *, uint16_t);
	void read(const uint8_t, uint8_t *, uint16_t);

	void start();
	void stop();

	void updateData();
	void getData();

};


#endif /* INC_MMC5983MA_HPP_ */
