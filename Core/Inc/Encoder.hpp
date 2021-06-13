/*
 * Encoder.hpp
 *
 *  Created on: Jun 13, 2021
 *      Author: under
 */

#ifndef INC_ENCODER_HPP_
#define INC_ENCODER_HPP_

#include "stm32f4xx_hal.h"

class Encoder{

private:
	uint16_t cnt_l_, cnt_r_;

public:
	Encoder();
	void init();
	void updateCnt();
	void getCnt(uint16_t &, uint16_t &);
	void clearCnt();
};



#endif /* INC_ENCODER_HPP_ */
