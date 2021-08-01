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
	int16_t cnt_l_, cnt_r_;
	float distance_;

public:
	Encoder();
	void init();
	void updateCnt();
	void getCnt(int16_t &, int16_t &);
	float getDistance();
	void clearCnt();
};

#endif /* INC_ENCODER_HPP_ */
