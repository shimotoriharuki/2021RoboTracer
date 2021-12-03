/*
 * Encoder.hpp
 *
 *  Created on: Jun 13, 2021
 *      Author: under
 */

#ifndef INC_ENCODER_HPP_
#define INC_ENCODER_HPP_

#include "stm32f4xx_hal.h"

#define R 0.05 //Lowpath filter constant. The smaller it is, the more effective/

class Encoder{

private:
	float cnt_l_, cnt_r_;
	float distance_; //[mm]
	float total_cnt_l_, total_cnt_r_;
	float total_distance_;

public:
	Encoder();
	void init();
	void updateCnt();
	void getCnt(float &, float &);
	float getDistance();
	float getTotalDistance();
	void clearDistance();
	void clearCnt();
	float getTotalCnt();
	void clearTotalCnt();
};

#endif /* INC_ENCODER_HPP_ */
