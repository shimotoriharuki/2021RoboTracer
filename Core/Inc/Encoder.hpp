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
	double cnt_l_, cnt_r_;
	double distance_; //[mm]
	double total_cnt_l_, total_cnt_r_;
	double total_distance_;

public:
	Encoder();
	void init();
	void updateCnt();
	void getCnt(double &, double &);
	double getDistance();
	double getTotalDistance();
	void clearDistance();
	void clearCnt();
	double getTotalCnt();
	void clearTotalCnt();
};

#endif /* INC_ENCODER_HPP_ */
