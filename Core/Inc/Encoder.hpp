/*
 * Encoder.hpp
 *
 *  Created on: Jun 13, 2021
 *      Author: under
 */

#ifndef INC_ENCODER_HPP_
#define INC_ENCODER_HPP_

#include "stm32f4xx_hal.h"

#define R_ENC 0.05 //Lowpath filter constant. The smaller it is, the more effective/

class Encoder{

private:
	float cnt_l_, cnt_r_;
	float distance_; //[mm]
	float total_cnt_l_, total_cnt_r_;
	float distance_10mm_;
	float total_distance_;
	float cross_line_ignore_distance_;
	float stable_distance_;

public:
	Encoder();
	void init();
	void update();
	void clear();
	void getCnt(float &, float &);
	float getDistance();
	float getDistance10mm();
	float getTotalDistance();
	void setTotalDistance(float);
	void clearDistance();
	//float getTotalCnt();
	void clearDistance10mm();
	void clearTotalDistance();
	float getCrossLineIgnoreDistance();
	void clearCrossLineIgnoreDistance();
	float getStableDistance();
	void clearStableDistance();
};

#endif /* INC_ENCODER_HPP_ */
