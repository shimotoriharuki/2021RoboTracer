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
	float cnt_l_, cnt_r_, cnt_c_;
	float distance_; //[mm]
	float total_cnt_l_, total_cnt_r_;
	float distance_10mm_;
	float total_distance_;
	float cross_line_ignore_distance_;
	float distance_center_;

public:
	Encoder();
	void init();
	void update();
	void extiCnt(uint16_t);
	void clear();
	void getCnt(float &, float &);
	float getDistance();
	float getDistance10mm();
	float getTotalDistance();
	float getCenterDistance();
	void setTotalDistance(float);

	void clearDistance();
	//float getTotalCnt();
	void clearDistance10mm();
	void clearTotalDistance();
	void clearCenterDistance();
	float getCrossLineIgnoreDistance();
	void clearCrossLineIgnoreDistance();
};

#endif /* INC_ENCODER_HPP_ */
