/*
 * Encoder.cpp
 *
 *  Created on: Jun 13, 2021
 *      Author: under
 */

#include "Encoder.hpp"
#include "G_variables.h"

#define MAX_ENCODER_CNT 65535

Encoder::Encoder() : cnt_l_(0), cnt_r_(0){}

void Encoder::init()
{
	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim8,TIM_CHANNEL_ALL);
}

void Encoder::updateCnt()
{
	cnt_l_ = TIM1 -> CNT;
	cnt_r_ = TIM8 -> CNT;
}

void Encoder::getCnt(uint16_t &cnt_l, uint16_t &cnt_r)
{
	cnt_l = cnt_l_;
	cnt_r = MAX_ENCODER_CNT - cnt_r_;
}

void::Encoder::clearCnt()
{
	cnt_l_ = 0;
	cnt_r_ = 0;
}
