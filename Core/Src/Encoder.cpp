/*
 * Encoder.cpp
 *
 *  Created on: Jun 13, 2021
 *      Author: under
 */

#include "Encoder.hpp"
#include "G_variables.h"

#define MAX_ENCODER_CNT 65535
#define CNT_OFFSET 32768

Encoder::Encoder() : cnt_l_(CNT_OFFSET), cnt_r_(CNT_OFFSET){}

void Encoder::init()
{
	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim8,TIM_CHANNEL_ALL);
	TIM1 -> CNT = CNT_OFFSET;
	TIM8 -> CNT = CNT_OFFSET;
}

void Encoder::updateCnt()
{
	cnt_l_ = TIM1 -> CNT;
	cnt_r_ = TIM8 -> CNT;
}

void Encoder::getCnt(int16_t &cnt_l, int16_t &cnt_r)
{
	cnt_l = CNT_OFFSET - cnt_l_;
	cnt_r = cnt_r_ - CNT_OFFSET;
}

void Encoder::clearCnt()
{
	cnt_l_ = 0;
	cnt_r_ = 0;
	TIM1 -> CNT = CNT_OFFSET;
	TIM8 -> CNT = CNT_OFFSET;
}
