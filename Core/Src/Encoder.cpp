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
#define WHEEL_RADIUS 11 //[mm]
#define PI 3.1415926535
#define ENCODER_RESOLUTION 4096
#define REDUCTION_RATIO 0.35 //Gear reduction ratio
#define DISTANCE_PER_CNT (2 * PI * WHEEL_RADIUS * REDUCTION_RATIO / ENCODER_RESOLUTION) //[m per cnt]

float monitor_distance;

Encoder::Encoder() : cnt_l_(CNT_OFFSET), cnt_r_(CNT_OFFSET), distance_(0){}

void Encoder::init()
{
	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim8,TIM_CHANNEL_ALL);
	TIM1 -> CNT = CNT_OFFSET;
	TIM8 -> CNT = CNT_OFFSET;
}

void Encoder::updateCnt()
{
	cnt_l_ = CNT_OFFSET - (TIM1 -> CNT);
	cnt_r_ = (TIM8 -> CNT) - CNT_OFFSET;

}

void Encoder::getCnt(int16_t &cnt_l, int16_t &cnt_r)
{
	cnt_l = cnt_l_;
	cnt_r = cnt_r_;
}

float Encoder::getDistance()
{
	distance_ = distance_ + DISTANCE_PER_CNT * (cnt_l_ + cnt_r_) / 2;
	monitor_distance = distance_;
	return distance_;

}

void Encoder::clearCnt()
{
	cnt_l_ = 0;
	cnt_r_ = 0;
	TIM1 -> CNT = CNT_OFFSET;
	TIM8 -> CNT = CNT_OFFSET;
}
