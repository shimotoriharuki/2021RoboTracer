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
#define CORRECTION_COEFFICIENT float(1.0874883*1.0324*1.01)

float monitor_distance;
float monitor_cnt_l;
float monitor_cnt_r;

Encoder::Encoder() : cnt_l_(0), cnt_r_(0), distance_(0), total_cnt_l_(0), total_cnt_r_(0){}

void Encoder::init()
{
	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim8,TIM_CHANNEL_ALL);
	TIM1 -> CNT = CNT_OFFSET;
	TIM8 -> CNT = CNT_OFFSET;
}

void Encoder::updateCnt()
{
	monitor_cnt_l = cnt_l_ = (float(CNT_OFFSET) - float(TIM1 -> CNT)) * CORRECTION_COEFFICIENT;
	monitor_cnt_r = cnt_r_ = (float(TIM8 -> CNT) - float(CNT_OFFSET)) * CORRECTION_COEFFICIENT;

	total_cnt_l_ += cnt_l_;
	total_cnt_r_ += cnt_r_;

	distance_ = distance_ + DISTANCE_PER_CNT * (cnt_l_ + cnt_r_) / 2;
	monitor_distance = distance_;
}

void Encoder::getCnt(float &cnt_l, float &cnt_r)
{
	cnt_l = cnt_l_;
	cnt_r = cnt_r_;
}

float Encoder::getDistance()
{
	return distance_;
}

void Encoder::clearDistance()
{
	distance_ = 0;
}

void Encoder::clearCnt()
{
	cnt_l_ = 0;
	cnt_r_ = 0;
	TIM1 -> CNT = CNT_OFFSET;
	TIM8 -> CNT = CNT_OFFSET;
}

long Encoder::getTotalCnt()
{
	return long((total_cnt_l_ + total_cnt_r_) / 2);
}

void Encoder::clearTotalCnt()
{
	total_cnt_l_ = 0;
	total_cnt_r_ = 0;
}
