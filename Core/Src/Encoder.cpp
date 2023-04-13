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
#define WHEEL_RADIUS 10.5 //[mm]
#define PI 3.1415926535
#define ENCODER_RESOLUTION 4096
#define REDUCTION_RATIO 0.35 //Gear reduction ratio
#define DISTANCE_PER_CNT (2 * PI * WHEEL_RADIUS * REDUCTION_RATIO / ENCODER_RESOLUTION) //[mm per cnt]
#define CORRECTION_COEFFICIENT float(1.0874883*1.0324*1.01)

float monitor_distance;
float monitor_cnt_l;
float monitor_cnt_l_lpf;

float mon_enc_l, mon_enc_r;

Encoder::Encoder() : cnt_l_(0), cnt_r_(0), distance_(0), total_cnt_l_(0), total_cnt_r_(0), distance_10mm_(0), total_distance_(0),
		side_line_ignore_distance_(), cross_line_ignore_distance_(0), goal_judge_distance_(0){}

void Encoder::init()
{
	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim8,TIM_CHANNEL_ALL);
	TIM1 -> CNT = CNT_OFFSET;
	TIM8 -> CNT = CNT_OFFSET;
}

void Encoder::update()
{
	float cnt_l = (float(CNT_OFFSET) - float(TIM1 -> CNT)) * CORRECTION_COEFFICIENT;
	float cnt_r = (float(TIM8 -> CNT) - float(CNT_OFFSET)) * CORRECTION_COEFFICIENT;
	//monitor_cnt_l = cnt_l;

	mon_enc_l = cnt_l_ = cnt_l;
	mon_enc_r = cnt_r_ = cnt_r;

	distance_ = DISTANCE_PER_CNT * (cnt_l_ + cnt_r_) / 2;
	distance_10mm_ += distance_;
	total_distance_ += distance_;
	side_line_ignore_distance_ += distance_;
	cross_line_ignore_distance_ += distance_;
	goal_judge_distance_ += distance_;
	//monitor_distance = distance_10mm_;
}

void Encoder::clear()
{
	cnt_l_ = 0;
	cnt_r_ = 0;
	TIM1 -> CNT = CNT_OFFSET;
	TIM8 -> CNT = CNT_OFFSET;
	distance_ = 0;
}

void Encoder::getCnt(float &cnt_l, float &cnt_r)
{
	cnt_l = cnt_l_;
	cnt_r = cnt_r_;
}

void Encoder::getLeftAndRightDistance(float &distance_l, float &distance_r)
{
	distance_l = DISTANCE_PER_CNT * cnt_l_;
	distance_r = DISTANCE_PER_CNT * cnt_r_;
}

float Encoder::getDistance()
{
	return distance_;
}

float Encoder::getDistance10mm()
{
	return distance_10mm_;
}

float Encoder::getTotalDistance()
{
	return total_distance_;
}

void Encoder::setTotalDistance(float true_distance){
	total_distance_ = true_distance;
}

/*
void Encoder::clearDistance()
{
	distance_ = 0;
}
*/

/*
float Encoder::getTotalCnt()
{
	return (total_cnt_l_ + total_cnt_r_) / 2;
}
*/

void Encoder::clearDistance10mm()
{
	//total_cnt_l_ = 0;
	//total_cnt_r_ = 0;
	distance_10mm_ = 0;
}

void Encoder::clearTotalDistance()
{
	total_distance_ = 0;
}

float Encoder::getSideLineIgnoreDistance()
{
	return side_line_ignore_distance_;
}

void Encoder::clearSideLineIgnoreDistance()
{
	side_line_ignore_distance_ = 0;
}
float Encoder::getCrossLineIgnoreDistance()
{
	return cross_line_ignore_distance_;
}

void Encoder::clearCrossLineIgnoreDistance()
{
	cross_line_ignore_distance_ = 0;
}

float Encoder::getGoalJudgeDistance()
{
	return goal_judge_distance_;
}

void Encoder::clearGoalJudgeDistance()
{
	goal_judge_distance_= 0;
}
