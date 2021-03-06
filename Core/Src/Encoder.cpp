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
#define WHEEL_RADIUS 10.75 //[mm]
#define PULLEY_RADIUS 15 //[mm]
#define PI 3.1415926535
#define ENCODER_RESOLUTION 4096
#define ENCODER_RESOLUTION_CENTER 1200
#define REDUCTION_RATIO 0.35 //Gear reduction ratio
#define DISTANCE_PER_CNT (2 * PI * WHEEL_RADIUS * REDUCTION_RATIO / ENCODER_RESOLUTION) //[mm per cnt]
#define DISTANCE_PER_CNT_CENTER (2 * PI * PULLEY_RADIUS / ENCODER_RESOLUTION_CENTER) //[mm per cnt]
#define CORRECTION_COEFFICIENT float(1.0874883*1.0324*1.01)

float monitor_distance;
float monitor_cnt_l;
float monitor_cnt_l_lpf;
uint16_t monA, monB;
float mon_cnt_c, mon_distance_c;

Encoder::Encoder() : cnt_l_(0), cnt_r_(0), cnt_c_(0), distance_(0), total_cnt_l_(0), total_cnt_r_(0), distance_10mm_(0),
		total_distance_(0), cross_line_ignore_distance_(0), distance_center_(0){}

void Encoder::init()
{
	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim8,TIM_CHANNEL_ALL);
	TIM1 -> CNT = CNT_OFFSET;
	TIM8 -> CNT = CNT_OFFSET;
	monA = 0;
	monB = 0;
}

void Encoder::update()
{
	//static float pre_cnt_l, pre_cnt_r;
	float cnt_l = (float(CNT_OFFSET) - float(TIM1 -> CNT)) * CORRECTION_COEFFICIENT;
	float cnt_r = (float(TIM8 -> CNT) - float(CNT_OFFSET)) * CORRECTION_COEFFICIENT;
	monitor_cnt_l = cnt_l;

	//cnt_l_ = ((R_ENC)*(cnt_l) + (1.0 - (R_ENC))* (pre_cnt_l)); // lowpath filter
	//cnt_r_ = ((R_ENC)*(cnt_r) + (1.0 - (R_ENC))* (pre_cnt_r)); // lowpath filter

	cnt_l_ = cnt_l;
	cnt_r_ = cnt_r;
	monitor_cnt_l_lpf = cnt_l_;

	//pre_cnt_l = cnt_l_;
	//pre_cnt_r = cnt_r_;


	//total_cnt_l_ += cnt_l_;
	//total_cnt_r_ += cnt_r_;

	//distance_ = distance_ + DISTANCE_PER_CNT * (cnt_l_ + cnt_r_) / 2;
	distance_ = DISTANCE_PER_CNT * (cnt_l_ + cnt_r_) / 2;
	distance_10mm_ += distance_;
	total_distance_ += distance_;
	distance_center_ += DISTANCE_PER_CNT_CENTER * cnt_c_;
	cross_line_ignore_distance_ += distance_;
	monitor_distance = distance_10mm_;
}

void Encoder::extiCnt(uint16_t gpio_pin){
	static uint16_t flag_A = 0;
	static uint16_t flag_B = 0;

	if(gpio_pin == GPIO_PIN_12){ //A
		cnt_c_++;
		/*
		flag_A ^= 1;

		if((flag_A == 1 && flag_B == 0) || (flag_A == 0 && flag_B == 1)){
			cnt_c_++;
		}
		else if((flag_A == 1 && flag_B == 1) || (flag_A == 0 && flag_B == 0)){
			cnt_c_--;
		}

		*/
		monA ^= 1;
	}
	if(gpio_pin == GPIO_PIN_15){ //B
		/*
		flag_B ^= 1;

		if((flag_A == 1 && flag_B == 1) || (flag_A == 0 && flag_B == 0)){
			cnt_c_++;
		}
		else if((flag_A == 0 && flag_B == 1) || (flag_A == 1 && flag_B == 0)){
			cnt_c_--;
		}
		*/

		monB ^= 1;
	}

	mon_cnt_c = cnt_c_;
}

void Encoder::clear()
{
	cnt_l_ = 0;
	cnt_r_ = 0;
	cnt_c_ = 0;
	TIM1 -> CNT = CNT_OFFSET;
	TIM8 -> CNT = CNT_OFFSET;
	distance_ = 0;
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

float Encoder::getDistance10mm()
{
	return distance_10mm_;
}

float Encoder::getTotalDistance()
{
	return total_distance_;
}

float Encoder::getCenterDistance()
{
	return distance_center_;
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

void Encoder::clearCenterDistance()
{
	distance_center_ = 0;
}

float Encoder::getCrossLineIgnoreDistance()
{
	return cross_line_ignore_distance_;
}

void Encoder::clearCrossLineIgnoreDistance()
{
	cross_line_ignore_distance_ = 0;
}
