/*
 * SideSensor.cpp
 *
 *  Created on: 2021/06/11
 *      Author: under
 */

#include "SideSensor.hpp"

uint16_t mon_status;
bool mon_status_L, mon_status_R;
uint16_t mon_cnt_l, mon_cnt_r;
bool mon_rise_fall_flag = true;

SideSensor::SideSensor() : status_(0), status_L_(false), status_R_(false), white_line_cnt_l_(0), white_line_cnt_r_(0), ignore_flag_(false),
		timer_R_(0), timer_L_(0), rise_fall_flag_R_(true), rise_fall_flag_L_(true)
{

}

void SideSensor::updateStatusRightExti()
{
	if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2) == GPIO_PIN_SET){ // 立ち上がり Black
		timer_R_ = 0;
		rise_fall_flag_R_ = true;
	}
	else{	//White
		timer_R_ = 0;
		rise_fall_flag_R_ = false;
	}
}

void SideSensor::updateStatusLeftExti()
{
	if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8) == GPIO_PIN_SET){ // 立ち上がり Black
		timer_L_ = 0;
		rise_fall_flag_L_ = true;
	}
	else{	//White
		timer_L_ = 0;
		rise_fall_flag_L_ = false;
	}
}

void SideSensor::updateStatus() // called timer flip
{
	timer_R_++;
	timer_L_++;
	if(timer_R_ >= 10000) timer_R_ = 10000;
	if(timer_L_ >= 10000) timer_L_ = 10000;

	if(ignore_flag_ == false){
		if(rise_fall_flag_R_ == true && timer_R_ >= 2){
			status_R_ = false;
		}
		else if(rise_fall_flag_R_ == false && timer_R_ >= 2){
			status_R_ = true;
		}
	}

	if(rise_fall_flag_L_ == true && timer_L_ >= 2){
		status_L_ = false;
	}
	else if(rise_fall_flag_L_ == false && timer_L_ >= 2){
		status_L_ = true;
	}

	mon_status_R = status_R_;
	mon_status_L = status_L_;
}

uint16_t SideSensor::getStatus()
{
	return status_;
}

bool SideSensor::getStatusL()
{
	return status_L_;
}

bool SideSensor::getStatusR()
{
	return status_R_;
}

uint16_t SideSensor::getWhiteLineCntL()
{
	return white_line_cnt_l_;
}

uint16_t SideSensor::getWhiteLineCntR()
{
	return white_line_cnt_r_;
}

void SideSensor::resetWhiteLineCnt()
{
	white_line_cnt_l_ = 0;
	white_line_cnt_r_ = 0;
}

void SideSensor::enableIgnore()
{
	ignore_flag_ = true;
}

void SideSensor::disableIgnore()
{
	ignore_flag_ = false;
}

bool SideSensor::getIgnoreFlag()
{
	return ignore_flag_;
}

