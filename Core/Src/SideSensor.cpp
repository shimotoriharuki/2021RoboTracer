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

SideSensor::SideSensor() : status_(0), status_L_(false), status_R_(false), white_line_cnt_l_(0), white_line_cnt_r_(0), ignore_flag_(false)
{

}

void SideSensor::updateStatus()
{
	static uint16_t cnt_l, cnt_r;

	if(ignore_flag_ == false){
		if(status_R_== false){
			if(!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_2)){ // Right is white
				cnt_r++;
			}
			else{
				cnt_r = 0;
			}
			if(cnt_r >= 2){
				//status_ |= 0x01;
				status_R_ = true;
				cnt_r = 0;
			}

		}
		else if(status_R_== true){
			if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_2)){ // Right is black
				cnt_r++;
			}
			else{
				cnt_r = 0;
			}
			if(cnt_r >= 2){
				//status_ ^= 0x01;
				status_R_ = false;

				white_line_cnt_r_++;
				//mon_cnt_r = white_line_cnt_r_;
			}
		}


		if(status_L_== false){
			if(!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_8)){ //Left is white
				cnt_l++;
			}
			else{
				cnt_l = 0;
			}
			if(cnt_l >= 2){
				//status_ |= 0x02;
				status_L_ = true;
				cnt_l = 0;
			}

		}
		else if(status_L_== true){
			if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_8)){ //Left is black
				cnt_l++;
			}
			else{
				cnt_l = 0;
			}
			if(cnt_l >= 2){
				//status_ ^= 0x02;
				status_L_ = false;

				white_line_cnt_l_++;
				//mon_cnt_l = white_line_cnt_l_;
			}

		}

		//mon_status = status_;
		mon_status_L = status_L_;
		mon_status_R = status_R_;
	}

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
