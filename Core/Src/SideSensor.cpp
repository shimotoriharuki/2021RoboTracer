/*
 * SideSensor.cpp
 *
 *  Created on: 2021/06/11
 *      Author: under
 */

#include "SideSensor.hpp"

uint16_t mon_status;
uint16_t mon_cnt_l, mon_cnt_r;

SideSensor::SideSensor() : status_(0), white_line_cnt_l_(0), white_line_cnt_r_(0), ignore_flag_(false)
{

}

void SideSensor::updateStatus(uint16_t gpio_pin)
{
	static bool white_flag1 = false;
	static bool white_flag2 = false;
	static uint16_t cnt_l, cnt_r;

	if(ignore_flag_ == false){

		if(white_flag1 == false){
			if (gpio_pin == GPIO_PIN_2){
				cnt_r++;
			}
			else{
				cnt_r = 0;
			}
			if(cnt_r >= 5){
				status_ |= 0x01;
				white_flag1 = true;
				cnt_r = 0;
			}

		}
		else if(white_flag1 == true){
			if(gpio_pin == GPIO_PIN_2 && white_flag1 == true){
				cnt_r++;
			}
			else{
				cnt_r = 0;
			}
			if(cnt_r >= 5){
				status_ ^= 0x01;
				white_flag1 = false;

				white_line_cnt_r_++;
				mon_cnt_r = white_line_cnt_r_;
			}
		}


		if(white_flag2 == false){
			if(gpio_pin == GPIO_PIN_8){
				cnt_r++;
			}
			else{
				cnt_r = 0;
			}
			if(cnt_r >= 5){
				status_ |= 0x02;
				white_flag2 = true;
				cnt_r = 0;
			}

		}
		if(white_flag2 == true){
			if(gpio_pin == GPIO_PIN_8){
				cnt_r++;
			}
			else{
				cnt_r = 0;
			}
			if(cnt_r >= 5){
				status_ ^= 0x02;
				white_flag2 = false;

				white_line_cnt_l_++;
				mon_cnt_l = white_line_cnt_l_;
			}

		}

		mon_status = status_;
	}

}

uint16_t SideSensor::getStatus()
{
	return status_;
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
