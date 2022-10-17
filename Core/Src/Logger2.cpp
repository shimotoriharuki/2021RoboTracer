/*
 * Logger2.cpp
 *
 *  Created on: 2022/10/16
 *      Author: Haruki Shimotori
 */

#include "Logger2.hpp"


Logger2::Logger2(sdCard *sd_card, uint16_t size) : log_idx_(0){
	sd_card_ = sd_card;
	buff_[size] = {0};
}

void Logger2::storeLogs(float data)
{

}
void Logger2::saveLogs()
{

}
void Logger2::clearLogs()
{

}



