/*
 * Logger2.cpp
 *
 *  Created on: 2022/10/16
 *      Author: Haruki Shimotori
 */

#include "Logger2.hpp"

Logger2::Logger2(sdCard *sd_card, uint16_t size) : sd_card_(sd_card), log_idx_(0), recording_flag_(false), max_log_size_(size)
{
	logs_ = new float[size];
}

void Logger2::storeLogs(float data)
{
	if(recording_flag_ == true){
		logs_[log_idx_] = data;

		log_idx_++;

		if(log_idx_ >= max_log_size_) log_idx_ = 0;
	}

}
void Logger2::saveLogs(const char *directory_name, const char *file_name)
{
	float soiya[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
	sd_card_->write_(directory_name, file_name, max_log_size_, logs_, OVER_WRITE);
}
void Logger2::clearLogs()
{
	for(int i = 0; i < max_log_size_; i++){
		logs_[i] = 0;
	}

	log_idx_ = 0;

}

void Logger2::start()
{
	recording_flag_ = true;
	log_idx_ = 0;

}

void Logger2::stop()
{
	recording_flag_ = false;
}
