/*
 * Logger2.cpp
 *
 *  Created on: 2022/10/16
 *      Author: Haruki Shimotori
 */

#include "Logger2.hpp"

#include <stdio.h>
#include "string.h"

char mon_c[32];


Logger2::Logger2(sdCard *sd_card, uint16_t size) : sd_card_(sd_card), log_idx_(0), recording_flag_(false), max_log_size_(size)
{
	logs_ = new float[size];

	for(int i = 0; i < size; i++){
		logs_[i] = 0;
	}
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
	sd_card_->write(directory_name, file_name, max_log_size_, logs_);
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

const float *Logger2::getLogsPointer()
{
	return logs_;
}

void Logger2::importLatestLogs(const char *directory_name, const char *file_name)
{
	float serial_number;
	char hidden_file_name[32];
	sprintf(hidden_file_name, "%c%s", '.', file_name);
	sd_card_->read(directory_name, hidden_file_name, 1, &serial_number);

	char directory_name_with_null[32];
	sprintf(directory_name_with_null, "%s", directory_name);

	char file_name_with_null[32];
	sprintf(file_name_with_null, "%s", file_name);

	char file_path[32];
	sprintf(file_path, "%s%d%s", file_name_with_null, int(serial_number-1), ".txt\0");

	sd_card_->read(directory_name_with_null, file_path, max_log_size_, logs_);
}

uint16_t Logger2::getLogsSize()
{
	return max_log_size_;
}

float Logger2::getLogData(uint16_t idx)
{
	return logs_[idx];
}
