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
	char file_name_with_null[32] ={'\0'};
	sprintf(file_name_with_null, "%s", file_name);

	char file_path2[5] = {'1', '\0'};
	char file_path3[5] = {'.', 't', 'x', 't', '\0'};

	char file_path[32] = {'\0'};
	sprintf(file_path, "%s%s%s", file_name_with_null, file_path2, file_path3);
	strcpy(mon_c, file_path);

	sd_card_->write(directory_name, file_path, max_log_size_, logs_, OVER_WRITE);
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
