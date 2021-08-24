/*
 * Logger.cpp
 *
 *  Created on: 2021/06/29
 *      Author: under
 */

#include "Logger.hpp"
#include "HAL_SDcard_lib.h"
#include <stdio.h>
#include "Macro.h"

Logger::Logger() : recording_flag_(false), log_index_tim_(0), log_index_dis_(0){}

bool Logger::sdCardInit()
{
	bool ret = false;

	if(sd_mount() == 1){
	  printf("mount success\r\n");
	  ret = true;
	}
	else{
	  printf("mount error\r\n");
	  ret = false;
	}

	int	data[1];
	int temp[1];

	data[0] = 100;
	sd_write_array_int("sdio", "write1.txt", DATA_SIZE, data, ADD_WRITE); //write
	sd_read_array_int("sdio", "write1.txt", DATA_SIZE, temp); //read
	sd_write_array_int("sdio", "write2.txt", DATA_SIZE, temp, ADD_WRITE); //write

	return ret;
}
void Logger::storeLogs(float *data, uint8_t save_num)
{
	if(recording_flag_ == true){


	}

}
void Logger::storeLogs(uint16_t *data, uint8_t save_num)
{
	if(recording_flag_ == true){


	}

}
void Logger::storeLog(float data)
{
	if(recording_flag_ == true){
		store_data_float_[log_index_tim_] = data;

		log_index_tim_++;

		if(log_index_tim_ >= LOG_DATA_SIZE_TIM) log_index_tim_ = 0;
	}
}

void Logger::storeLog(uint16_t data)
{

}

void Logger::storeDistanceAndTheta(double distance, double theta)
{
	if(recording_flag_ == true){
		store_distance_[log_index_dis_] = distance;
		store_theta_[log_index_dis_] = theta;

		log_index_dis_++;

		if(log_index_dis_ >= LOG_DATA_SIZE_DIS) log_index_dis_ = 0;
	}
}

void Logger::saveLogs(const char *folder_name, const char *file_name)
{
	sd_write_array_float(folder_name, file_name, LOG_DATA_SIZE_TIM, store_data_float_, OVER_WRITE); //write
}

void Logger::saveDistanceAndTheta(const char *folder_name, const char *file_name1, const char *file_name2)
{
	sd_write_array_double(folder_name, file_name1, LOG_DATA_SIZE_DIS, store_distance_, OVER_WRITE); //write
	sd_write_array_double(folder_name, file_name2, LOG_DATA_SIZE_DIS, store_theta_, OVER_WRITE); //write
}

/*
void Logger::continuousWriteStart(const char *folder_name, const char *file_name)
{
	user_fopen(folder_name, file_name);
	continuous_recording_flag_ = true;

}

void Logger::continuousDataWrite(float data)
{
	if(continuous_recording_flag_ == true){
		sd_write(1, &data, ADD_WRITE);
	}
}

void Logger::continuousDataWrite(uint16_t data)
{

}

void Logger::continuousWriteStop()
{
	continuous_recording_flag_ = false;
	user_fclose();
}
*/

void Logger::resetLogs()
{
	for(auto &log : store_data_float_){
		log = 0;
	}
	for(auto &log : store_data_uint16_){
		log = 0;
	}
	for(auto &log : store_distance_){
		log = 0;
	}
	for(auto &log : store_theta_){
		log = 0;
	}

	log_index_tim_ = 0;
	log_index_dis_ = 0;
}

void Logger::start()
{
	recording_flag_ = true;
}

void Logger::stop()
{
	recording_flag_ = false;
}

