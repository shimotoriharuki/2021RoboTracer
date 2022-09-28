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
#include "AQM0802.h"

uint16_t mon_idx1, mon_idx2;

Logger::Logger() : recording_flag_(false), log_index_tim_(0), log_index_tim2_(0), log_index_tim_int_(0), log_index_tim2_int_(0), log_index_dis_(0){}

bool Logger::sdCardInit()
{
	bool ret = false;

	if(sd_mount() == 1){
	  //printf("mount success\r\n");

	  lcd_clear();
	  lcd_locate(0,0);
	  lcd_printf("SD mount");
	  lcd_locate(0,1);
	  lcd_printf("success");
	  HAL_Delay(500);

	  ret = true;
	}
	else{
	  //printf("mount error\r\n");

	  lcd_clear();
	  lcd_locate(0,0);
	  lcd_printf("SD mount");
	  lcd_locate(0,1);
	  lcd_printf("fail");
	  HAL_Delay(1000);

	  ret = false;
	}

	//int	data[1];
	//int temp[1];

	//data[0] = 100;
	//sd_write_array_int("sdio", "write1.txt", DATA_SIZE, data, ADD_WRITE); //write
	//sd_read_array_int("sdio", "write1.txt", DATA_SIZE, temp); //read
	//sd_write_array_int("sdio", "write2.txt", DATA_SIZE, temp, ADD_WRITE); //write

	return ret;
}
void Logger::storeLog(float data)
{
	if(recording_flag_ == true){
		store_data_float_[log_index_tim_] = data;

		log_index_tim_++;
		mon_idx1 = log_index_tim_;

		if(log_index_tim_ >= LOG_DATA_SIZE_TIM) log_index_tim_ = 0;
	}
}

void Logger::storeLog2(float data)
{
	if(recording_flag_ == true){
		store_data_float2_[log_index_tim2_] = data;

		log_index_tim2_++;
		mon_idx2 = log_index_tim2_;

		if(log_index_tim2_ >= LOG_DATA_SIZE_TIM2) log_index_tim2_ = 0;
	}
}

void Logger::storeLogInt(int16_t data)
{
	if(recording_flag_ == true){
		store_data_int_[log_index_tim_int_] = data;

		log_index_tim_int_++;

		if(log_index_tim_int_ >= LOG_DATA_SIZE_TIM) log_index_tim_int_ = 0;
	}

}
void Logger::storeLog2Int(int16_t data)
{
	if(recording_flag_ == true){
		store_data_int2_[log_index_tim2_int_] = data;

		log_index_tim2_int_++;

		if(log_index_tim2_int_ >= LOG_DATA_SIZE_TIM2) log_index_tim2_int_ = 0;
	}

}

void Logger::storeDistanceAndTheta(float distance, float theta)
{
	//if(recording_flag_ == true){
		store_distance_[log_index_dis_] = distance;
		store_theta_[log_index_dis_] = theta;

		log_index_dis_++;

		if(log_index_dis_ >= LOG_DATA_SIZE_DIS) log_index_dis_ = 0;
	//}
}

void Logger::storeDistanceAndTheta2(float distance, float theta)
{
	//if(recording_flag_ == true){
		store_distance2_[log_index_dis_] = distance;
		store_theta2_[log_index_dis_] = theta;

		log_index_dis_++;

		if(log_index_dis_ >= LOG_DATA_SIZE_DIS) log_index_dis_ = 0;
	//}
}
const float *Logger::getDistanceArrayPointer()
{
	return store_distance_;
}

const float *Logger::getThetaArrayPointer()
{
	return store_theta_;
}

void Logger::saveLogs(const char *folder_name, const char *file_name)
{
	sd_write_array_float(folder_name, file_name, LOG_DATA_SIZE_TIM, store_data_float_, OVER_WRITE); //write
}
void Logger::saveLogs2(const char *folder_name, const char *file_name)
{
	sd_write_array_float(folder_name, file_name, LOG_DATA_SIZE_TIM2, store_data_float2_, OVER_WRITE); //write
}

void Logger::saveLogsInt(const char *folder_name, const char *file_name)
{
	sd_write_array_int(folder_name, file_name, LOG_DATA_SIZE_TIM, store_data_int_, OVER_WRITE); //write
}
void Logger::saveLogs2Int(const char *folder_name, const char *file_name)
{
	sd_write_array_int(folder_name, file_name, LOG_DATA_SIZE_TIM2, store_data_int2_, OVER_WRITE); //write
}
void Logger::saveDistanceAndTheta(const char *folder_name, const char *file_name1, const char *file_name2)
{
	sd_write_array_float(folder_name, file_name1, LOG_DATA_SIZE_DIS, store_distance_, OVER_WRITE); //write
	sd_write_array_float(folder_name, file_name2, LOG_DATA_SIZE_DIS, store_theta_, OVER_WRITE); //write
}

void Logger::saveDistanceAndTheta2(const char *folder_name, const char *file_name1, const char *file_name2)
{
	sd_write_array_float(folder_name, file_name1, LOG_DATA_SIZE_DIS, store_distance2_, OVER_WRITE); //write
	sd_write_array_float(folder_name, file_name2, LOG_DATA_SIZE_DIS, store_theta2_, OVER_WRITE); //write
}

void Logger::importDistanceAndTheta(const char *folder_name, const char *file_name1, const char *file_name2)
{
	sd_read_array_float(folder_name, file_name1, LOG_DATA_SIZE_DIS, store_distance_); //read
	sd_read_array_float(folder_name, file_name2, LOG_DATA_SIZE_DIS, store_theta_); //read
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

void Logger::resetLogs1()
{
	for(auto &log : store_distance_){
		log = 0;
	}
	for(auto &log : store_theta_){
		log = 0;
	}
	for(auto &log : store_data_float_){
		log = 0;
	}

	log_index_tim_ = 0;
	//log_index_tim2_ = 0;
	log_index_dis_ = 0;
}

void Logger::resetLogs2()
{
	for(auto &log : store_distance2_){
		log = 0;
	}
	for(auto &log : store_theta2_){
		log = 0;
	}
	for(auto &log : store_data_float2_){
		log = 0;
	}

	//log_index_tim_ = 0;
	log_index_tim2_ = 0;
	log_index_dis_ = 0;
}
void Logger::resetIdx()
{
	log_index_tim_ = 0;
	log_index_tim2_ = 0;
	log_index_dis_ = 0;
}

void Logger::start()
{
	log_index_tim_ = 0;
	log_index_tim2_ = 0;
	recording_flag_ = true;
}

void Logger::stop()
{
	recording_flag_ = false;
}

