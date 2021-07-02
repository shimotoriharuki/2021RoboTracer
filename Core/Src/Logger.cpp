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

void Logger::sdCardInit()
{
	if(sd_mount() == 1){
	  printf("mount success\r\n");
	}
	else{
	  printf("error\r\n");
	}

	int	data[1];
	int temp[1];

	data[0] = 100;
	sd_write_array_int("sdio", "write1.txt", DATA_SIZE, data, ADD_WRITE); //write
	sd_read_array_int("sdio", "write1.txt", DATA_SIZE, temp); //read
	sd_write_array_int("sdio", "write2.txt", DATA_SIZE, temp, ADD_WRITE); //write

	printf("sd write and read success!!\r\n");
	//sd_unmount();
}
void Logger::storeLogs(float *data, uint8_t save_num)
{

}
void Logger::storeLogs(uint16_t *data, uint8_t save_num)
{

}
void Logger::storeLog(float data)
{
	store_data_float_.push_back(data);
	if(store_data_float_.size() > 2000) store_data_float_.clear();
}

void Logger::storeLog(uint16_t data)
{

}

void Logger::saveLogs(const char *folder_name, const char *file_name){
	uint16_t data_size = store_data_float_.size();
	int data_array[data_size];

	uint16_t cnt = 0;
	for(const auto &s : store_data_float_){
		data_array[cnt] = s;
		cnt++;
	}
	sd_write_array_int(folder_name, file_name, data_size, data_array, OVER_WRITE); //write

}


