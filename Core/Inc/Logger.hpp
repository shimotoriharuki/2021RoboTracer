/*
 * Logger.hpp
 *
 *  Created on: 2021/06/29
 *      Author: under
 */

#ifndef INC_LOGGER_HPP_
#define INC_LOGGER_HPP_

#include "stm32f4xx_hal.h"
#include <vector>

#define LOG_DATA_SIZE_TIM 5000 //Time based size. Can record for 50 seconds every 10 ms
#define LOG_DATA_SIZE_DIS 6000 //Distance based size. Can record for 60 m every 10 cm

class Logger{

private:
	float store_data_float_[LOG_DATA_SIZE_TIM];
	uint16_t store_data_uint16_[LOG_DATA_SIZE_TIM];

	float store_distance_[LOG_DATA_SIZE_DIS];
	float store_theta_[LOG_DATA_SIZE_DIS];

	bool recording_flag_;
	//bool continuous_recording_flag_;

	uint16_t log_index_tim_;
	uint16_t log_index_dis_;

public:

	Logger();
	bool sdCardInit();

	void storeLogs(float *, uint8_t);
	void storeLogs(uint16_t *, uint8_t);
	void storeLog(float);
	void storeLog(uint16_t);
	void storeDistanceAndTheta(float , float);

	void saveLogs(const char *, const char *);
	void saveDistanceAndTheta(const char *, const char *, const char *);
	void resetLogs();

	/*
	void continuousWriteStart(const char *, const char *);
	void continuousDataWrite(float);
	void continuousDataWrite(uint16_t);
	void continuousWriteStop();
	*/

	void start();
	void stop();

};



#endif /* INC_LOGGER_HPP_ */