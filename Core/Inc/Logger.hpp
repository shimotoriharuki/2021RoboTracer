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

#define LOG_DATA_SIZE_TIM 3000 //Time based size. Can record for 50 seconds every 10 ms. max 5000
#define LOG_DATA_SIZE_TIM2 3000 //Time based size. Can record for 50 seconds every 10 ms
#define LOG_DATA_SIZE_DIS 6000 //Distance based size. Can record for 60 m every 10 mm

class Logger{

private:
	float store_data_float_[LOG_DATA_SIZE_TIM];
	float store_data_float2_[LOG_DATA_SIZE_TIM2];
	int16_t store_data_int_[LOG_DATA_SIZE_TIM];
	int16_t store_data_int2_[LOG_DATA_SIZE_TIM2];
	//uint16_t store_data_uint16_[LOG_DATA_SIZE_TIM];

	float  store_distance_[LOG_DATA_SIZE_DIS];
	float store_theta_[LOG_DATA_SIZE_DIS];
	float  store_distance2_[LOG_DATA_SIZE_DIS];
	float store_theta2_[LOG_DATA_SIZE_DIS];

	bool recording_flag_;
	//bool continuous_recording_flag_;

	uint16_t log_index_tim_;
	uint16_t log_index_tim2_;
	uint16_t log_index_tim_int_;
	uint16_t log_index_tim2_int_;
	uint16_t log_index_dis_;

public:

	Logger();
	bool sdCardInit();

	void storeLog(float);
	void storeLog2(float);
	void storeLogInt(int16_t);
	void storeLog2Int(int16_t);
	void storeDistanceAndTheta(float , float);
	void storeDistanceAndTheta2(float , float);
	const float *getDistanceArrayPointer();
	const float *getThetaArrayPointer();

	void saveLogs(const char *, const char *);
	void saveLogs2(const char *, const char *);
	void saveLogsInt(const char *, const char *);
	void saveLogs2Int(const char *, const char *);
	void saveDistanceAndTheta(const char *, const char *, const char *);
	void saveDistanceAndTheta2(const char *, const char *, const char *);
	void importDistanceAndTheta(const char *, const char *, const char *);
	void resetLogsDis1();
	void resetLogsDis2();
	void resetLogsTim1();
	void resetLogsTim2();
	void resetIdx();

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
