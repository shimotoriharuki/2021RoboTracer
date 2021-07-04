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

#define LOG_DATA_SIZE 5000

class Logger{

private:
	float store_data_float_[LOG_DATA_SIZE];
	uint16_t store_data_uint16_[LOG_DATA_SIZE];

	bool recording_flag_;
	//bool continuous_recording_flag_;

	uint16_t log_index_;

public:

	Logger();
	void sdCardInit();

	void storeLogs(float *, uint8_t);
	void storeLogs(uint16_t *, uint8_t);
	void storeLog(float);
	void storeLog(uint16_t);

	void saveLogs(const char *, const char *);
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
