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

class Logger{

private:
	std::vector<float> store_data_float_;
	std::vector<uint16_t> store_data_uint16_;

public:

	void sdCardInit();
	void storeLogs(float *, uint8_t);
	void storeLogs(uint16_t *, uint8_t);
	void storeLog(float);
	void storeLog(uint16_t);
	void saveLogs(const char *, const char *);
};



#endif /* INC_LOGGER_HPP_ */
