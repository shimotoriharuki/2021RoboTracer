/*
 * SideSensor.cpp
 *
 *  Created on: 2021/06/11
 *      Author: under
 */

#ifndef INC_SIDESENSOR_CPP_
#define INC_SIDESENSOR_CPP_

#include "stm32f4xx_hal.h"

class SideSensor{

private:

	uint16_t status_;
	uint16_t white_line_cnt_l_, white_line_cnt_r_;
	bool ignore_flag_;

public:

	SideSensor();
	void updateStatus(uint16_t);
	uint16_t getStatus();
	uint16_t getWhiteLineCntL();
	uint16_t getWhiteLineCntR();
	void resetWhiteLineCnt();
	void enableIgnore();
	void disableIgnore();
	bool getIgnoreFlag();

};


#endif /* INC_SIDESENSOR_CPP_ */
