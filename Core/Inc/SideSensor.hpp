/*
 * SideSensor.cpp
 *
 *  Created on: 2021/06/11
 *      Author: under
 */

#ifndef INC_SIDESENSOR_CPP_
#define INC_SIDESENSOR_CPP_

#include "stm32f4xx_hal.h"

#define LEFT_STATUS 0x01
#define RIGHT_STATUS 0x02

class SideSensor{

private:

	uint16_t status_;
	bool status_L_, status_R_;
	uint16_t white_line_cnt_l_, white_line_cnt_r_;
	bool ignore_flag_;

public:

	SideSensor();
	//void updateStatus(uint16_t);
	void updateStatus();
	uint16_t getStatus();
	bool getStatusL();
	bool getStatusR();
	uint16_t getWhiteLineCntL();
	uint16_t getWhiteLineCntR();
	void resetWhiteLineCnt();
	void enableIgnore();
	void disableIgnore();
	bool getIgnoreFlag();

};


#endif /* INC_SIDESENSOR_CPP_ */
