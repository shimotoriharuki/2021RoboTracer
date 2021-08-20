/*
 * IMU.hpp
 *
 *  Created on: Jun 27, 2021
 *      Author: Haruki Shimotori
 */

#ifndef INC_IMU_HPP_
#define INC_IMU_HPP_

#include "LED.hpp"

class IMU{
private:
	int16_t xa_, ya_, za_;
	int16_t xg_, yg_, zg_;
	double offset_;
	LED led;

public:
	IMU();
	void init();
	void updateValues();
	double getOmega();
	void calibration();
	double getOffsetVal();

};



#endif /* INC_IMU_HPP_ */
