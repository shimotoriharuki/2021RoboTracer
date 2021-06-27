/*
 * IMU.hpp
 *
 *  Created on: Jun 27, 2021
 *      Author: under
 */

#ifndef INC_IMU_HPP_
#define INC_IMU_HPP_

#include "LED.hpp"

class IMU{
private:
	float xa_, ya_, za_;
	float xg_, yg_, zg_;
	float offset_;
	LED led;

public:
	IMU();
	void init();
	void updateValues();
	float getOmega();
	void calibration();
	float getOffsetVal();

};



#endif /* INC_IMU_HPP_ */
