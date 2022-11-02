/*
 * IMU.hpp
 *
 *  Created on: Jun 27, 2021
 *      Author: Haruki Shimotori
 */

#ifndef INC_IMU_HPP_
#define INC_IMU_HPP_

#include "LED.hpp"

#define STORE_NUM 5
#define R_IMU 0.01 //0.03 Lowpath filter constant. The smaller it is, the more effective/

class IMU{
private:
	int16_t xa_, ya_, za_, xg_, yg_, zg_;
	float omega_;
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
