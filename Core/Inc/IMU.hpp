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
#define R_IMU 0.03 //Lowpath filter constant. The smaller it is, the more effective/

class IMU{
private:
	//int16_t xa_store_[STORE_NUM], ya_store_[STORE_NUM], za_store_[STORE_NUM];
	//int16_t xg_store_[STORE_NUM], yg_store_[STORE_NUM], zg_store_[STORE_NUM];
	int16_t xa_, ya_, za_, xg_, yg_, zg_;
	//uint16_t array_idx;
	float offset_;
	LED led;

public:
	IMU();
	void init();
	//void storeValues();
	void updateValues();
	float getOmega();
	void calibration();
	float getOffsetVal();

};



#endif /* INC_IMU_HPP_ */
