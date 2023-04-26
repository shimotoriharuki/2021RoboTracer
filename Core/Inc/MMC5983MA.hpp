/*
 * MMC5983MA.hpp
 *
 *  Created on: 2023/04/23
 *      Author: under
 */

#ifndef INC_MMC5983MA_HPP_
#define INC_MMC5983MA_HPP_

#include "main.h"

#define X_OUT0_ADDRESS 0x00
#define X_OUT1_ADDRESS 0x01
#define Y_OUT0_ADDRESS 0x02
#define Y_OUT1_ADDRESS 0x03
#define Z_OUT0_ADDRESS 0x04
#define Z_OUT1_ADDRESS 0x05
#define XYZ_OUT2_ADDRESS 0x06
#define T_OUT_ADDRESS 0x07
#define STATUS_ADDRESS 0x08
#define INTERNAL_CONTROL0_ADDRESS 0x09
#define INTERNAL_CONTROL1_ADDRESS 0x0A
#define INTERNAL_CONTROL2_ADDRESS 0x0B
#define INTERNAL_CONTROL3_ADDRESS 0x0C
#define PRODUCT_ID1_ADDRESS 0x2f

class MMC5983MA{
private:
	struct Offset{
		int32_t x;
		int32_t y;
		int32_t z;
	};
	struct Gauss{
		int32_t x;
		int32_t y;
		int32_t z;
	};

	Offset offset_;
	Gauss gauss_;
	bool enable_flag_;

	void send(uint8_t *, uint16_t);
	void receive(uint8_t *, uint16_t);


public:
	MMC5983MA();

	void write(const uint8_t, uint8_t *, uint16_t);
	void read(const uint8_t, uint8_t *, uint16_t);

	void measurementStartOnce();
	void measurementStartContinuous();
	void measurementStop();

	void calibration();
	void updateData();
	int32_t getGaussXData();
	int32_t getGaussYData();
	int32_t getGaussZData();

};


#endif /* INC_MMC5983MA_HPP_ */
