/*
 * PowerSensor.hpp
 *
 *  Created on: Jun 27, 2021
 *      Author: Haruki Shimotori
 */

#ifndef INC_POWERSENSOR_HPP_
#define INC_POWERSENSOR_HPP_


class PowerSensor{

private:
	float current_l_, current_r_;
	float buttery_voltage_;

public:
	void init();
	void updateValues();
	void getCurrentValue(float &, float &);
	float getButteryVoltage();
	bool butteryCheck();

};




#endif /* INC_POWERSENSOR_HPP_ */
