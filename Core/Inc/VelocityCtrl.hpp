/*
 * VelocityCtrl.hpp
 *
 *  Created on: Jun 13, 2021
 *      Author: under
 */

#ifndef INC_VELOCITYCTRL_HPP_
#define INC_VELOCITYCTRL_HPP_

#include "Motor.hpp"
#include "Encoder.hpp"

#define WHEEL_RADIUS 11 //[mm]
#define PI 3.1415926535
#define ENCODER_RESOLUTION 512
#define REDUCTION_RATIO 0.35
#define VELOCITY_PER_CNT (2 * PI * WHEEL_RADIUS * REDUCTION_RATIO / ENCODER_RESOLUTION) //[m/s per cnt]

class VelocityCtrl
{

private:
	float target_velocity_, target_omega_;
	float current_velocity_, current_omega_;
	float v_kp_, v_kd_, v_ki_;
	float o_kp_, o_kd_, o_ki_;
	Motor motor_;
	Encoder encoder_;

	float calcVelocity();
	void pid();

public:
	VelocityCtrl();
	void init();
	void setVelocity(float, float);
	void setVelocityGain(float, float, float);
	void setOmegaGain(float, float, float);
	float flip();

};

#endif /* INC_VELOCITYCTRL_HPP_ */
