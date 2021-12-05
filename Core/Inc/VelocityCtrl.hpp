/*
 * VelocityCtrl.hpp
 *
 *  Created on: Jun 13, 2021
 *      Author: Haruki Shimotori
 */

#ifndef INC_VELOCITYCTRL_HPP_
#define INC_VELOCITYCTRL_HPP_

#include "Motor.hpp"
#include "Encoder.hpp"
#include "IMU.hpp"

#define WHEEL_RADIUS 11 //[mm]
#define PI 3.1415926535
#define ENCODER_RESOLUTION 4096
#define REDUCTION_RATIO 0.35 //Gear reduction ratio 0.35
#define VELOCITY_PER_CNT (2 * PI * WHEEL_RADIUS * REDUCTION_RATIO / ENCODER_RESOLUTION) //[m/s per cnt]
#define DELTA_T 0.001

class VelocityCtrl
{

private:
	float target_velocity_, target_omega_;
	float current_velocity_;
	float current_omega_;
	float v_kp_, v_kd_, v_ki_;
	float o_kp_, o_kd_, o_ki_;
	bool excution_flag_;
	bool i_reset_flag_;
	bool rotation_ratio_;
	Motor *motor_;
	Encoder *encoder_;
	IMU *imu_;

	float calcVelocity();
	//float calcOmega();
	void pid();
	void pidTranslationOnly();

public:
	VelocityCtrl(Motor *, Encoder *, IMU *);
	void init();
	void setVelocity(float, float);
	void setTranslationVelocityOnly(float, float);
	void setVelocityGain(float, float, float);
	void setOmegaGain(float, float, float);
	void flip();
	void start();
	void stop();
	float getCurrentVelocity();
	//float getCurrentOmega();

};

#endif /* INC_VELOCITYCTRL_HPP_ */
