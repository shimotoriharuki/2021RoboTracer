/*
 * VelocityCtrl.cpp
 *
 *  Created on: Jun 13, 2021
 *      Author: Haruki Shimotori
 */

#include "VelocityCtrl.hpp"
//#include "ICM_20648.h"
#include <stdio.h>

float mon_current_velocity;

VelocityCtrl::VelocityCtrl(Motor *motor, Encoder *encoder, IMU *imu) :
target_velocity_(0), target_omega_(0), current_velocity_(0), current_omega_(0), v_kp_(0), v_kd_(0), v_ki_(0),
	o_kp_(0), o_kd_(0), o_ki_(0), excution_flag_(false), i_reset_flag_(false), rotation_ratio_(0)
{
	motor_ = motor;
	encoder_ = encoder;
	imu_ = imu;

}

// ---------private ---------//

float VelocityCtrl::calcVelocity()
{
	float enc_l, enc_r;
	encoder_->getCnt(enc_l, enc_r);
	float enc_cnt = (enc_l + enc_r) / 2;

	current_velocity_ = VELOCITY_PER_CNT * enc_cnt;
	mon_current_velocity = current_velocity_;

	return current_velocity_;
}

/*
float VelocityCtrl::calcOmega()
{
	float omega = imu_->getOmega();
	current_omega_ = -(omega / 16.4) * PI / 180;
	//printf("omegao: %f\n", current_omega_);

	return current_omega_;
}
*/

void VelocityCtrl::pid()
{
	float static v_pre_diff, o_pre_diff;
	float v_diff = target_velocity_ - current_velocity_;
	float o_diff = target_omega_- imu_->getOmega();

	float v_p, v_d, o_p, o_d;
	static float v_i, o_i;

	if(i_reset_flag_ == true){
		v_i = o_i = 0;
		i_reset_flag_ = false;
	}

	v_p = v_kp_ * v_diff;
	v_i += v_ki_ * v_diff * DELTA_T;
	v_d = v_kd_ * (v_diff - v_pre_diff) / DELTA_T;

	o_p = o_kp_ * o_diff;
	o_i += o_ki_ * o_diff * DELTA_T;
	o_d = o_kd_ * (o_diff - o_pre_diff) / DELTA_T;

	float v_left_ratio, v_right_ratio, o_left_ratio, o_right_ratio;

	v_left_ratio = v_right_ratio =  v_p + v_d + v_i;

	o_left_ratio = o_p + o_d + o_i;
	o_right_ratio = -(o_p + o_d + o_i);

	motor_->setRatio(v_left_ratio + o_left_ratio, v_right_ratio + o_right_ratio);

	v_pre_diff = v_diff;
	o_pre_diff = o_diff;
}

void VelocityCtrl::pidTranslationOnly()
{
	float static v_pre_diff;
	float v_diff = target_velocity_ - current_velocity_;

	float v_p, v_d ;
	static float v_i;

	if(i_reset_flag_ == true){
		v_i = 0;
		i_reset_flag_ = false;
	}

	v_p = v_kp_ * v_diff;
	v_i += v_ki_ * v_diff * DELTA_T;
	v_d = v_kd_ * (v_diff - v_pre_diff) / DELTA_T;

	float translation_ratio;

	translation_ratio =  v_p + v_d + v_i;

	if(translation_ratio >= 0.9) translation_ratio = 0.9;
	else if(translation_ratio <= -0.9) translation_ratio = -0.9;

	float exceeded = 0;
	if(translation_ratio + rotation_ratio_ >= 1.0){
		exceeded = (translation_ratio + rotation_ratio_) - 1.0;
	}

	translation_ratio -= exceeded;
	rotation_ratio_ += exceeded;

	motor_->setRatio(translation_ratio + rotation_ratio_, translation_ratio - rotation_ratio_);

	v_pre_diff = v_diff;
}

// --------public -----------//
void VelocityCtrl::init(){}

void VelocityCtrl::setVelocity(float velocity, float omega)
{
	target_velocity_ = velocity;
	target_omega_= omega;
}

void VelocityCtrl::setTranslationVelocityOnly(float velocity, float rotation_ratio)
{
	target_velocity_ = velocity;
	rotation_ratio_ = rotation_ratio;
}

void VelocityCtrl::setVelocityGain(float kp, float ki, float kd)
{
	v_kp_ = kp;
	v_ki_ = ki;
	v_kd_ = kd;
}

void VelocityCtrl::setOmegaGain(float kp, float ki, float kd)
{
	o_kp_ = kp;
	o_ki_ = ki;
	o_kd_ = kd;
}

void VelocityCtrl::flip()
{
    calcVelocity();
	//calcOmega();

	if(excution_flag_ == true){
		//pid();
		pidTranslationOnly();
	}


}

void VelocityCtrl::start()
{
	excution_flag_ = true;
	i_reset_flag_ = true;
	//calcOmega();
}

void VelocityCtrl::stop()
{
	excution_flag_ = false;
	motor_->setRatio(0, 0);

}

float VelocityCtrl::getCurrentVelocity()
{
	return current_velocity_;
}

/*
float VelocityCtrl::getCurrentOmega()
{
	return current_omega_;
}
*/
