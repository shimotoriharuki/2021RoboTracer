/*
 * VelocityCtrl.cpp
 *
 *  Created on: Jun 13, 2021
 *      Author: under
 */

#include "VelocityCtrl.hpp"

VelocityCtrl::VelocityCtrl(Motor *motor, Encoder *encoder) :
target_velocity_(0), target_omega_(0), current_velocity_(0), current_omega_(0), v_kp_(0), v_kd_(0), v_ki_(0), o_kp_(0), o_kd_(0), o_ki_(0)
{
	motor_ = motor;
	encoder_ = encoder;

}

// ---------private ---------//

float VelocityCtrl::calcVelocity()
{
	int16_t enc_l, enc_r;
	encoder_->getCnt(enc_l, enc_r);
	float enc_cnt = (enc_l + enc_r) / 2;

	current_velocity_ = VELOCITY_PER_CNT * enc_cnt;

	return current_velocity_;

}


void VelocityCtrl::pid()
{
	float diff = target_velocity_ - current_velocity_;

	float p = v_kp_ * diff;

	motor_->setRatio(p, p);


}

// --------public -----------//
void VelocityCtrl::init()
{
	//motor_.init();
	//encoder_.init();

}

void VelocityCtrl::setVelocity(float velocity, float omega)
{
	target_velocity_ = velocity;
	target_omega_= omega;

}

void VelocityCtrl::setVelocityGain(float kp, float kd, float ki)
{
	v_kp_ = kp;
	v_kd_ = kd;
	v_ki_ = ki;
}

void VelocityCtrl::setOmegaGain(float kp, float kd, float ki)
{
	o_kp_ = kp;
	o_kd_ = kd;
	o_ki_ = ki;
}

float VelocityCtrl::flip()
{
	//encoder_->updateCnt();

	float velocity;
	velocity = calcVelocity();
	pid();

	//motor_->motorCtrl();

	//encoder_->clearCnt();

	return velocity;

}
