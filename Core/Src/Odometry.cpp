/*
 * Odometry.cpp
 *
 *  Created on: 2021/08/01
 *      Author: under
 */

#include "Odometry.hpp"
#include <cmath>

#define DELTA_T 0.001

float monitor_x, monitor_y, monitor_theta;

Odometry::Odometry(Encoder *encoder, IMU *imu, VelocityCtrl *velocity_ctrl) : x_(0), y_(0), theta_(0)
{
	encoder_ = encoder;
	imu_ = imu;
	velocity_ctrl_ = velocity_ctrl;
}


void Odometry::calcPotition()
{
	float current_velocity = velocity_ctrl_->getCurrentVelocity();
	float current_omega = velocity_ctrl_->getCurrentOmega();

	float delta_theta = current_omega * DELTA_T;

	x_ = x_ + current_velocity * DELTA_T * cos(theta_ + delta_theta / 2);
	y_ = y_ + current_velocity * DELTA_T * sin(theta_ + delta_theta / 2);
	theta_ = theta_ + delta_theta;


	monitor_x = x_;
	monitor_y = y_;
	monitor_theta = theta_;

}

void Odometry::flip()
{
	calcPotition();
}

float Odometry::getX()
{
	return x_;
}

float Odometry::getY()
{
	return y_;
}

float Odometry::getTheta()
{
	return theta_;
}

void Odometry::resetPotition()
{
	x_ = 0;
	y_ = 0;
	theta_ = 0;
}












