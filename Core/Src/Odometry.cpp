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
	//float current_velocity = velocity_ctrl_->getCurrentVelocity();
	double current_omega = velocity_ctrl_->getCurrentOmega();
	float distance = encoder_->getDistance();

	delta_theta_ = current_omega * DELTA_T;

	x_ = x_ + distance * cos(theta_ + delta_theta_ / 2);
	y_ = y_ + distance * sin(theta_ + delta_theta_ / 2);
	theta_ = theta_ + delta_theta_;


	monitor_x = x_;
	monitor_y = y_;
	monitor_theta = theta_;


}

void Odometry::flip()
{
	calcPotition();
}

double Odometry::getX()
{
	return x_;
}

double Odometry::getY()
{
	return y_;
}

double Odometry::getTheta()
{
	return theta_;
}

double Odometry::getDeltaTheta()
{
	return delta_theta_;
}

void Odometry::clearPotition()
{
	x_ = 0;
	y_ = 0;
	theta_ = 0;
}












