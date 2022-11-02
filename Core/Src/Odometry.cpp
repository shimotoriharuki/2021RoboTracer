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
float mon_theta;

Odometry::Odometry(Encoder *encoder, IMU *imu, VelocityCtrl *velocity_ctrl) : x_robot_(0), y_robot_(0), theta_(0), x_sens_(0), y_sens_(0), delta_theta_(0)
{
	encoder_ = encoder;
	imu_ = imu;
	velocity_ctrl_ = velocity_ctrl;
}


void Odometry::calcPotition()
{
	double current_omega = imu_->getOmega();

	delta_theta_ = current_omega * DELTA_T;

	theta_= theta_ + delta_theta_;
	mon_theta = theta_;

}

void Odometry::flip()
{
	calcPotition();
}

double Odometry::getX()
{
	return x_sens_;
}

double Odometry::getY()
{
	return y_sens_;
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
	x_sens_ = 0;
	y_sens_ = 0;
	theta_ = 0;
}
