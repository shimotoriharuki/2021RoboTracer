/*
 * Odometry.cpp
 *
 *  Created on: 2021/08/01
 *      Author: under
 */

#include "Odometry.hpp"
#include <cmath>

#define DELTA_T 0.001

float moni_x, moni_y, moni_theta;

Odometry::Odometry(Encoder *encoder, IMU *imu, VelocityCtrl *velocity_ctrl) : x_robot_(0), y_robot_(0), theta_(0), x_sens_(0), y_sens_(0), delta_theta_(0)
{
	encoder_ = encoder;
	imu_ = imu;
	velocity_ctrl_ = velocity_ctrl;
}


void Odometry::calcPotition()
{
	//double current_omega = imu_->getOmega();
	//delta_theta_ = current_omega * DELTA_T;

	float distance_l, distance_r;
	encoder_->getLeftAndRightDistance(distance_l, distance_r);
	delta_theta_ = (distance_r - distance_l) * DELTA_T / TRED; //rad

	float distance = encoder_->getDistance();

	x_robot_ = x_robot_ + distance * cos(theta_ + delta_theta_ / 2); //calculate the rotation center position.
	y_robot_ = y_robot_ + distance * sin(theta_ + delta_theta_ / 2);
	theta_= theta_ + delta_theta_;

	moni_x = x_robot_;
	moni_y = y_robot_;
	moni_theta = theta_;

	x_sens_ = x_robot_ + SENSOR_LENGTH * cos(theta_); //calculate a sensor position from robot's center position
	y_sens_ = y_robot_ + SENSOR_LENGTH * sin(theta_);

}

void Odometry::flip()
{
	calcPotition();
}

double Odometry::getX()
{
	//return x_sens_;
	return x_robot_;
}

double Odometry::getY()
{
	//return y_sens_;
	return y_robot_;
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
	x_robot_ = 0;
	y_robot_ = 0;
	x_sens_ = 0;
	y_sens_ = 0;
	theta_ = 0;
}

