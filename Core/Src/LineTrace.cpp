/*
 * LineTrace.cpp
 *
 *  Created on: 2021/06/18
 *      Author: under
 */

#include "LineTrace.hpp"
#include <stdio.h>

LineTrace::LineTrace(Motor *motor, LineSensor *line_sensor) : kp_(0), kd_(0), ki_(0), excution_flag(false), normal_ratio_(0){
	motor_ = motor;
	line_sensor_ = line_sensor;
}

// --------private--------- //
float LineTrace::calcError()
{
	float diff = (line_sensor_->sensor[0] + line_sensor_->sensor[1] + line_sensor_->sensor[2] + line_sensor_->sensor[3] + line_sensor_->sensor[4] + line_sensor_->sensor[5] + line_sensor_->sensor[6])
			- (line_sensor_->sensor[7] + line_sensor_->sensor[8] + line_sensor_->sensor[9] + line_sensor_->sensor[10] + line_sensor_->sensor[11] + line_sensor_->sensor[12] + line_sensor_->sensor[13]);

	return diff;

}

void LineTrace::pid()
{
	float diff = calcError();
	static float pre_diff = 0;
	float p, d;
	static float i;

	p = kp_ * diff;
	d = kd_ * (diff - pre_diff) / DELTA_T;
	i += ki_ * diff * DELTA_T;

	float left_ratio = normal_ratio_ + (p + d + i);
	float right_ratio = normal_ratio_ - (p + d + i);

	motor_->setRatio(left_ratio, right_ratio);

	pre_diff = diff;


}

// -------public---------- //
void LineTrace::init()
{

}

void LineTrace::setGain(float kp, float kd, float ki)
{
	kp_ = kp;
	kd_ = kd;
	ki_ = ki;

}

void LineTrace::setNormalRatio(float ratio){
	normal_ratio_ = ratio;

}

void LineTrace::flip()
{
	if(excution_flag == true){
		pid();
	}
	if(line_sensor_->emergencyStop() == true){
		motor_->setRatio(0, 0);
		led_.LR(1, -1);
	}
	else{
		led_.LR(0, -1);

	}


}

void LineTrace::Start()
{
	excution_flag = true;
}

void LineTrace::Stop()
{
	excution_flag = false;
	motor_->setRatio(0, 0);
}

