/*
 * LineTrace.cpp
 *
 *  Created on: 2021/06/18
 *      Author: under
 */

#include "LineTrace.hpp"
#include <stdio.h>

LineTrace::LineTrace(Motor *motor, LineSensor *line_sensor) : kp_(0), kd_(0), ki_(0){
	motor_ = motor;
	line_sensor_ = line_sensor;
}

// --------private--------- //
float LineTrace::calcError()
{
	float diff = line_sensor_->sensor[6] - line_sensor_->sensor[7];

	return diff;

}

void LineTrace::pid()
{
	float diff = calcError();
	static float pre_diff = 0;
	float p, d;
	static float i;

	p = kp_ * diff;
	d = kd_ * (pre_diff - diff) / DELTA_T;
	i += ki_ * diff * DELTA_T;

	float normal_ratio = 0.0;
	float left_ratio = normal_ratio + (p + d + i);
	float right_ratio = normal_ratio - (p + d + i);

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

void LineTrace::flip()
{
	//line_sensor_.updateSensorvaluses();
	pid();
	motor_->motorCtrl();

}



