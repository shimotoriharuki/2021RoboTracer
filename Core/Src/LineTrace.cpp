/*
 * LineTrace.cpp
 *
 *  Created on: 2021/06/18
 *      Author: under
 */

#include "LineTrace.hpp"
#include <stdio.h>
#include "Macro.h"
#include <cmath>

#define R_DIFF 0.08

float monitor_std_angle;
float monitor_norm_l, monitor_norm_r;
float monitor_delta_theta;
float monitor_steering_angle;
float monitor_target_omega;
float monitor_r;

float mon_diff, mon_diff_lpf;

uint16_t mon_store_cnt;

LineTrace::LineTrace(Motor *motor, LineSensor *line_sensor, VelocityCtrl *velocity_ctrl, SideSensor *side_sensor, Encoder *encoder, Odometry *odometry, Logger *logger) :
				kp_(0), kd_(0), ki_(0), kp_velo_(0), kd_velo_(0), ki_velo_(0),
				excution_flag_(false), i_reset_flag_(false), normal_ratio_(0), target_velocity_(0), logging_flag_(false)
{
	motor_ = motor;
	line_sensor_ = line_sensor;
	velocity_ctrl_ = velocity_ctrl;
	side_sensor_ = side_sensor;
	encoder_ = encoder;
	odometry_ = odometry;
	logger_ = logger;
}

// --------private--------- //
float LineTrace::calcError()
{
	static float pre_diff;
	float diff = (line_sensor_->sensor[0] + line_sensor_->sensor[1] + line_sensor_->sensor[2] + line_sensor_->sensor[3] + line_sensor_->sensor[4] + line_sensor_->sensor[5] + line_sensor_->sensor[6])
			- (line_sensor_->sensor[7] + line_sensor_->sensor[8] + line_sensor_->sensor[9] + line_sensor_->sensor[10] + line_sensor_->sensor[11] + line_sensor_->sensor[12] + line_sensor_->sensor[13]);
	mon_diff = diff;

	diff = ((R_DIFF)*(diff) + (1.0 - (R_DIFF))* (pre_diff));
	mon_diff_lpf = diff;

	pre_diff = diff;

	return diff;

}

float LineTrace::calcAngle()
{
	getSensorValues();

	float standard_angle;
	uint16_t standard_index;
	calcStandardAngle(standard_angle, standard_index);

	float norm_l, norm_r;
	calcNormalizedSensorValue(standard_index, norm_l, norm_r);

	float delta_theta;
	calcDeltaTheta(norm_l, norm_r, delta_theta);

	float steering_angle = standard_angle + delta_theta;

	monitor_std_angle = standard_angle;

	monitor_norm_l = norm_l;
	monitor_norm_r = norm_r;

	monitor_delta_theta = delta_theta;

	monitor_steering_angle = steering_angle;

	return steering_angle;
}

void LineTrace::getSensorValues()
{
	sensor_values_[0] = line_sensor_->sensor[0];
	sensor_values_[1] = line_sensor_->sensor[1];
	sensor_values_[2] = line_sensor_->sensor[2];
	sensor_values_[3] = line_sensor_->sensor[3];
	sensor_values_[4] = line_sensor_->sensor[4];
	sensor_values_[5] = line_sensor_->sensor[5];
	sensor_values_[6] = (line_sensor_->sensor[6] + line_sensor_->sensor[7]) / 2;
	sensor_values_[7] = line_sensor_->sensor[8];
	sensor_values_[8] = line_sensor_->sensor[9];
	sensor_values_[9] = line_sensor_->sensor[10];
	sensor_values_[10] = line_sensor_->sensor[11];
	sensor_values_[11] = line_sensor_->sensor[12];
	sensor_values_[12] = line_sensor_->sensor[13];
}

void LineTrace::calcStandardAngle(float &angle, uint16_t &index)
{
	if(line_sensor_->emergencyStop() == false){ // If all the sensors weren't black
		float min = 1000;

		for(uint8_t i = 0; i < SENSOR_NUM; i++){
			if(min > sensor_values_[i])	{
				min = sensor_values_[i];
				index = i;
			}
		}

		if(index < 6){
			angle = -(CENTER_NUM - index) * ANGLE_BETWEEN_SENSORS;
		}
		else if(index > 6){
			angle = (index - CENTER_NUM)  * ANGLE_BETWEEN_SENSORS;
		}
		else{
			angle = 0;
		}
	}
	else{
		angle = 0;
		index = 6;
	}

}

void LineTrace::calcNormalizedSensorValue(const uint16_t index, float &left_value, float &right_value)
{
	left_value = sensor_values_[index - 1] / (sensor_values_[index - 1] + sensor_values_[index + 1]);
	right_value = sensor_values_[index + 1] / (sensor_values_[index - 1] + sensor_values_[index + 1]);
}

void LineTrace::calcDeltaTheta(const float norm_l, const float norm_r, float &delta_theta)
{
	float phi = atan2(norm_l - norm_r, 1.0);
	delta_theta = (phi * ANGLE_BETWEEN_SENSORS/2) / (PI / 4);
}

void LineTrace::pidTrace()
{
	float diff = calcError();
	static float pre_diff = 0;
	float p, d;
	static float i;

	if(i_reset_flag_ == true){
		i = 0;
		i_reset_flag_ = false;
	}

	p = kp_ * diff;
	d = kd_ * (diff - pre_diff) / DELTA_T;
	i += ki_ * diff * DELTA_T;

	float rotation_ratio = p + d + i;

	//motor_->setRatio(left_ratio, right_ratio);
	velocity_ctrl_->setTranslationVelocityOnly(target_velocity_, rotation_ratio);

	pre_diff = diff;

}

void LineTrace::pidAngularVelocityTrace()
{
	float diff = calcError();
	static float pre_diff = 0;
	float p, d;
	static float i;
	float target_omega = 0;

	if(i_reset_flag_ == true){
		i = 0;
		i_reset_flag_ = false;
	}

	p = kp_velo_ * diff;
	d = kd_velo_ * (diff - pre_diff) / DELTA_T;
	i += ki_velo_ * diff * DELTA_T;

	target_omega = p + d + i;

	velocity_ctrl_->setVelocity(target_velocity_, target_omega);

	pre_diff = diff;

}

void LineTrace::steeringAngleTrace()
{
	float steering_angle = calcAngle();
	float r = tan(steering_angle / CENTER_OF_ROTATION_TO_CENTER_OF_SENSOR);

	//float current_velocity = velocity_ctrl_->getCurrentVelocity();
	float current_velocity = 0.1;
	float target_omega = 0;

	if(r >= 10000) target_omega = 0;
	else if(r != 0) target_omega = current_velocity / r; //Division by zero prevention

	velocity_ctrl_->setVelocity(target_velocity_, target_omega);

	//velocity_ctrl_->setVelocity(target_velocity_, 0);

	monitor_target_omega = target_omega;
	monitor_r = r;
}

// -------public---------- //
void LineTrace::init()
{
	float temp_kp, temp_ki, temp_kd;
	sd_read_array_float("PARAMS", "KP.TXT", 1, &temp_kp);
	sd_read_array_float("PARAMS", "KI.TXT", 1, &temp_ki);
	sd_read_array_float("PARAMS", "KD.TXT", 1, &temp_kd);
	setGain(temp_kp, temp_ki, temp_kd);

}

void LineTrace::setGain(float kp, float ki, float kd)
{
	kp_ = kp;
	ki_ = ki;
	kd_ = kd;
}


void LineTrace::setVeloGain(float kp, float ki, float kd)
{
	kp_velo_ = kp;
	ki_velo_ = ki;
	kd_velo_ = kd;
}

float LineTrace::getKp()
{
	return kp_;
}


float LineTrace::getKi()
{
	return ki_;
}

float LineTrace::getKd()
{
	return kd_;
}

float LineTrace::getKpV()
{
	return kp_velo_;
}
float LineTrace::getKiV()
{
	return ki_velo_;
}
float LineTrace::getKdV()
{
	return kd_velo_;
}

void LineTrace::setNormalRatio(float ratio)
{
	normal_ratio_ = ratio;
}

void LineTrace::setTargetVelocity(float velocity)
{
	target_velocity_ = velocity;
}

void LineTrace::flip()
{
	if(excution_flag_ == true){
		pidTrace();
		//pidAngularVelocityTrace();
		//steeringAngleTrace();

		if(line_sensor_->emergencyStop() == true){
			motor_->setRatio(0, 0);
			led_.LR(1, -1);
		}
		else{
			led_.LR(0, -1);

		}
	}
}

void LineTrace::start()
{
	excution_flag_ = true;
	i_reset_flag_ = true;
	velocity_ctrl_->start();
	side_sensor_->resetWhiteLineCnt();
}

void LineTrace::stop()
{
	excution_flag_ = false;
	velocity_ctrl_->stop();

	logger_->saveDistanceAndTheta("COURSLOG", "DISTANCE.TXT", "THETA.TXT");
	//logger_->resetLogs();
}

void LineTrace::running()
{
	uint16_t stage = 0;
	bool goal_flag = false;
	start();

	while(goal_flag == false){
		switch(stage){
		case 0:
			if(side_sensor_->getWhiteLineCntR() == 1){
				loggerStart();
				stage = 10;
			}

			break;

		case 10:
			if(side_sensor_->getWhiteLineCntR() == 2){
				loggerStop();
				HAL_Delay(100); //Run through after the goal

				setTargetVelocity(0);
				HAL_Delay(500); //Stop for a while on the spot

				goal_flag = true;

			}

			break;
		}
	}

	stop();

}

void LineTrace::storeLogs()
{
	if(logging_flag_ == true && encoder_->getTotalDistance() >= 10){
		logger_->storeDistanceAndTheta(encoder_->getTotalDistance(), odometry_->getTheta());
		encoder_->clearTotalCnt();
		odometry_->clearPotition();
		mon_store_cnt++;
	}
}

void LineTrace::loggerStart()
{
	encoder_->clearTotalCnt();
	odometry_->clearPotition();
	//logger_->start();

	logging_flag_ = true;
}

void LineTrace::loggerStop()
{
	logger_->stop();
	logging_flag_ = false;
}

