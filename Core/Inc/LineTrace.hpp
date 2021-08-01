/*
 * LineTrace.hpp
 *
 *  Created on: 2021/06/18
 *      Author: under
 */

#ifndef INC_LINETRACE_HPP_
#define INC_LINETRACE_HPP_

#include "Motor.hpp"
#include "LineSensor.hpp"
#include "LED.hpp"
#include "VelocityCtrl.hpp"

#define DELTA_T 0.001
#define ANGLE_BETWEEN_SENSORS 0.17104 //[rad]
#define SENSOR_ANGLE 2.0525 //[rad]
#define SENSOR_NUM 13
#define CENTER_NUM 6
#define PI 3.1415926535
#define CENTER_OF_ROTATION_TO_CENTER_OF_SENSOR 0.060 //[m]
//extern float monitor_angle;

class LineTrace
{
private:
	Motor *motor_;
	LineSensor *line_sensor_;
	VelocityCtrl *velocity_ctrl_;
	LED led_;
	float kp_, kd_, ki_;
	bool excution_flag_;
	float normal_ratio_;
	float sensor_values_[SENSOR_NUM];
	float target_velocity_;

	float calcError();
	float calcAngle();
	void getSensorValues();
	void calcStandardAngle(float &, uint16_t &);
	void calcNormalizedSensorValue(const uint16_t, float &, float &);
	void calcDeltaTheta(const float, const float, float &);
	void pid();
	void steeringAngleTrace();

public:
	LineTrace(Motor *, LineSensor *, VelocityCtrl *);
	void init();
	void setGain(float, float, float);
	void setNormalRatio(float);
	void setTargetVelocity(float);
	void flip();
	void start();
	void stop();

};



#endif /* INC_LINETRACE_HPP_ */
