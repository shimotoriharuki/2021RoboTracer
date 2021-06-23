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

#define DELTA_T 0.001

class LineTrace
{
private:
	Motor *motor_;
	LineSensor *line_sensor_;
	LED led_;
	float kp_, kd_, ki_;
	bool excution_flag;
	float normal_ratio_;

	float calcError();
	void pid();

public:
	LineTrace(Motor *, LineSensor *);
	void init();
	void setGain(float, float, float);
	void setNormalRatio(float);
	void flip();
	void Start();
	void Stop();

};



#endif /* INC_LINETRACE_HPP_ */
