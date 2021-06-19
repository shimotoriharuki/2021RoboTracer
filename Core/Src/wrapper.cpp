/*
 * wrapper.cpp
 *
 *  Created on: Jun 9, 2021
 *      Author: Haruki Shimotori
 */
#include "wrapper.hpp"
#include <stdio.h>

#include "LineSensor.hpp"
#include "SideSensor.hpp"
#include "Joystick.hpp"
#include "RotarySwitch.hpp"
#include "Motor.hpp"
#include "LED.hpp"
#include "Encoder.hpp"
#include "VelocityCtrl.hpp"
#include "LineTrace.hpp"

LineSensor line_sensor;
SideSensor side_sensor;
JoyStick joy_stick;
RotarySwitch rotary_switch;
Motor motor;
LED led;

//Encoder encoder;
//VelocityCtrl velocity_ctrl;
LineTrace line_trace(&motor, &line_sensor);

float velocity;

void cppInit(void)
{
	line_sensor.ADCStart();
	line_sensor.calibration();
	motor.init();
	//encoder.init();
	//velocity_ctrl.init();
	//line_trace.init();
	line_trace.setGain(0.001, 0, 0);

	//bline_trace.calibration();

}

void cppFlip(void)
{
	line_sensor.updateSensorvaluses();
	//motor.motorCtrl();
	//encoder.updateCnt();
	//velocity = velocity_ctrl.flip();
	line_trace.flip();
}

void cppExit(uint16_t gpio_pin)
{
	side_sensor.updateStatus(gpio_pin);
}

void cppLoop(void)
{
	//printf("cpp loop test\n");
	//printf("cpp AD %d\n", line_sensor.sensor[0]);
	//printf("cpp side: %d\n", side_sensor.status());
	//printf("cpp joystick: %d\n", joy_stick.getValue());
	//printf("cpp rotaryswitch: %d\n", rotary_switch.getValue());
	//printf("cpp velocity: %f\n", velocity);

	//uint16_t cnt_l, cnt_r;
	//encoder.getCnt(cnt_l, cnt_r);
	//printf("cpp encode: %d, %d\n", cnt_l, cnt_r);

	//motor.setRatio(0, 1.0);
	//velocity_ctrl.setVelocityGain(1, 1, 1);
	//velocity_ctrl.setVelocity(0.1, 1);
	line_trace.printSensorValues();


	led.fullColor('C');
	led.LR(1, 1);

	HAL_Delay(1000);

	//motor.setRatio(0, -0.5);
	//velocity_ctrl.setOmegaGain(1, 1, 1);
	led.fullColor('Y');
	led.LR(-1, 0);

	HAL_Delay(1000);

}




