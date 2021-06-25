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
//#include "ICM_20648.h"

LineSensor line_sensor;
SideSensor side_sensor;
JoyStick joy_stick;
RotarySwitch rotary_switch;
Motor motor;
LED led;

Encoder encoder;
VelocityCtrl velocity_ctrl(&motor, &encoder);
LineTrace line_trace(&motor, &line_sensor);

float velocity;

void cppInit(void)
{
	line_sensor.ADCStart();
	motor.init();
	encoder.init();

	//line_sensor.calibration();

	line_trace.setGain(0.0005, 0.000003, 0);
	velocity_ctrl.setVelocityGain(1.5, 0, 20);
	velocity_ctrl.setOmegaGain(0, 0, 0);
}

void cppFlip1ms(void)
{
	line_sensor.updateSensorValues();
	encoder.updateCnt();


	velocity = velocity_ctrl.flip();
	//line_trace.flip();


	motor.motorCtrl();

	encoder.clearCnt();

	if(rotary_switch.getValue() == 1){
		//line_trace.start();
		//line_trace.setNormalRatio(0.1);
		velocity_ctrl.start();
		velocity_ctrl.setVelocity(0, 0);
		led.fullColor('R');
	}
	else{
		//line_trace.stop();
		//line_trace.setNormalRatio(0.0);
		velocity_ctrl.stop();
		led.fullColor('G');
	}
}

void cppFlip100ns(void)
{
	line_sensor.storeSensorValues();
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
	//velocity_ctrl.setVelocity(0., 1);

	//line_sensor.printSensorValues();

	//int16_t enc_l, enc_r;
	//encoder.getCnt(enc_l, enc_r);
	//printf("cnt: %d, %d\n", enc_l, enc_r);

	//line_sensor.updateSensorValues();
	//line_sensor.printSensorValues();

	//led.fullColor('C');
	led.LR(-1, 1);

	HAL_Delay(100);

	//motor.setRatio(0, -0.5);
	//velocity_ctrl.setOmegaGain(1, 1, 1);
	//led.fullColor('Y');
	led.LR(-1, 0);

	HAL_Delay(100);

}




