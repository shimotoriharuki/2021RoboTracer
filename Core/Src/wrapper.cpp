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
#include "PowerSensor.hpp"
#include "IMU.hpp"
#include "AQM0802.h"
#include "Logger.hpp"

LineSensor line_sensor;
SideSensor side_sensor;
JoyStick joy_stick;
RotarySwitch rotary_switch;
Motor motor;
LED led;
PowerSensor power_sensor;
IMU imu;
Logger logger;

Encoder encoder;
VelocityCtrl velocity_ctrl(&motor, &encoder, &imu);
LineTrace line_trace(&motor, &line_sensor);

float velocity;

void cppInit(void)
{
	line_sensor.ADCStart();
	motor.init();
	encoder.init();
	//power_sensor.init();
	lcd_init();
	imu.init();

	line_sensor.calibration();
	imu.calibration();
	//printf("imu offset %f", imu.getOffsetVal());

	line_trace.setGain(0.0005, 0.000003, 0);

	//velocity_ctrl.setVelocityGain(1.5, 0, 20);
	//velocity_ctrl.setVelocityGain(0, 0, 0);
	//velocity_ctrl.setOmegaGain(0.15, 0, 20);

	logger.sdCardInit();
}

void cppFlip1ms(void)
{
	line_sensor.updateSensorValues();
	imu.updateValues();
	encoder.updateCnt();

	//velocity = velocity_ctrl.flip();
	line_trace.flip();

	motor.motorCtrl();

	encoder.clearCnt();


	/*
	if(rotary_switch.getValue() == 1){
		//line_trace.start();
		//line_trace.setNormalRatio(0.1);
		velocity_ctrl.start();
		velocity_ctrl.setVelocity(0, 0);
		led.LR(1, -1);
	}
	else{
		//line_trace.stop();
		//line_trace.setNormalRatio(0.0);
		velocity_ctrl.stop();
		led.LR(0, -1);
	}
	*/

	//Buttery Check
	//power_sensor.updateValues();
	//if(power_sensor.butteryCheck() == true) led.fullColor('R');

}

void cppFlip100ns(void)
{
	line_sensor.storeSensorValues();
}

void cppFlip10ms(void)
{
	logger.storeLog(line_sensor.sensor[7]);
}

void cppExit(uint16_t gpio_pin)
{
	side_sensor.updateStatus(gpio_pin);
}

void cppLoop(void)
{
	switch(rotary_switch.getValue()){

	case 0:
		lcd_clear();
		lcd_locate(0,0);
		lcd_printf("LCD");
		lcd_locate(0,1);
		lcd_printf("TEST0");
		break;

	case 1:
		lcd_clear();
		lcd_locate(0,0);
		lcd_printf("velocity");
		lcd_locate(0,1);
		lcd_printf("test");

		if(joy_stick.getValue() == JOY_C){
			HAL_Delay(500);

			logger.start();
			velocity_ctrl.start();
			velocity_ctrl.setVelocity(0.0, 3.14/2);
			led.LR(1, -1);

			HAL_Delay(2000);

			velocity_ctrl.stop();
			led.LR(0, -1);

			logger.stop();
		}

		break;
	case 2:
		lcd_clear();
		lcd_locate(0,0);
		lcd_printf("LOG");
		lcd_locate(0,1);
		lcd_printf("SAVE");

		if(joy_stick.getValue() == JOY_C){
			led.LR(1, -1);
			logger.saveLogs("line_sensors", "sensor6.csv");
			led.LR(0, -1);
		}
		break;
	case 3:
		led.fullColor('C');

		lcd_clear();
		lcd_locate(0,0);
		lcd_printf("Line");
		lcd_locate(0,1);
		lcd_printf("Trace");

		if(joy_stick.getValue() == JOY_C){
			led.LR(-1, 1);
			HAL_Delay(500);

			logger.start();
			line_trace.setNormalRatio(0.1);
			line_trace.start();

			HAL_Delay(5000);

			logger.stop();
			line_trace.setNormalRatio(0.1);
			line_trace.stop();

			led.LR(1, -1);
			logger.saveLogs("line_sensors", "sensor7.csv");
			led.LR(0, -1);

			led.LR(-1, 0);
		}

		break;
	case 4:
		led.fullColor('M');

		lcd_clear();
		lcd_locate(0,0);
		lcd_printf("Contiue");
		lcd_locate(0,1);
		lcd_printf("SaveTest");


		if(joy_stick.getValue() == JOY_C){
			led.LR(-1, 1);
			HAL_Delay(500);

			line_trace.setNormalRatio(0.1);
			line_trace.start();

			HAL_Delay(5000);

			line_trace.setNormalRatio(0.1);
			line_trace.stop();


			led.LR(-1, 0);
		}
		break;
	case 5:

		break;
	case 6:

		break;
	case 7:

		break;
	case 8:

		break;
	case 9:

		break;
	case 10:

		break;
	case 11:

		break;
	case 12:

		break;
	case 13:

		break;
	case 14:

		break;
	case 15:

		break;

	default:
		break;
	}
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

	//led.LR(-1, 1);

	//printf("imu zg: %f\n", imu.getOmega());

	//HAL_Delay(100);

	//motor.setRatio(0, -0.5);
	//velocity_ctrl.setOmegaGain(1, 1, 1);
	//led.fullColor('Y');
	//led.LR(-1, 0);

	HAL_Delay(10);

}




