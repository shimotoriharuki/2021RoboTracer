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

	HAL_Delay(10);

}




