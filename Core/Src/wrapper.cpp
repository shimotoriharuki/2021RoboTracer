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
#include "Odometry.hpp"
#include "HAL_SDcard_lib.h"

#include "PathFollowing.hpp"

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
LineTrace line_trace(&motor, &line_sensor, &velocity_ctrl);
Odometry odometry(&encoder, &imu, &velocity_ctrl);

PathFollowing path_following;

double mon_f, mon_d;
double mon_v, mon_w;

bool flag = false;

void batteryLowMode()
{
	lcd_clear();
	lcd_locate(0,0);
	lcd_printf("Battery");
	lcd_locate(0,1);
	lcd_printf("Low");

	while(1){
		led.fullColor('R');
		HAL_Delay(100);
		led.fullColor('Y');
		HAL_Delay(100);

		if(joy_stick.getValue() == JOY_C){
			HAL_Delay(500);
			break;
		}
	}
}

void cppInit(void)
{
	lcd_init();

	//---------- Buttery Check ----------//
	power_sensor.init();
	HAL_Delay(100);
	power_sensor.updateValues();
	if(power_sensor.butteryCheck() == true) batteryLowMode(); //if battery low, informed

	// -----------initialize-------//
	if(logger.sdCardInit() == true){ //sd mount successfull
		led.fullColor('G');
		HAL_Delay(100);
	}
	else{ //sd mount fali
		led.fullColor('R');
		HAL_Delay(100);
	}

	line_sensor.ADCStart();
	motor.init();
	encoder.init();
	imu.init();

	//line_sensor.calibration();
	HAL_Delay(1000);

	led.fullColor('M');
	//imu.calibration();

	//line_trace.setGain(0.0005, 0.000003, 0);
	line_trace.setGain(0.0005, 0.000002, 0);

	//velocity_ctrl.setVelocityGain(1.5, 0, 20);
	velocity_ctrl.setVelocityGain(0, 0, 0);
	//velocity_ctrl.setOmegaGain(0.05, 0, 7);
	velocity_ctrl.setOmegaGain(0.0, 0, 0);


	encoder.clearDistance();
	odometry.clearPotition();

	path_following.init();

}

void cppFlip1ms(void)
{
	line_sensor.updateSensorValues();
	imu.updateValues();
	encoder.updateCnt();

	line_trace.flip();
	velocity_ctrl.flip();
	odometry.flip();

	motor.motorCtrl();

	/*
	if(encoder.getTotalDistance() >= 10){
		logger.storeDistanceAndTheta(encoder.getTotalDistance(), odometry.getTheta());
		encoder.clearTotalCnt();
		odometry.clearPotition();
	}
	*/


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

	//path_following.setGain(0.0, 0.0, 0.0);
	static double x, y, th;
	if(flag == true){
		x += 0.001;
		y += 0.00;
		th += 0.00;
	}
	//path_following.setTargetPathSingle(x, y, th);
	path_following.setCurrentPath(odometry.getX(), odometry.getY(), odometry.getTheta());
	path_following.targetUpdate();
	path_following.flip();

	path_following.getTargetVelocitys(mon_v, mon_w);

	velocity_ctrl.setVelocity(mon_v, mon_w);
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
			velocity_ctrl.setVelocity(0.0, 0.0);
			led.LR(1, -1);

			HAL_Delay(3000);

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
			led.LR(-1, 1);

			HAL_Delay(1000);
			float f = 0.123456789123456789123456789;
			double d = 0.123456789123456789123456789;
			mon_f = f;
			mon_d = d;
			sd_write_array_float("type test", "float.txt", 1, &f, OVER_WRITE);
			sd_write_array_double("type test", "double.txt", 1, &d, OVER_WRITE);

			led.LR(-1, 0);
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
		lcd_printf("Steering");
		lcd_locate(0,1);
		lcd_printf("Trace");

		if(joy_stick.getValue() == JOY_C){
			led.LR(-1, 1);
			HAL_Delay(500);

			line_trace.setTargetVelocity(0.1);
			velocity_ctrl.start();
			line_trace.start();

			HAL_Delay(10000);

			line_trace.stop();
			velocity_ctrl.stop();


			led.LR(-1, 0);
		}
		break;

	case 5:
		led.fullColor('Y');

		lcd_clear();
		lcd_locate(0,0);
		lcd_printf("Teoshi");
		lcd_locate(0,1);
		lcd_printf("Following");

		if(joy_stick.getValue() == JOY_C){
			led.LR(-1, 1);
			HAL_Delay(500);

			line_trace.setNormalRatio(0.0);
			line_trace.start();
			HAL_Delay(500);

			led.fullColor('R');
			encoder.clearTotalCnt();
			encoder.clearDistance();

			HAL_Delay(10000);

			line_trace.stop();
			//long total = encoder.getTotalCnt();

			//user_fopen("total_cnts", "cnts.txt");
			user_fopen("distance", "1m.txt");
			float d = encoder.getDistance();
			sd_write_float(1, &d, ADD_WRITE);
			user_fclose();

			led.LR(-1, 0);
		}

		break;

	case 6:
		led.fullColor('C');

		lcd_clear();
		lcd_locate(0,0);
		lcd_printf("Position");
		lcd_locate(0,1);
		lcd_printf("Record");

		if(joy_stick.getValue() == JOY_C){
			HAL_Delay(500);
			led.LR(-1, 1);

			line_trace.setNormalRatio(0.1);
			line_trace.start();
			HAL_Delay(500);

			led.fullColor('R');
			encoder.clearTotalCnt();
			encoder.clearDistance();
			odometry.clearPotition();
			logger.start();

			HAL_Delay(10000);

			line_trace.stop();
			logger.stop();

			logger.saveDistanceAndTheta("Position", "delta_distance.txt", "delta_theta.txt");

			led.LR(-1, 0);
		}

		break;

	case 7:
		led.fullColor('M');

		lcd_clear();
		lcd_locate(0,0);
		lcd_printf("Path");
		lcd_locate(0,1);
		lcd_printf("Following");

		if(joy_stick.getValue() == JOY_D){
			led.LR(-1, 1);
			path_following.setTargetPathMulti();
			led.LR(-1, 0);
		}

		else if(joy_stick.getValue() == JOY_C){
			led.LR(-1, 1);
			HAL_Delay(500);

			led.fullColor('R');
			encoder.clearTotalCnt();
			encoder.clearDistance();
			odometry.clearPotition();
			path_following.start();
			//velocity_ctrl.start();
			flag = true;

			HAL_Delay(10000);

			path_following.stop();
			velocity_ctrl.stop();
			flag = false;

			led.LR(-1, 0);
		}
		break;

	case 8:
		led.fullColor('M');

		lcd_clear();
		lcd_locate(0,0);
		lcd_printf("%3.1lf     ", path_following.getKxVal());
		lcd_locate(0,1);
		lcd_printf("%3.1lf,%3.1lf", path_following.getKyVal(), path_following.getKtVal());

		static double adj_kx, adj_ky, adj_kt;
		static int16_t pf_gain_selector;

		if(joy_stick.getValue() == JOY_U){
			led.LR(-1, 1);
			HAL_Delay(100);

			pf_gain_selector++;
			if(pf_gain_selector >= 3) pf_gain_selector = 0;

			led.LR(-1, 0);
		}
		else if(joy_stick.getValue() == JOY_R){
			led.LR(-1, 1);
			HAL_Delay(100);

			if(pf_gain_selector == 0){
				adj_kx++;
			}
			else if(pf_gain_selector == 1){
				adj_ky++;
			}
			else{
				adj_kt++;
			}

			led.fullColor('R');

			led.LR(-1, 0);
		}

		else if(joy_stick.getValue() == JOY_L){
			led.LR(-1, 1);
			HAL_Delay(100);

			if(pf_gain_selector == 0){
				adj_kx--;
			}
			else if(pf_gain_selector == 1){
				adj_ky--;
			}
			else{
				adj_kt--;
			}

			led.fullColor('R');

			led.LR(-1, 0);
		}
		else if(joy_stick.getValue() == JOY_D){
			led.LR(-1, 1);
			HAL_Delay(100);

			double temp_kx, temp_ky, temp_kt;
			sd_read_array_double("Params", "kx.txt", 1, &temp_kx);
			sd_read_array_double("Params", "ky.txt", 1, &temp_ky);
			sd_read_array_double("Params", "kt.txt", 1, &temp_kt);
			path_following.setGain(temp_kx, temp_ky, temp_kt);

			adj_kx = temp_kx;
			adj_ky = temp_ky;
			adj_kt = temp_kt;

			led.LR(-1, 0);
		}
		else if(joy_stick.getValue() == JOY_C){
			led.LR(-1, 1);
			HAL_Delay(100);

			sd_write_array_double("Params", "kx.txt", 1, &adj_kx, OVER_WRITE);
			sd_write_array_double("Params", "ky.txt", 1, &adj_ky, OVER_WRITE);
			sd_write_array_double("Params", "kt.txt", 1, &adj_kt, OVER_WRITE);
			path_following.setGain(adj_kx, adj_ky, adj_kt);

			led.LR(-1, 0);
		}


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




