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
#include "SystemIdentification.hpp"

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
SystemIdentification sys_ident(&logger, &motor);

PathFollowing path_following;

double mon_f, mon_d;
float mon_v, mon_w;
uint16_t mon_cnt;
float mon_zg, mon_offset;

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

	lcd_clear();
	lcd_locate(0,0);
	lcd_printf("Voltage");
	lcd_locate(0,1);
	lcd_printf("%f", power_sensor.getButteryVoltage());
	HAL_Delay(1000);

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
	line_trace.init();

	line_sensor.calibration();
	HAL_Delay(1000);

	led.fullColor('M');
	imu.calibration();

	//line_trace.setGain(0.0005, 0.000003, 0);
	line_trace.setGain(0.0005, 0.000002, 0);

	//velocity_ctrl.setVelocityGain(1.5, 20, 0);
	velocity_ctrl.setVelocityGain(1.8295, 16.1174, 0.025243);
	//velocity_ctrl.setVelocityGain(1.9842, 22.9078, 0.02079);
	//velocity_ctrl.setOmegaGain(0.5, 5, 0);
	//velocity_ctrl.setOmegaGain(0.05, 7, 0);
	velocity_ctrl.setOmegaGain(0.069793, 0.86816, 0.0014027);
	//velocity_ctrl.setOmegaGain(0.12175, 1.0604, 0.002614);
	//velocity_ctrl.setOmegaGain(0.0, 0, 0);


	encoder.clearDistance();
	odometry.clearPotition();

	path_following.init();

}

void cppFlip1ms(void)
{
	line_sensor.updateSensorValues();
	imu.updateValues();
	mon_zg = imu.getOmega();
	mon_offset = imu.getOffsetVal();
	encoder.updateCnt();

	line_trace.flip();
	velocity_ctrl.flip();
	odometry.flip();

	motor.motorCtrl();

	logger.storeLog(velocity_ctrl.getCurrentVelocity());
	//logger.storeLog(imu.getOmega());

	static uint16_t twice_cnt;
	twice_cnt++;
	if(twice_cnt >= 2){ //2ms
		sys_ident.inOutputStore(imu.getOmega());
		twice_cnt = 0;
	}

	//mon_cnt = twice_cnt;
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
	/*
	static uint8_t cnt;
	cnt++;
	if(cnt >= 2){ //200ns
		cnt = 0;
		//imu.storeValues();
	}
	*/
}

void cppFlip10ms(void)
{
	static uint16_t twice_cnt;
	twice_cnt++;
	if(twice_cnt >= 7){ //70ms
		sys_ident.updateMsig();
		twice_cnt = 0;
	}

	mon_cnt = twice_cnt;

	/*
	path_following.setCurrentPath(odometry.getX(), odometry.getY(), odometry.getTheta());

	if(path_following.isTargetNear() == true){
		path_following.targetUpdate();
		path_following.flip();
	}
	velocity_ctrl.setVelocity(path_following.getV(), path_following.getW());
	*/

}

void cppExit(uint16_t gpio_pin)
{
	side_sensor.updateStatus(gpio_pin);
}

void cppLoop(void)
{
	switch(rotary_switch.getValue()){
	static int16_t selector;

	case 0:
		led.fullColor('R');

		lcd_clear();
		lcd_locate(0,0);
		lcd_printf("%4.2lf    ", line_trace.getKp()*1000);
		lcd_locate(0,1);
		lcd_printf("%4.2lf%4.2lf", line_trace.getKi()*100, line_trace.getKd()*1000);

		static float adj_kp = line_trace.getKp();
		static float adj_ki = line_trace.getKi();
		static float adj_kd = line_trace.getKd();

		if(joy_stick.getValue() == JOY_U){
			led.LR(-1, 1);
			HAL_Delay(300);

			selector++;
			if(selector >= 3) selector = 0;

			led.LR(-1, 0);
		}
		else if(joy_stick.getValue() == JOY_R){
			led.LR(-1, 1);
			HAL_Delay(100);

			if(selector == 0){
				adj_kp = adj_kp + 0.00001;
			}
			else if(selector == 1){
				adj_ki = adj_ki + 0.0001;
			}
			else{
				adj_kd = adj_kd + 0.00001;
			}

			led.fullColor('R');

			led.LR(-1, 0);
		}

		else if(joy_stick.getValue() == JOY_L){
			led.LR(-1, 1);
			HAL_Delay(100);

			if(selector == 0){
				adj_kp = adj_kp - 0.00001;
			}
			else if(selector == 1){
				adj_ki = adj_ki - 0.0001;
			}
			else{
				adj_kd = adj_kd - 0.00001;
			}

			led.fullColor('R');

			led.LR(-1, 0);
		}
		else if(joy_stick.getValue() == JOY_D){
			led.LR(-1, 1);
			HAL_Delay(300);

			float temp_kp, temp_ki, temp_kd;
			sd_read_array_float("PARAMS", "KP.TXT", 1, &temp_kp);
			sd_read_array_float("PARAMS", "KI.TXT", 1, &temp_ki);
			sd_read_array_float("PARAMS", "KD.TXT", 1, &temp_kd);
			line_trace.setGain(temp_kp, temp_ki, temp_kd);

			adj_kp = temp_kp;
			adj_ki = temp_kp;
			adj_kd = temp_kp;

			led.LR(-1, 0);
		}
		else if(joy_stick.getValue() == JOY_C){
			led.LR(-1, 1);
			HAL_Delay(300);

			sd_write_array_float("PARAMS", "KP.TXT", 1, &adj_kp, OVER_WRITE);
			sd_write_array_float("PARAMS", "KI.TXT", 1, &adj_ki, OVER_WRITE);
			sd_write_array_float("PARAMS", "KD.TXT", 1, &adj_kd, OVER_WRITE);
			line_trace.setGain(adj_kp, adj_ki, adj_kd);

			led.LR(-1, 0);
		}
		break;

	case 1:
		led.fullColor('G');

		lcd_clear();
		lcd_locate(0,0);
		lcd_printf("velocity");
		lcd_locate(0,1);
		lcd_printf("trace");

		if(joy_stick.getValue() == JOY_C){
			HAL_Delay(500);

			velocity_ctrl.start();
			line_trace.start();
			line_trace.setTargetVelocity(0.8);
			led.LR(1, -1);

			HAL_Delay(3000);

			velocity_ctrl.stop();
			line_trace.stop();
			led.LR(0, -1);

			//logger.stop();
		}

		break;

	case 2:
		led.fullColor('B');

		lcd_clear();
		lcd_locate(0,0);
		lcd_printf("Msig");
		lcd_locate(0,1);
		lcd_printf("Record");

		if(joy_stick.getValue() == JOY_C){
			led.LR(-1, 1);
			HAL_Delay(1500);

			sys_ident.setInputRatio(0.3);
			sys_ident.start();
			HAL_Delay(17500);
			sys_ident.stop();
			sys_ident.inOutputSave();

			led.LR(-1, 0);
		}
		break;

	case 3:
		led.fullColor('M');

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
			//logger.saveLogs("line_sensors", "sensor7.csv");
			led.LR(0, -1);

			led.LR(-1, 0);
		}

		break;

	case 4:
		led.fullColor('Y');

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
		led.fullColor('C');

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
		led.fullColor('R');

		lcd_clear();
		lcd_locate(0,0);
		lcd_printf("Position");
		lcd_locate(0,1);
		lcd_printf("Record");

		if(joy_stick.getValue() == JOY_C){
			HAL_Delay(500);
			led.LR(-1, 1);

			line_trace.setNormalRatio(0.07);
			line_trace.start();
			HAL_Delay(500);

			led.fullColor('R');
			encoder.clearTotalCnt();
			encoder.clearDistance();
			odometry.clearPotition();
			logger.start();

			HAL_Delay(3000);

			line_trace.stop();
			logger.stop();

			logger.saveDistanceAndTheta("Pos", "dis_s2.txt", "th_s2.txt");

			led.LR(-1, 0);
		}

		break;

	case 7:
		led.fullColor('G');

		lcd_clear();
		lcd_locate(0,0);
		lcd_printf("Velocity");
		lcd_locate(0,1);
		lcd_printf("Test");

		if(joy_stick.getValue() == JOY_C){
			led.LR(-1, 1);
			HAL_Delay(500);

			led.fullColor('R');
			velocity_ctrl.setVelocity(0, 1.57);
			velocity_ctrl.start();

			HAL_Delay(1000);
			velocity_ctrl.setVelocity(0, 0);
			HAL_Delay(100);
			velocity_ctrl.stop();

			led.LR(-1, 0);
		}
		break;

	case 8:
		led.fullColor('B');

		lcd_clear();
		lcd_locate(0,0);
		lcd_printf("%4.2lf    ", path_following.getKxVal()*1000);
		lcd_locate(0,1);
		lcd_printf("%4.2lf%4.2lf", path_following.getKyVal()*1000, path_following.getKtVal()*1000);

		static float adj_kx = path_following.getKxVal();
		static float adj_ky = path_following.getKyVal();
		static float adj_kt = path_following.getKtVal();

		if(joy_stick.getValue() == JOY_U){
			led.LR(-1, 1);
			HAL_Delay(300);

			selector++;
			if(selector >= 3) selector = 0;

			led.LR(-1, 0);
		}
		else if(joy_stick.getValue() == JOY_R){
			led.LR(-1, 1);
			HAL_Delay(100);

			if(selector == 0){
				adj_kx = adj_kx + 0.00001;
			}
			else if(selector == 1){
				adj_ky = adj_ky + 0.00001;
			}
			else{
				adj_kt = adj_kt + 0.00001;
			}

			led.fullColor('R');

			led.LR(-1, 0);
		}

		else if(joy_stick.getValue() == JOY_L){
			led.LR(-1, 1);
			HAL_Delay(100);

			if(selector == 0){
				adj_kx = adj_kx - 0.00001;
			}
			else if(selector == 1){
				adj_ky = adj_ky - 0.00001;
			}
			else{
				adj_kt = adj_kt - 0.00001;
			}

			led.fullColor('R');

			led.LR(-1, 0);
		}
		else if(joy_stick.getValue() == JOY_D){
			led.LR(-1, 1);
			HAL_Delay(300);

			float temp_kx, temp_ky, temp_kt;
			sd_read_array_float("PARAMS", "KX.TXT", 1, &temp_kx);
			sd_read_array_float("PARAMS", "KY.TXT", 1, &temp_ky);
			sd_read_array_float("PARAMS", "KT.TXT", 1, &temp_kt);
			path_following.setGain(temp_kx, temp_ky, temp_kt);

			adj_kx = temp_kx;
			adj_ky = temp_ky;
			adj_kt = temp_kt;

			led.LR(-1, 0);
		}
		else if(joy_stick.getValue() == JOY_C){
			led.LR(-1, 1);
			HAL_Delay(300);

			sd_write_array_float("PARAMS", "KX.TXT", 1, &adj_kx, OVER_WRITE);
			sd_write_array_float("PARAMS", "KY.TXT", 1, &adj_ky, OVER_WRITE);
			sd_write_array_float("PARAMS", "KT.TXT", 1, &adj_kt, OVER_WRITE);
			path_following.setGain(adj_kx, adj_ky, adj_kt);

			led.LR(-1, 0);
		}


		break;

	case 9:
		led.fullColor('M');

		lcd_clear();
		lcd_locate(0,0);
		lcd_printf("Step");
		lcd_locate(0,1);
		lcd_printf("Record");

		if(joy_stick.getValue() == JOY_C){
			HAL_Delay(1500);
			led.LR(-1, 1);

			logger.start();
			motor.setRatio(0.3, -0.3);

			HAL_Delay(1000);

			logger.stop();
			motor.setRatio(0.0, 0.0);

			logger.saveLogs("SYSIDENT", "STEPRES.txt");

			led.LR(-1, 0);
		}
		break;

	case 10:
		led.fullColor('Y');

		lcd_clear();
		lcd_locate(0,0);
		lcd_printf("PID");
		lcd_locate(0,1);
		lcd_printf("Response");

		if(joy_stick.getValue() == JOY_C){
			HAL_Delay(1500);
			led.LR(-1, 1);

			logger.start();
			velocity_ctrl.start();
			velocity_ctrl.setVelocity(1, 0);

			HAL_Delay(1000);

			logger.stop();
			velocity_ctrl.stop();

			logger.saveLogs("SYSIDENT", "PIDRES.txt");

			led.LR(-1, 0);
		}

		break;

	case 11:
		led.fullColor('C');

		lcd_clear();
		lcd_locate(0,0);
		lcd_printf("11      ");
		lcd_locate(0,1);
		lcd_printf("        ");

		break;

	case 12:
		led.fullColor('R');

		lcd_clear();
		lcd_locate(0,0);
		lcd_printf("12      ");
		lcd_locate(0,1);
		lcd_printf("        ");

		break;

	case 13:
		led.fullColor('G');

		lcd_clear();
		lcd_locate(0,0);
		lcd_printf("13      ");
		lcd_locate(0,1);
		lcd_printf("        ");

		break;

	case 14:
		led.fullColor('B');

		lcd_clear();
		lcd_locate(0,0);
		lcd_printf("14      ");
		lcd_locate(0,1);
		lcd_printf("        ");

		break;

	case 15:
		led.fullColor('M');

		lcd_clear();
		lcd_locate(0,0);
		lcd_printf("15      ");
		lcd_locate(0,1);
		lcd_printf("        ");

		break;

	default:
		break;

	}

	HAL_Delay(30);

}





