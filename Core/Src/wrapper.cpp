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
Odometry odometry(&encoder, &imu, &velocity_ctrl);
LineTrace line_trace(&motor, &line_sensor, &velocity_ctrl, &side_sensor, &encoder, &odometry, &logger, &imu);
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

	//if(power_sensor.butteryCheck() == true) batteryLowMode(); //if battery low, informed

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
	//line_trace.setGain(0.0005, 0.000002, 0);

	//velocity_ctrl.setVelocityGain(1.5, 20, 0);
	velocity_ctrl.setVelocityGain(1.8295, 16.1174, 0.025243);
	//velocity_ctrl.setVelocityGain(1.9842, 22.9078, 0.02079);
	//velocity_ctrl.setOmegaGain(0.5, 5, 0);
	//velocity_ctrl.setOmegaGain(0.05, 7, 0);
	velocity_ctrl.setOmegaGain(0.069793, 0.86816, 0.0014027);
	//velocity_ctrl.setOmegaGain(0.12175, 1.0604, 0.002614);
	//velocity_ctrl.setOmegaGain(0.0, 0, 0);


	//encoder.clearDistance();
	odometry.clearPotition();

	path_following.init();

}

void cppFlip1ms(void)
{
	line_sensor.updateSensorValues();
	imu.updateValues();
	encoder.update();
	line_trace.flip();
	velocity_ctrl.flip();
	odometry.flip();
	side_sensor.updateStatus();

	motor.motorCtrl();

	//logger.storeLog(velocity_ctrl.getCurrentVelocity());
	//logger.storeLog(imu.getOmega());

	static uint16_t twice_cnt;
	twice_cnt++;
	if(twice_cnt >= 2){ //2ms
		sys_ident.inOutputStore(imu.getOmega());
		twice_cnt = 0;
	}

	//mon_cnt = twice_cnt;
	/*
	*/

	encoder.clear();

	//Buttery Check
	//power_sensor.updateValues();
	//if(power_sensor.butteryCheck() == true) led.fullColor('R');

}

void cppFlip100ns(void)
{
	line_sensor.storeSensorValues();
	//line_trace.flip100ns();
}

void cppFlip10ms(void)
{
	static uint16_t twice_cnt;
	twice_cnt++;
	if(twice_cnt >= 7){ //70ms
		sys_ident.updateMsig();
		twice_cnt = 0;
	}


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
	//side_sensor.updateStatus(gpio_pin);
}

void cppLoop(void)
{
	static int16_t selector;
	static int16_t selector_acc, selector_acc2;
	static int16_t selector_vel, selector_vel2;
	static int16_t selector_fast;

	static float adj_kp = line_trace.getKp();
	static float adj_ki= line_trace.getKi();
	static float adj_kd = line_trace.getKd();

	static float adj_kp_fast = line_trace.getKpFast();
	static float adj_ki_fast = line_trace.getKiFast();
	static float adj_kd_fast = line_trace.getKdFast();

	static float adj_velocity = line_trace.getTargetVelocity();
	static float adj_max_velocity = line_trace.getMaxVelocity();
	static float adj_max_velocity2 = line_trace.getMaxVelocity2();
	static float adj_min_velocity = line_trace.getMinVelocity();
	static float adj_min_velocity2 = line_trace.getMinVelocity2();

	static float adj_acc = line_trace.getMaxAcc();
	static float adj_dec = line_trace.getMaxDec();
	static float adj_acc2 = line_trace.getMaxAcc2();
	static float adj_dec2 = line_trace.getMaxDec2();

	switch(rotary_switch.getValue()){
	case 0:
		led.fullColor('W');

		lcd_clear();
		lcd_locate(0,0);
		lcd_printf("%4.2lf    ", line_trace.getKp()*1000);
		lcd_locate(0,1);
		lcd_printf("%4.2lf%4.2lf", line_trace.getKi()*100, line_trace.getKd()*10000);

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
				adj_kd = adj_kd + 0.000001;
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
				adj_kd = adj_kd - 0.000001;
			}

			led.fullColor('R');

			led.LR(-1, 0);
		}
		else if(joy_stick.getValue() == JOY_D){
			led.LR(-1, 1);
			HAL_Delay(300);

			/*
			float temp_kp, temp_ki, temp_kd;
			sd_read_array_float("PARAMS", "KP.TXT", 1, &temp_kp);
			sd_read_array_float("PARAMS", "KI.TXT", 1, &temp_ki);
			sd_read_array_float("PARAMS", "KD.TXT", 1, &temp_kd);
			line_trace.setGain(temp_kp, temp_ki, temp_kd);

			adj_kp = temp_kp;
			adj_ki = temp_kp;
			adj_kd = temp_kp;
			*/

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
		led.fullColor('C');

		lcd_clear();
		lcd_locate(0,0);
		lcd_printf("FirstRun");
		lcd_locate(0,1);
		lcd_printf("Start%3.1f", adj_velocity);

		if(joy_stick.getValue() == JOY_C){
			HAL_Delay(500);

			line_trace.setTargetVelocity(adj_velocity);
			led.LR(1, -1);

			line_trace.setMode(FIRST_RUNNING);
			line_trace.running();

			led.LR(0, -1);
		}

		break;

	case 2:
		led.fullColor('B');

		lcd_clear();
		lcd_locate(0,0);
		lcd_printf("FirstRun");
		lcd_locate(0,1);
		lcd_printf("%Vel: %3.1f", adj_velocity);


		if(joy_stick.getValue() == JOY_R){
			led.LR(-1, 1);
			HAL_Delay(100);

			adj_velocity = adj_velocity + 0.1;

			led.LR(-1, 0);
		}

		else if(joy_stick.getValue() == JOY_L){
			led.LR(-1, 1);
			HAL_Delay(100);

			adj_velocity = adj_velocity - 0.1;

			led.LR(-1, 0);
		}
		else if(joy_stick.getValue() == JOY_C){
			led.LR(-1, 1);
			HAL_Delay(300);

			sd_write_array_float("PARAMS", "TARVEL1.TXT", 1, &adj_velocity, OVER_WRITE);
			line_trace.setTargetVelocity(adj_velocity);

			led.LR(-1, 0);
		}
		break;

	case 3:
		led.fullColor('Y');

		lcd_clear();
		lcd_locate(0,0);
		lcd_printf("2:   %3.1f", adj_max_velocity);
		lcd_locate(0,1);
		lcd_printf("Start%3.1f", adj_min_velocity);

		if(joy_stick.getValue() == JOY_C){
			HAL_Delay(500);

			led.LR(1, -1);
			line_trace.setMode(SECOND_RUNNING);
			line_trace.setTargetVelocity(adj_min_velocity);
			line_trace.setMaxVelocity(adj_max_velocity);
			line_trace.setMinVelocity(adj_min_velocity);
			line_trace.createVelocityTabele();

			line_trace.running();

			led.LR(0, -1);
		}

		break;

	case 4:
		led.fullColor('G');

		lcd_clear();
		lcd_locate(0,0);
		lcd_printf("2:   %3.1f", line_trace.getMaxVelocity());
		lcd_locate(0,1);
		lcd_printf("%Vel: %3.1f", line_trace.getMinVelocity());

		if(joy_stick.getValue() == JOY_U){
			led.LR(-1, 1);
			HAL_Delay(300);

			selector_vel++;
			if(selector_vel >= 2) selector_vel = 0;

			led.LR(-1, 0);
		}
		else if(joy_stick.getValue() == JOY_R){
			led.LR(-1, 1);
			HAL_Delay(100);

			if(selector_vel == 0)
				adj_max_velocity = adj_max_velocity + 0.1;
			else
				adj_min_velocity = adj_min_velocity + 0.1;

			led.LR(-1, 0);
		}

		else if(joy_stick.getValue() == JOY_L){
			led.LR(-1, 1);
			HAL_Delay(100);

			if(selector_vel == 0)
				adj_max_velocity = adj_max_velocity - 0.1;
			else
				adj_min_velocity = adj_min_velocity - 0.1;

			led.LR(-1, 0);
		}
		else if(joy_stick.getValue() == JOY_C){
			led.LR(-1, 1);
			HAL_Delay(300);

			sd_write_array_float("PARAMS", "TARVEL2.TXT", 1, &adj_max_velocity, OVER_WRITE);
			sd_write_array_float("PARAMS", "MINVEL.TXT", 1, &adj_min_velocity, OVER_WRITE);
			line_trace.setMaxVelocity(adj_max_velocity);
			line_trace.setMinVelocity(adj_min_velocity);

			led.LR(-1, 0);
		}
		break;

	case 5:
		led.fullColor('M');

		lcd_clear();
		lcd_locate(0,0);
		lcd_printf("3:   %3.1f", adj_max_velocity2);
		lcd_locate(0,1);
		lcd_printf("Start%3.1f", adj_min_velocity2);

		if(joy_stick.getValue() == JOY_C){
			HAL_Delay(500);

			led.LR(1, -1);
			line_trace.setMode(THIRD_RUNNING);
			line_trace.setTargetVelocity(adj_min_velocity2);
			line_trace.setMaxVelocity2(adj_max_velocity2);
			line_trace.setMinVelocity2(adj_min_velocity2);
			line_trace.createVelocityTabele();

			line_trace.running();

			led.LR(0, -1);
		}

		break;

	case 6:
		led.fullColor('R');

		lcd_clear();
		lcd_locate(0,0);
		lcd_printf("3:   %3.1f", line_trace.getMaxVelocity2());
		lcd_locate(0,1);
		lcd_printf("%Vel: %3.1f", line_trace.getMinVelocity2());

		if(joy_stick.getValue() == JOY_U){
			led.LR(-1, 1);
			HAL_Delay(300);

			selector_vel2++;
			if(selector_vel2 >= 2) selector_vel2 = 0;

			led.LR(-1, 0);
		}
		else if(joy_stick.getValue() == JOY_R){
			led.LR(-1, 1);
			HAL_Delay(100);

			if(selector_vel2 == 0)
				adj_max_velocity2 = adj_max_velocity2 + 0.1;
			else
				adj_min_velocity2 = adj_min_velocity2 + 0.1;

			led.LR(-1, 0);
		}

		else if(joy_stick.getValue() == JOY_L){
			led.LR(-1, 1);
			HAL_Delay(100);

			if(selector_vel2 == 0)
				adj_max_velocity2 = adj_max_velocity2 - 0.1;
			else
				adj_min_velocity2 = adj_min_velocity2 - 0.1;

			led.LR(-1, 0);
		}
		else if(joy_stick.getValue() == JOY_C){
			led.LR(-1, 1);
			HAL_Delay(300);

			sd_write_array_float("PARAMS", "TARVEL3.TXT", 1, &adj_max_velocity2, OVER_WRITE);
			sd_write_array_float("PARAMS", "MINVEL2.TXT", 1, &adj_min_velocity2, OVER_WRITE);
			line_trace.setMaxVelocity2(adj_max_velocity2);
			line_trace.setMinVelocity2(adj_min_velocity2);

			led.LR(-1, 0);
		}

		break;

	case 7:
		led.fullColor('W');

		lcd_clear();
		lcd_locate(0,0);
		lcd_printf("ACC:%4.1f", line_trace.getMaxAcc());
		lcd_locate(0,1);
		lcd_printf("DEC:%4.1f", line_trace.getMaxDec());

		if(joy_stick.getValue() == JOY_U){
			led.LR(-1, 1);
			HAL_Delay(300);

			selector_acc++;
			if(selector_acc >= 2) selector_acc = 0;

			led.LR(-1, 0);
		}
		else if(joy_stick.getValue() == JOY_R){
			led.LR(-1, 1);
			HAL_Delay(100);

			if(selector_acc == 0){
				adj_acc = adj_acc + 0.1;
			}
			else{
				adj_dec = adj_dec + 0.1;
			}

			led.LR(-1, 0);
		}

		else if(joy_stick.getValue() == JOY_L){
			led.LR(-1, 1);
			HAL_Delay(100);

			if(selector_acc == 0){
				adj_acc = adj_acc - 0.1;
			}
			else{
				adj_dec = adj_dec - 0.1;
			}

			led.LR(-1, 0);
		}
		else if(joy_stick.getValue() == JOY_C){
			led.LR(-1, 1);
			HAL_Delay(300);

			sd_write_array_float("PARAMS", "ACC.TXT", 1, &adj_acc, OVER_WRITE);
			sd_write_array_float("PARAMS", "DEC.TXT", 1, &adj_dec, OVER_WRITE);
			line_trace.setMaxAccDec(adj_acc, adj_dec);

			led.LR(-1, 0);
		}
		break;

	case 8:
		led.fullColor('W');

		lcd_clear();
		lcd_locate(0,0);
		lcd_printf("ACC2:%3.1f", line_trace.getMaxAcc2());
		lcd_locate(0,1);
		lcd_printf("DEC2:%3.1f", line_trace.getMaxDec2());

		if(joy_stick.getValue() == JOY_U){
			led.LR(-1, 1);
			HAL_Delay(300);

			selector_acc2++;
			if(selector_acc2 >= 2) selector_acc2 = 0;

			led.LR(-1, 0);
		}
		else if(joy_stick.getValue() == JOY_R){
			led.LR(-1, 1);
			HAL_Delay(100);

			if(selector_acc2 == 0){
				adj_acc2 = adj_acc2 + 0.1;
			}
			else{
				adj_dec2 = adj_dec2 + 0.1;
			}

			led.LR(-1, 0);
		}

		else if(joy_stick.getValue() == JOY_L){
			led.LR(-1, 1);
			HAL_Delay(100);

			if(selector_acc2 == 0){
				adj_acc2 = adj_acc2 - 0.1;
			}
			else{
				adj_dec2 = adj_dec2 - 0.1;
			}

			led.LR(-1, 0);
		}
		else if(joy_stick.getValue() == JOY_C){
			led.LR(-1, 1);
			HAL_Delay(300);

			sd_write_array_float("PARAMS", "ACC2.TXT", 1, &adj_acc2, OVER_WRITE);
			sd_write_array_float("PARAMS", "DEC2.TXT", 1, &adj_dec2, OVER_WRITE);
			line_trace.setMaxAccDec2(adj_acc2, adj_dec2);

			led.LR(-1, 0);
		}
		break;

	case 9:
		led.fullColor('W');

		lcd_clear();
		lcd_locate(0,0);
		lcd_printf("F%4.2lf   ", line_trace.getKpFast()*1000);
		lcd_locate(0,1);
		lcd_printf("%4.2lf%4.2lf", line_trace.getKiFast()*100, line_trace.getKdFast()*10000);

		if(joy_stick.getValue() == JOY_U){
			led.LR(-1, 1);
			HAL_Delay(300);

			selector_fast++;
			if(selector_fast >= 3) selector_fast = 0;

			led.LR(-1, 0);
		}
		else if(joy_stick.getValue() == JOY_R){
			led.LR(-1, 1);
			HAL_Delay(100);

			if(selector_fast == 0){
				adj_kp_fast = adj_kp_fast + 0.00001;
			}
			else if(selector_fast == 1){
				adj_ki_fast = adj_ki_fast + 0.0001;
			}
			else{
				adj_kd_fast = adj_kd_fast + 0.000001;
			}

			led.fullColor('R');

			led.LR(-1, 0);
		}

		else if(joy_stick.getValue() == JOY_L){
			led.LR(-1, 1);
			HAL_Delay(100);

			if(selector_fast == 0){
				adj_kp_fast = adj_kp_fast - 0.00001;
			}
			else if(selector_fast == 1){
				adj_ki_fast = adj_ki_fast - 0.0001;
			}
			else{
				adj_kd_fast = adj_kd_fast - 0.000001;
			}

			led.fullColor('R');

			led.LR(-1, 0);
		}
		else if(joy_stick.getValue() == JOY_D){
			led.LR(-1, 1);
			HAL_Delay(300);

			/*
			float temp_kp, temp_ki, temp_kd;
			sd_read_array_float("PARAMS", "KP.TXT", 1, &temp_kp);
			sd_read_array_float("PARAMS", "KI.TXT", 1, &temp_ki);
			sd_read_array_float("PARAMS", "KD.TXT", 1, &temp_kd);
			line_trace.setGain(temp_kp, temp_ki, temp_kd);

			adj_kp = temp_kp;
			adj_ki = temp_kp;
			adj_kd = temp_kp;
			*/

			led.LR(-1, 0);
		}
		else if(joy_stick.getValue() == JOY_C){
			led.LR(-1, 1);
			HAL_Delay(300);

			sd_write_array_float("PARAMS", "KPFAST.TXT", 1, &adj_kp_fast, OVER_WRITE);
			sd_write_array_float("PARAMS", "KIFAST.TXT", 1, &adj_ki_fast, OVER_WRITE);
			sd_write_array_float("PARAMS", "KDFAST.TXT", 1, &adj_kd_fast, OVER_WRITE);
			line_trace.setGainFast(adj_kp_fast, adj_ki_fast, adj_kd_fast);

			led.LR(-1, 0);
		}
		/*
		led.fullColor('~');

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
			encoder.clearDistance10mm();
			//encoder.clearDistance();

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
		*/
		break;

	case 10:
		led.fullColor('~');

		lcd_clear();
		lcd_locate(0,0);
		lcd_printf("Create  ");
		lcd_locate(0,1);
		lcd_printf("VelTable");
		if(joy_stick.getValue() == JOY_C){
			HAL_Delay(500);
			led.LR(-1, 1);

			line_trace.setMode(THIRD_RUNNING);
			line_trace.setTargetVelocity(adj_max_velocity2);
			line_trace.setMaxVelocity(adj_max_velocity2);
			line_trace.createVelocityTabeleFromSD();

			led.LR(-1, 0);
		}

		break;

	case 11:
		led.fullColor('~');

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

	case 12:
		led.fullColor('~');

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

	case 13:
		led.fullColor('~');

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

	case 14:
		led.fullColor('W');

		lcd_clear();
		lcd_locate(0,0);
		lcd_printf("LogRun2    ");
		lcd_locate(0,1);
		lcd_printf("Start%3.1f", adj_max_velocity2);

		if(joy_stick.getValue() == JOY_C){
			HAL_Delay(500);

			led.LR(1, -1);
			line_trace.setMode(THIRD_RUNNING);
			line_trace.setTargetVelocity(adj_max_velocity2);
			line_trace.setMaxVelocity(adj_max_velocity2);
			line_trace.createVelocityTabeleFromSD();

			line_trace.running();

			led.LR(0, -1);
		}

		break;

	case 15:
		led.fullColor('W');

		lcd_clear();
		lcd_locate(0,0);
		lcd_printf("LogRun1    ");
		lcd_locate(0,1);
		lcd_printf("Start%3.1f", adj_max_velocity);

		if(joy_stick.getValue() == JOY_C){
			HAL_Delay(500);

			led.LR(1, -1);
			line_trace.setMode(SECOND_RUNNING);
			line_trace.setTargetVelocity(adj_max_velocity);
			line_trace.setMaxVelocity(adj_max_velocity);
			line_trace.createVelocityTabeleFromSD();

			line_trace.running();

			led.LR(0, -1);
		}
		break;

	default:
		break;

	}

	HAL_Delay(30);

}

void prameterSttingMode()
{

}





