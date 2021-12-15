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
float mon_pdis;
float mon_ave_l, mon_ave_r;
float mon_ref_dis, mon_current_dis;
uint16_t mon_vel_idx, mon_i;
float mon_tar_vel;

LineTrace::LineTrace(Motor *motor, LineSensor *line_sensor, VelocityCtrl *velocity_ctrl, SideSensor *side_sensor, Encoder *encoder, Odometry *odometry, Logger *logger, IMU *imu) :
				kp_(0), kd_(0), ki_(0), kp_velo_(0), kd_velo_(0), ki_velo_(0),
				excution_flag_(false), i_reset_flag_(false), normal_ratio_(0),
				target_velocity_(0), max_velocity_(0), max_velocity2_(0), min_velocity_(0), min_velocity2_(0), logging_flag_(false),
				ref_distance_(0), velocity_play_flag_(false), velocity_table_idx_(0), mode_selector_(0), crossline_idx_(0), sideline_idx_(0),
				ignore_crossline_flag_(false), stable_flag_(false), stable_cnt_reset_flag_(false), max_acc_(0), max_dec_(0), correction_check_cnt_(0)

{
	motor_ = motor;
	line_sensor_ = line_sensor;
	velocity_ctrl_ = velocity_ctrl;
	side_sensor_ = side_sensor;
	encoder_ = encoder;
	odometry_ = odometry;
	logger_ = logger;
	imu_ = imu;

	for(uint16_t i = 0; i < LOG_DATA_SIZE_DIS; i++){
		velocity_table_[i] = 0;
	}
	for(uint16_t i = 0; i < CROSSLINE_SIZE; i++){
		crossline_distance_[i] = 0;
	}
	for(uint16_t i = 0; i < SIDELINE_SIZE; i++){
		sideline_distance_[i] = 0;
	}
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

void LineTrace::loggerStart()
{
	encoder_->clearDistance10mm();
	odometry_->clearPotition();
	logger_->resetLogs2();

	logging_flag_ = true;
}

void LineTrace::loggerStop()
{
	logger_->stop();
	logging_flag_ = false;
}

bool LineTrace::isCrossLine()
{
	static uint16_t cnt = 0;
	float sensor_edge_val_l = (line_sensor_->sensor[0] + line_sensor_->sensor[1] + line_sensor_->sensor[2]) / 3;
	float sensor_edge_val_r = (line_sensor_->sensor[11] + line_sensor_->sensor[12] + line_sensor_->sensor[13]) / 3;
	static bool flag = false;
	static bool white_flag = false;
	mon_ave_l = sensor_edge_val_l;
	mon_ave_r = sensor_edge_val_r;

	if(white_flag == false){
		if(sensor_edge_val_l < 600 && sensor_edge_val_r < 600){
			cnt++;
		}
		else{
			cnt = 0;
		}

		if(cnt >= 3){
			flag = true;
			white_flag = true;
			cnt = 0;

			stable_cnt_reset_flag_ = true; //Because the conditions do not differ between when you tremble and when you do not tremble
			if(mode_selector_ == FIRST_RUNNING){
				storeCrossLineDistance();
			}
			else{
				correctionTotalDistanceFromCrossLine();
				correction_check_cnt_ = 0;
			}

			//led_.LR(-1, 1);
		}
	}
	else{
		if(sensor_edge_val_l > 500 && sensor_edge_val_r > 500){
			cnt++;
		}
		else{
			cnt = 0;
		}

		if(cnt >= 10){
			flag = false;
			white_flag = false;
			cnt = 0;

			//led_.LR(-1, 0);
		}

	}

	return flag;
}

float LineTrace::calcRadius(float distance, float theta)
{
	if(theta == 0) theta = 0.000001;
	return distance / theta;
}

float LineTrace::radius2Velocity(float radius)
{
	float velocity;

	if(mode_selector_ == SECOND_RUNNING){
		if(radius < 130) velocity = min_velocity_;
		else if(radius < 2000) velocity = 1.6;
		else velocity = max_velocity_;
	}
	else if(mode_selector_ == THIRD_RUNNING){
		if(radius < 130) velocity = min_velocity2_;
		else if(radius < 2000) velocity = 2;
		else velocity = max_velocity2_;
	}
	else velocity = 1.3;

	return velocity;
}

float LineTrace::radius2VelocityFnc(float radius)
{
	float a =       2.162;
	float b =   2.94e-05;
	float c =      -0.9206;
	float d =   -0.001755;

	return a * exp(b * radius) + c * exp(d * radius);
}

void LineTrace::createVelocityTabele()
{
	const float *p_distance, *p_theta;
	p_distance = logger_->getDistanceArrayPointer();
	p_theta= logger_->getThetaArrayPointer();

	float temp_distance, temp_theta;
	for(uint16_t i = 0; i < LOG_DATA_SIZE_DIS; i++){
		temp_distance = p_distance[i];
		temp_theta = p_theta[i];

		if(temp_theta == 0) temp_theta = 0.00001;
		float radius = abs(temp_distance / temp_theta);
		if(radius >= 5000) radius = 5000;

		//velocity_table_[i] = radius2Velocity(radius);
		velocity_table_[i] = radius2VelocityFnc(radius);

		ref_delta_distances_[i] = p_distance[i]; //copy
	}

	// ----- Decelerate processing -----//
	decelerateProcessing(max_dec_, p_distance);
	// ----- Accelerate processing -----//
	accelerateProcessing(max_acc_, p_distance);

	sd_write_array_float("COURSLOG", "VELTABLE.TXT", LOG_DATA_SIZE_DIS, velocity_table_, OVER_WRITE);

}

//float mon_crossdis;
void LineTrace::createVelocityTabeleFromSD()
{
	logger_->importDistanceAndTheta("COURSLOG", "DISTANCE.TXT", "THETA.TXT");
	sd_read_array_float("COURSLOG", "CROSSDIS.TXT", CROSSLINE_SIZE, crossline_distance_);
	sd_read_array_float("COURSLOG", "SIDEDIS.TXT", SIDELINE_SIZE, sideline_distance_);

	const float *p_distance, *p_theta;
	p_distance = logger_->getDistanceArrayPointer();
	p_theta= logger_->getThetaArrayPointer();

	float temp_distance, temp_theta;
	for(uint16_t i = 0; i < LOG_DATA_SIZE_DIS; i++){
		temp_distance = p_distance[i];
		temp_theta = p_theta[i];

		if(temp_theta == 0) temp_theta = 0.00001;
		float radius = abs(temp_distance / temp_theta);
		if(radius >= 5000) radius = 5000;

		//velocity_table_[i] = radius2Velocity(radius);
		velocity_table_[i] = radius2VelocityFnc(radius);

		ref_delta_distances_[i] = p_distance[i]; //copy
	}

	// ----- Decelerate processing -----//
	decelerateProcessing(max_dec_, p_distance);
	// ----- Accelerate processing -----//
	accelerateProcessing(max_acc_, p_distance);

	for(uint16_t i = 0; i < LOG_DATA_SIZE_DIS - 1; i++){
		velocity_table_[i] = ((R_VELTABLE)*(velocity_table_[i + 1]) + (1.0 - (R_VELTABLE))* (velocity_table_[i]));
	}

	sd_write_array_float("COURSLOG", "VELTABLE.TXT", LOG_DATA_SIZE_DIS, velocity_table_, OVER_WRITE);

}

void LineTrace::decelerateProcessing(const float am, const float *p_distance)
{
	for(uint16_t i = LOG_DATA_SIZE_DIS - 1; i >= 1; i--){
		float v_diff = velocity_table_[i-1] - velocity_table_[i];

		if(v_diff > 0){
			float t = p_distance[i]*1e-3 / v_diff;
			float a = v_diff / t;
			if(a > am){
				velocity_table_[i-1] = velocity_table_[i] + am * p_distance[i]*1e-3;
			}

		}
	}

}

void LineTrace::accelerateProcessing(const float am, const float *p_distance)
{
	for(uint16_t i = 0; i <= LOG_DATA_SIZE_DIS - 1; i++){
		float v_diff = velocity_table_[i+1] - velocity_table_[i];

		if(v_diff > 0){
			float t = p_distance[i]*1e-3 / v_diff;
			float a = v_diff / t;
			if(a > am){
				velocity_table_[i+1] = velocity_table_[i] + am * p_distance[i]*1e-3;
			}

		}
	}

}

void LineTrace::updateTargetVelocity()
{
	if(velocity_play_flag_ == true){
		//if(encoder_->getTotalDistance() >= ref_distance_){
			while(encoder_->getTotalDistance() >= ref_distance_){
				ref_distance_ += ref_delta_distances_[velocity_table_idx_];
				velocity_table_idx_++;
			}
		//}

		if(velocity_table_idx_ >= LOG_DATA_SIZE_DIS) velocity_table_idx_ = LOG_DATA_SIZE_DIS - 1;

		mon_ref_dis = ref_distance_;
		mon_current_dis = encoder_->getTotalDistance();
		mon_vel_idx = velocity_table_idx_;

		setTargetVelocity(velocity_table_[velocity_table_idx_]);

		mon_tar_vel = velocity_table_[velocity_table_idx_];

	}
}

bool LineTrace::isStable()
{
	bool ret = false;
	static uint16_t stable_cnt = 0;
	float temp_distance = encoder_->getDistance10mm();
	float temp_theta = odometry_->getTheta();;

	if(temp_theta == 0) temp_theta = 0.00001;
	float radius = abs(temp_distance / temp_theta);
	if(radius >= 5000) radius = 5000;

	if(stable_cnt_reset_flag_ == true){
		stable_cnt = 0;
		stable_cnt_reset_flag_ = false;
	}

	if(radius >= 2000){ //150
		stable_cnt++;
	}
	else{
		stable_cnt = 0;
	}

	if(stable_cnt >= 23){ //230mm
		ret = true;
	}

	return ret;
}

// -------public---------- //
void LineTrace::init()
{
	float temp_kp, temp_ki, temp_kd;
	sd_read_array_float("PARAMS", "KP.TXT", 1, &temp_kp);
	sd_read_array_float("PARAMS", "KI.TXT", 1, &temp_ki);
	sd_read_array_float("PARAMS", "KD.TXT", 1, &temp_kd);
	setGain(temp_kp, temp_ki, temp_kd);

	float temp_velocity, temp_max_velocity, temp_max_velocity2, temp_min_velocity, temp_min_velocity2;
	sd_read_array_float("PARAMS", "TARVEL1.TXT", 1, &temp_velocity);
	sd_read_array_float("PARAMS", "TARVEL2.TXT", 1, &temp_max_velocity);
	sd_read_array_float("PARAMS", "TARVEL3.TXT", 1, &temp_max_velocity2);
	sd_read_array_float("PARAMS", "MINVEL.TXT", 1, &temp_min_velocity);
	sd_read_array_float("PARAMS", "MINVEL2.TXT", 1, &temp_min_velocity2);
	setTargetVelocity(temp_velocity);
	setMaxVelocity(temp_max_velocity);
	setMaxVelocity2(temp_max_velocity2);
	setMinVelocity(temp_min_velocity);
	setMinVelocity2(temp_min_velocity2);

	float temp_acc, temp_dec;
	sd_read_array_float("PARAMS", "ACC.TXT", 1, &temp_acc);
	sd_read_array_float("PARAMS", "DEC.TXT", 1, &temp_dec);
	setMaxAccDec(temp_acc, temp_dec);
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

void LineTrace::setMaxVelocity(float velocity)
{
	max_velocity_ = velocity;
}

void LineTrace::setMaxVelocity2(float velocity)
{
	max_velocity2_ = velocity;
}

void LineTrace::setMinVelocity(float velocity)
{
	min_velocity_ = velocity;
}

void LineTrace::setMinVelocity2(float velocity)
{
	min_velocity2_ = velocity;
}
float LineTrace::getTargetVelocity()
{
	return target_velocity_;
}

float LineTrace::getMaxVelocity()
{
	return max_velocity_;
}

float LineTrace::getMaxVelocity2()
{
	return max_velocity2_;
}

float LineTrace::getMinVelocity()
{
	return min_velocity_;
}

float LineTrace::getMinVelocity2()
{
	return min_velocity2_;
}
void LineTrace::setMaxAccDec(const float acc, const float dec)
{
	max_acc_ = acc;
	max_dec_ = dec;
}

float LineTrace::getMaxAcc()
{
	return max_acc_;
}

float LineTrace::getMaxDec()
{
	return max_dec_;
}
void LineTrace::flip()
{
	if(excution_flag_ == true){
		// ---- line following processing -----//
		pidTrace();
		//pidAngularVelocityTrace();
		//steeringAngleTrace();


		if(isTargetDistance(10) == true){
			// ---- Store Logs ------//
			storeLogs();

			// -------- Detect Robot stabilization ------//
			if(isStable() == true && (~(side_sensor_->getStatus()) & 0x02) == 0x02){ // Stabilizing and side sensor is black
				stable_flag_ = true;
			}

			// ---reset total cnt ---//
			encoder_->clearDistance10mm();
			odometry_->clearPotition();
		}

		// ---- Target Velocity Updata ------//
		updateTargetVelocity();

		// ----- cross line ignore processing ------//
		if(isCrossLine() == true){ //detect cross line
			side_sensor_->enableIgnore();
			encoder_->clearCrossLineIgnoreDistance();
		}

		if(side_sensor_->getIgnoreFlag() == true && encoder_->getCrossLineIgnoreDistance() >= 200){
			side_sensor_->disableIgnore();

		}

		// ------- Store side line distance ------//
		if(stable_flag_ == true && (side_sensor_->getStatus() & 0x02) == 0x02){ //stabilizing and side sensor is white
			//storeSideLineDistance();
			if(mode_selector_ == FIRST_RUNNING){
				storeSideLineDistance();
			}
			else{
				correctionTotalDistanceFromSideMarker();
				correction_check_cnt_ = 0;
			}

			stable_flag_ = false;
			stable_cnt_reset_flag_ = true;
		}

		if(stable_flag_ == true) led_.LR(-1, 1);
		else led_.LR(-1, 0);

		// ----- emergency stop processing------//
		if(line_sensor_->emergencyStop() == true){
			velocity_ctrl_->setTranslationVelocityOnly(0, 0);
			//led_.LR(1, -1);
		}
		else{
			//led_.LR(0, -1);
		}

		correction_check_cnt_++;
		if(correction_check_cnt_ >= 10000) correction_check_cnt_ = 10000;

		if(correction_check_cnt_ <= 500) led_.LR(-1, 1);
		else led_.LR(-1, 0);
	}
}

void LineTrace::flip100ns()
{
	if(isTargetDistance(10) == true){
		// ---- Target Velocity Updata ------//
		updateTargetVelocity();

		// ---- Store Logs ------//
		storeLogs();

		// ---reset total cnt ---//
		encoder_->clearDistance10mm();
		odometry_->clearPotition();
	}
}

void LineTrace::start()
{
	excution_flag_ = true;
	i_reset_flag_ = true;
	velocity_ctrl_->start();
	side_sensor_->resetWhiteLineCnt();
	crossline_idx_ = 0;
	sideline_idx_ = 0;
}

void LineTrace::stop()
{
	excution_flag_ = false;
	velocity_ctrl_->stop();

	led_.LR(-1, 1);
	if(mode_selector_ == FIRST_RUNNING){ //First running
		logger_->saveDistanceAndTheta("COURSLOG", "DISTANCE.TXT", "THETA.TXT");
	}
	else{//Secondary run
		logger_->saveDistanceAndTheta2("COURSLOG", "DISTANC2.TXT", "THETA2.TXT");
	}
	sd_write_array_float("COURSLOG", "CROSSDIS.TXT", CROSSLINE_SIZE, crossline_distance_, OVER_WRITE);
	sd_write_array_float("COURSLOG", "SIDEDIS.TXT", SIDELINE_SIZE, sideline_distance_, OVER_WRITE);

	led_.LR(-1, 0);

	logger_->resetIdx();
	logger_->resetLogs2();
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
				/*
				if(mode_selector_ == FIRST_RUNNING){ // Other than first running
					loggerStart();
				}
				else{ // Other than first running
					startVelocityPlay();
				}
				*/
				loggerStart();
				if(mode_selector_ != FIRST_RUNNING){ // Other than first running
					startVelocityPlay();
				}

				encoder_->clearCrossLineIgnoreDistance();
				encoder_->clearTotalDistance();
				led_.LR(0, -1);
				stage = 10;
			}

			break;

		case 10:
			if(side_sensor_->getWhiteLineCntR() == 2){
				loggerStop();
				stopVelocityPlay();
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
	if(logging_flag_ == true){
		if(mode_selector_ == FIRST_RUNNING)
			logger_->storeDistanceAndTheta(encoder_->getDistance10mm(), odometry_->getTheta());
		else
			//logger_->storeDistanceAndTheta2(encoder_->getDistance10mm(), odometry_->getTheta());
			//logger_->storeDistanceAndTheta2(encoder_->getTotalDistance(), odometry_->getTheta());
			logger_->storeDistanceAndTheta2(velocity_ctrl_->getCurrentVelocity(), odometry_->getTheta());

		mon_store_cnt++;
	}
}

void LineTrace::startVelocityPlay()
{
	encoder_->clearTotalDistance();
	velocity_play_flag_ = true;
}

void LineTrace::stopVelocityPlay()
{
	velocity_play_flag_ = false;
	velocity_table_idx_ = 0;
	ref_distance_ = 0;
}

void LineTrace::setMode(int16_t mode)
{
	mode_selector_ = mode;
}

bool LineTrace::isTargetDistance(float target_distance)
{
	bool ret = false;
	if(encoder_->getDistance10mm() >= target_distance){
		ret = true;
	}

	return ret;
}

void LineTrace::storeCrossLineDistance()
{
	crossline_distance_[crossline_idx_] = encoder_->getTotalDistance();
	crossline_idx_++;

	if(crossline_idx_ >= CROSSLINE_SIZE) crossline_idx_ = CROSSLINE_SIZE - 1;
}

void LineTrace::storeSideLineDistance()
{
	sideline_distance_[sideline_idx_] = encoder_->getTotalDistance();
	sideline_idx_++;

	if(sideline_idx_ >= SIDELINE_SIZE) sideline_idx_ = SIDELINE_SIZE - 1;
}
void LineTrace::correctionTotalDistanceFromCrossLine()
{
	encoder_->setTotalDistance(crossline_distance_[crossline_idx_]);
	crossline_idx_++;

	if(crossline_idx_ >= CROSSLINE_SIZE) crossline_idx_ = CROSSLINE_SIZE - 1;

}

void LineTrace::correctionTotalDistanceFromSideMarker()
{
	for(uint16_t i = 0; i < SIDELINE_SIZE; i++){
		float temp_sideline_distance = sideline_distance_[i];
		float diff = abs(temp_sideline_distance - encoder_->getTotalDistance());
		if(diff <= 80){
			encoder_->setTotalDistance(sideline_distance_[i]);
			break;
		}
	}

	if(sideline_idx_ >= SIDELINE_SIZE) sideline_idx_ = SIDELINE_SIZE - 1;

}
