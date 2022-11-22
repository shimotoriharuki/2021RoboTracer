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

#define DISTANCE_TO_RECORD 10 //mm

float mon_steer_angle;

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

float my_theta;


LineTrace::LineTrace(Motor *motor, LineSensor *line_sensor, VelocityCtrl *velocity_ctrl, SideSensor *side_sensor, Encoder *encoder, Odometry *odometry, IMU *imu,
		DownForceUnit *down_force_unit, sdCard *sd_card) :
				kp_(0), kd_(0), ki_(0), kp_slow_(0), kd_slow_(0), ki_slow_(0),
				excution_flag_(false), i_reset_flag_(false), normal_ratio_(0),
				target_velocity_(0), target_omega_(0), max_velocity_(0), min_velocity_(0), max_velocity2_(0),  min_velocity2_(0), max_velocity3_(0),  min_velocity3_(0), max_velocity4_(0),  min_velocity4_(0),
				logging_flag_(false),
				ref_distance_(0), velocity_play_flag_(false), velocity_table_idx_(0), mode_selector_(0), crossline_idx_(0), crossline_idx2_(0), sideline_idx_(0), sideline_idx2_(0), all_sideline_idx_(0),
				ignore_crossline_flag_(false), stable_flag_(false), stable_flag_force_(false), stable_cnt_reset_flag_(false),
				max_acc_(0), max_dec_(0), max_acc2_(0), max_dec2_(0), max_acc3_(0), max_dec3_(0), max_acc4_(0), max_dec4_(0),
				correction_check_cnt_(0), store_check_cnt_(0), ignore_check_cnt_(0), all_sideline_flag_(false)

{
	motor_ = motor;
	line_sensor_ = line_sensor;
	velocity_ctrl_ = velocity_ctrl;
	side_sensor_ = side_sensor;
	encoder_ = encoder;
	odometry_ = odometry;
	imu_ = imu;
	down_force_unit_ = down_force_unit;
	sd_card_ = sd_card;

	//debugger_ = new Logger2(sd_card_, LOG_SIZE_TIM);
	//debugger2_ = new Logger2(sd_card_, LOG_SIZE_TIM);
	debugger3_ = new Logger2(sd_card_, LOG_SIZE_TIM);
	debugger4_ = new Logger2(sd_card_, LOG_SIZE_TIM);

	first_run_distance_logger_ = new Logger2(sd_card_, LOG_SIZE_DIS);
	first_run_theta_logger_ = new Logger2(sd_card_, LOG_SIZE_DIS);

	accdec_run_distance_logger_ = new Logger2(sd_card_, LOG_SIZE_DIS);
	accdec_run_theta_logger_ = new Logger2(sd_card_, LOG_SIZE_DIS);

	first_run_crossline_distance_logger_ = new Logger2(sd_card_, LOG_CROSSLINE_SIZE);
	first_run_sideline_distance_logger_ = new Logger2(sd_card_, LOG_SIDELINE_SIZE);

	accdec_run_crossline_distance_logger_ = new Logger2(sd_card_, LOG_CROSSLINE_SIZE);
	accdec_run_sideline_distance_logger_ = new Logger2(sd_card_, LOG_SIDELINE_SIZE);

	total_distance_logger_ = new Logger2(sd_card_, LOG_SIZE_DIS);

	for(uint16_t i = 0; i < LOG_SIZE_DIS; i++){
		velocity_table_[i] = 0;
	}
}

// --------private functions--------- //

// ---------------------------------------------------------------------------------------------------//
// -------------------------------------Sensor angle based line following --------------------------//
// ---------------------------------------------------------------------------------------------------//
float LineTrace::calcError()
{
	/*
	float diff = (line_sensor_->sensor[0] + line_sensor_->sensor[1] + line_sensor_->sensor[2] + line_sensor_->sensor[3] + line_sensor_->sensor[4] + line_sensor_->sensor[5] + line_sensor_->sensor[6])
			- (line_sensor_->sensor[7] + line_sensor_->sensor[8] + line_sensor_->sensor[9] + line_sensor_->sensor[10] + line_sensor_->sensor[11] + line_sensor_->sensor[12] + line_sensor_->sensor[13]);
	*/

	float diff = (line_sensor_->sensor[3] + line_sensor_->sensor[4] + line_sensor_->sensor[5])
			- (line_sensor_->sensor[8] + line_sensor_->sensor[9] + line_sensor_->sensor[10]);
	//mon_diff = diff;

	return diff;

}

float LineTrace::calcAngle()
{
	getSensorValues();

	float den = 0;
	float num = 0;
	float angle_list[SENSOR_NUM] = {-1.02, -0.85, -0.68, -0.51, -0.34, -0.17, 0, 0.17, 0.34, 0.51, 0.68, 0.85, 1.02};

	for(uint16_t i = 0; i < SENSOR_NUM; i++){


		num += angle_list[i] * sensor_digital_values_[i];
		den += sensor_digital_values_[i];

		/*
		num += angle_list[i] * sensor_values_[i];
		den += sensor_values_[i];
		*/
	}

	float angle = 0;
	if(den != 0) angle = num / den;
	else angle = 0;

	mon_steer_angle = angle;

	return angle;
}

void LineTrace::getSensorValues()
{
	sensor_values_[0] = 1000 - line_sensor_->sensor[0];
	sensor_values_[1] = 1000 - line_sensor_->sensor[1];
	sensor_values_[2] = 1000 - line_sensor_->sensor[2];
	sensor_values_[3] = 1000 - line_sensor_->sensor[3];
	sensor_values_[4] = 1000 - line_sensor_->sensor[4];
	sensor_values_[5] = 1000 - line_sensor_->sensor[5];
	sensor_values_[6] = 1000 - (line_sensor_->sensor[6] + line_sensor_->sensor[7]) / 2;
	sensor_values_[7] = 1000 - line_sensor_->sensor[8];
	sensor_values_[8] = 1000 - line_sensor_->sensor[9];
	sensor_values_[9] = 1000 - line_sensor_->sensor[10];
	sensor_values_[10] = 1000 - line_sensor_->sensor[11];
	sensor_values_[11] = 1000 - line_sensor_->sensor[12];
	sensor_values_[12] = 1000 - line_sensor_->sensor[13];

	for(uint16_t i = 0; i < 12; i++){
		if(sensor_values_[i] >= 500) sensor_digital_values_[i] = 1;
		else sensor_digital_values_[i] = 0;
	}

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

void LineTrace::steeringAngleTrace()
{
	float steering_angle = calcAngle();

	float r = 0;
	float current_velocity = velocity_ctrl_->getCurrentVelocity();
	//float current_velocity = 0.1;
	float target_omega = 0;

	if(steering_angle != 0){
		r = CENTER_OF_ROTATION_TO_CENTER_OF_SENSOR / tan(steering_angle);
		target_omega = current_velocity / r;
	}
	else target_omega = 0;

	velocity_ctrl_->setVelocity(target_velocity_, target_omega);
	target_omega_ = target_omega;

	monitor_target_omega = target_omega;
	monitor_r = r;
}
// ---------------------------------------------------------------------------------------------------//
// ----------------------------------Standar line following ------------------------------------------//
// ---------------------------------------------------------------------------------------------------//
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

	if(mode_selector_ == FIRST_RUNNING){
		p = kp_slow_ * diff;
		d = (kd_slow_ / 10) * (diff - pre_diff) / DELTA_T;
		i += ki_slow_ * diff * DELTA_T;
	}
	else{
		p = kp_ * diff;
		d = (kd_ / 10) * (diff - pre_diff) / DELTA_T;
		i += ki_ * diff * DELTA_T;
	}

	float rotation_ratio = p + d + i;

	velocity_ctrl_->setTranslationVelocityOnly(target_velocity_, rotation_ratio);

	pre_diff = diff;

}

// ---------------------------------------------------------------------------------------//
// -------------------------------------Logging-------------------------------------------//
// ---------------------------------------------------------------------------------------//
void LineTrace::loggerStart()
{
	encoder_->clearDistance10mm();
	odometry_->clearPotition();


	if(mode_selector_ == FIRST_RUNNING){
		first_run_distance_logger_->clearLogs();
		first_run_theta_logger_->clearLogs();
		first_run_crossline_distance_logger_->clearLogs();
		first_run_sideline_distance_logger_->clearLogs();

		first_run_distance_logger_->start();
		first_run_theta_logger_->start();
		first_run_crossline_distance_logger_->start();
		first_run_sideline_distance_logger_->start();
	}
	else{
		accdec_run_distance_logger_->clearLogs();
		accdec_run_theta_logger_->clearLogs();
		accdec_run_crossline_distance_logger_->clearLogs();
		accdec_run_sideline_distance_logger_->clearLogs();

		accdec_run_distance_logger_->start();
		accdec_run_theta_logger_->start();
		accdec_run_crossline_distance_logger_->start();
		accdec_run_sideline_distance_logger_->start();
	}

	total_distance_logger_->clearLogs();
	total_distance_logger_->start();

	logging_flag_ = true;
}

void LineTrace::debuggerStart()
{
	/*
	debugger_->clearLogs();
	debugger_->start();
	debugger2_->clearLogs();
	debugger2_->start();
	*/
	debugger3_->clearLogs();
	debugger3_->start();
	debugger4_->clearLogs();
	debugger4_->start();

}

void LineTrace::loggerStop()
{

	first_run_distance_logger_->stop();
	first_run_theta_logger_->stop();

	accdec_run_distance_logger_->stop();
	accdec_run_theta_logger_->stop();

	first_run_crossline_distance_logger_->stop();
	first_run_sideline_distance_logger_->stop();

	accdec_run_crossline_distance_logger_->stop();
	accdec_run_sideline_distance_logger_->stop();

	total_distance_logger_->stop();

	logging_flag_ = false;
}

void LineTrace::debuggerStop()
{
	//debugger_->stop();
	//debugger2_->stop();
	debugger3_->stop();
	debugger4_->stop();

}

void LineTrace::storeFirstRunCrossLineDistance()
{
	first_run_crossline_distance_logger_->storeLogs(encoder_->getTotalDistance());
}

void LineTrace::storeAccDecRunCrossLineDistance()
{
	accdec_run_crossline_distance_logger_->storeLogs(encoder_->getTotalDistance());
}

void LineTrace::storeFirstRunSideLineDistance()
{
	first_run_sideline_distance_logger_->storeLogs(encoder_->getTotalDistance());
}

void LineTrace::storeAccDecRunSideLineDistance()
{
	accdec_run_sideline_distance_logger_->storeLogs(encoder_->getTotalDistance());
}


void LineTrace::storeLogs()
{
	if(logging_flag_ == true){
		if(mode_selector_ == FIRST_RUNNING){
			first_run_distance_logger_->storeLogs(encoder_->getDistance10mm());
			first_run_theta_logger_->storeLogs(odometry_->getTheta());
		}
		else{
			accdec_run_distance_logger_->storeLogs(encoder_->getDistance10mm());
			accdec_run_theta_logger_->storeLogs(odometry_->getTheta());
		}

		total_distance_logger_->storeLogs(encoder_->getTotalDistance());

	}
}


// ---------------------------------------------------------------------------------------------------//
// ----------------------------------Position correction----------------------------------------------//
// ---------------------------------------------------------------------------------------------------//
void LineTrace::correctionTotalDistanceFromCrossLine()
{
	/*
	encoder_->setTotalDistance(crossline_distance_[crossline_idx_] / DISTANCE_CORRECTION_CONST);
	crossline_idx_++;
	correction_check_cnt_ = 0;
	*/
	/*
	for(uint16_t i = 0; i < CROSSLINE_SIZE; i++){
		float temp_crossline_distance = crossline_distance_[i];
		float diff = abs(temp_crossline_distance - (encoder_->getTotalDistance() / DISTANCE_CORRECTION_CONST));
		if(diff <= 250){
			correction_check_cnt_ = 0;
			encoder_->setTotalDistance(crossline_distance_[i] / DISTANCE_CORRECTION_CONST);
			break;
		}
	}
	*/

	while(crossline_idx_ <= first_run_crossline_distance_logger_->getLogsSize()){
		float temp_crossline_distance = first_run_crossline_distance_logger_->getLogData(crossline_idx_);
		float diff = abs(temp_crossline_distance - (encoder_->getTotalDistance() / DISTANCE_CORRECTION_CONST));
		if(diff <= 250){
			correction_check_cnt_ = 0;
			encoder_->setTotalDistance(first_run_crossline_distance_logger_->getLogData(crossline_idx_) / DISTANCE_CORRECTION_CONST);
			crossline_idx_++;
			break;
		}
		crossline_idx_++;
	}

	if(crossline_idx_ >= first_run_crossline_distance_logger_->getLogsSize()) crossline_idx_ = first_run_crossline_distance_logger_->getLogsSize() - 1;

}

void LineTrace::correctionTotalDistanceFromSideMarker()
{

	/*
	for(uint16_t i = 0; i < first_run_sideline_distance_logger_->getLogsSize(); i++){
		//float temp_sideline_distance = sideline_distance_[i];
		float temp_sideline_distance = first_run_sideline_distance_logger_->getLogData(i);
		float diff = abs(temp_sideline_distance - (encoder_->getTotalDistance() / DISTANCE_CORRECTION_CONST));
		//if(diff <= 230){
		if(diff <= 450){
			correction_check_cnt_ = 0;
			encoder_->setTotalDistance(first_run_sideline_distance_logger_->getLogData(i) / DISTANCE_CORRECTION_CONST);
			break;
		}
	}
	*/
	while(sideline_idx_ <= first_run_sideline_distance_logger_->getLogsSize()){
		float temp_sideline_distance = first_run_sideline_distance_logger_->getLogData(sideline_idx_);
		float diff = abs(temp_sideline_distance - (encoder_->getTotalDistance() / DISTANCE_CORRECTION_CONST));
		//if(diff <= 230){
		if(diff <= 700){
			correction_check_cnt_ = 0;
			encoder_->setTotalDistance(first_run_sideline_distance_logger_->getLogData(sideline_idx_) / DISTANCE_CORRECTION_CONST);
			break;
		}
		sideline_idx_++;
	}

	if(sideline_idx_ >= first_run_sideline_distance_logger_->getLogsSize()) sideline_idx_ = first_run_sideline_distance_logger_->getLogsSize() - 1;

}

// ---------------------------------------------------------------------------------------------------//
// ------------------------ Acceleration / deceleration processing------------------------------------//
// ---------------------------------------------------------------------------------------------------//
float LineTrace::dtheta2Velocity(float dtheta)
{
	float velocity;

	if(mode_selector_ == SECOND_RUNNING){
		if(dtheta > 0.0025) velocity = min_velocity_; //1.0 R10
		else if(dtheta > 0.001) velocity = 2.0; //midium radius and snake
		else velocity = max_velocity_; //3.0 large R and straight
	}
	else if(mode_selector_ == THIRD_RUNNING){
		if(dtheta > 0.0025) velocity = min_velocity2_; //1.0 R10
		else if(dtheta > 0.001) velocity = 2.0; //midium radius and snake
		else velocity = max_velocity2_; //4.0 large R and straight
	}
	else if (mode_selector_ == FOURTH_RUNNING){
		if(dtheta > 0.0030) velocity = min_velocity3_; //1.0 R10
		else if(dtheta > 0.0017) velocity = 2.0; //
		else if(dtheta > 0.0013) velocity = 2.2; // snake
		else if(dtheta > 0.0010) velocity = 3.0; // large R
		else if(dtheta > 0.0007) velocity = 3.0; // large R
		else if(dtheta > 0.0005) velocity = 3.5; //
		else velocity = max_velocity3_; //6.0 straight
	}
	else if (mode_selector_ == FIFTH_RUNNING){
		if(dtheta > 0.0030) velocity = min_velocity4_; //1.0 R10
		else if(dtheta > 0.0017) velocity = 2.0; //
		else if(dtheta > 0.0013) velocity = 2.2; // snake
		else if(dtheta > 0.0010) velocity = 3.0; // large R
		else if(dtheta > 0.0007) velocity = 3.0; // large R
		else if(dtheta > 0.0005) velocity = 3.5; //
		else velocity = max_velocity4_; //6.0 straight
	}
	else velocity = 1.5;

	return velocity;
}

float LineTrace::radius2Velocity(float radius)
{
	float velocity;

	if(mode_selector_ == SECOND_RUNNING){
		if(radius < 400) velocity = min_velocity_;
		else if(radius < 500) velocity = 1.5;
		else if(radius < 650) velocity = 2.0;
		else if(radius < 1500) velocity = 2.5;
		else if(radius < 2000) velocity = 3.0;
		else velocity = max_velocity_;
	}
	else if(mode_selector_ == THIRD_RUNNING){
		if(radius < 400) velocity = min_velocity2_;
		else if(radius < 500) velocity = 1.5;
		else if(radius < 650) velocity = 2.0;
		else if(radius < 1500) velocity = 2.5;
		else if(radius < 2000) velocity = 3.3;
		else velocity = max_velocity2_;
	}
	else if(mode_selector_ == FOURTH_RUNNING){
		if(radius < 200) velocity = min_velocity3_;
		else if(radius < 400) velocity = 2.4;
		else if(radius < 650) velocity = 3.0;
		else if(radius < 1100) velocity = 3.3;
		else if(radius < 1900) velocity = 3.3;
		else if(radius < 2100) velocity = 4.0;
		else velocity = max_velocity3_;
	}
	else if(mode_selector_ == FIFTH_RUNNING){
		if(radius < 200) velocity = min_velocity3_;
		else if(radius < 400) velocity = 2.4;
		else if(radius < 650) velocity = 3.0;
		else if(radius < 1100) velocity = 3.3;
		else if(radius < 1900) velocity = 3.3;
		else if(radius < 2100) velocity = 4.0;
		else velocity = max_velocity4_;
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

void LineTrace::decelerateProcessing(const float am, const float *p_distance)
{
	for(uint16_t i = first_run_distance_logger_->getLogsSize() - 1; i >= 1; i--){
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
	for(uint16_t i = 0; i <= first_run_distance_logger_->getLogsSize() - 1; i++){
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

void LineTrace::startVelocityPlay()
{
	encoder_->clearTotalDistance();
	velocity_play_flag_ = true;
	velocity_table_idx_ = 0;
	ref_distance_ = 0;
}

void LineTrace::stopVelocityPlay()
{
	velocity_play_flag_ = false;
	velocity_table_idx_ = 0;
	ref_distance_ = 0;
}

void LineTrace::updateTargetVelocity()
{
	if(velocity_play_flag_ == true){
		/*
		while(encoder_->getTotalDistance() * DISTANCE_CORRECTION_CONST >= ref_distance_){
			ref_distance_ += ref_delta_distances_[velocity_table_idx_];
			velocity_table_idx_++;
		}
		*/
		if(encoder_->getTotalDistance() * DISTANCE_CORRECTION_CONST >= ref_distance_){
			ref_distance_ += ref_delta_distances_[velocity_table_idx_];
			velocity_table_idx_++;
		}

		if(velocity_table_idx_ >= LOG_SIZE_DIS) velocity_table_idx_ = LOG_SIZE_DIS - 1;

		setTargetVelocity(velocity_table_[velocity_table_idx_]);

		/*
		mon_ref_dis = ref_distance_;
		mon_current_dis = encoder_->getTotalDistance();
		mon_vel_idx = velocity_table_idx_;
		mon_tar_vel = velocity_table_[velocity_table_idx_];
		*/

	}
}

bool LineTrace::isTargetDistance(float target_distance)
{
	bool ret = false;
	if(encoder_->getDistance10mm() >= target_distance){
		ret = true;
	}

	return ret;
}

bool LineTrace::isCrossLine()
{
	static uint16_t cnt = 0;
	//float sensor_edge_val_l = (line_sensor_->sensor[3] + line_sensor_->sensor[4]) / 2;
	//float sensor_edge_val_r = (line_sensor_->sensor[9] + line_sensor_->sensor[10]) / 2;
	float sensor_edge_val_l = (line_sensor_->sensor[0] + line_sensor_->sensor[1]) / 2;
	float sensor_edge_val_r = (line_sensor_->sensor[12] + line_sensor_->sensor[13]) / 2;
	static bool flag = false;
	//static bool white_flag = false;
	mon_ave_l = sensor_edge_val_l;
	mon_ave_r = sensor_edge_val_r;

	//if(white_flag == false){
		if(sensor_edge_val_l < 700 && sensor_edge_val_r < 700 && encoder_->getCrossLineIgnoreDistance() >= 50){
			cnt++;
		}
		else{
			cnt = 0;
		}

		if(cnt >= 3){
			flag = true;
			//white_flag = true;
			cnt = 0;

			side_sensor_->enableIgnore();
			encoder_->clearSideLineIgnoreDistance();
			encoder_->clearCrossLineIgnoreDistance();

			stable_cnt_reset_flag_ = true; //Because the conditions do not differ between when you tremble and when you do not tremble
			//stable_flag_force_ = true;
			if(mode_selector_ == FIRST_RUNNING){
				store_check_cnt_ = 0;
				storeFirstRunCrossLineDistance();
			}
			else{
				store_check_cnt_ = 0;
				correctionTotalDistanceFromCrossLine();
				storeAccDecRunCrossLineDistance(); //for correction check
			}
		}

	return flag;
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

	if(radius >= 2000){
		stable_cnt++;
	}
	else{
		stable_cnt = 0;
	}

	//if(stable_cnt >= 25){ //250mm
	if(stable_cnt >= int(250 / DISTANCE_TO_RECORD)){ //250mm
		ret = true;
	}

	return ret;
}

float LineTrace::calcRadius(float distance, float theta)
{
	if(theta == 0) theta = 0.000001;
	return distance / theta;
}

// -------public---------- //
// ---------------------------------------------------------------------------------------------------//
// ------------------------------------ Initialize----------------------------------------------------//
// ---------------------------------------------------------------------------------------------------//
void LineTrace::init()
{
	float temp_kp, temp_ki, temp_kd;
	sd_read_array_float("PARAMS", "KP.TXT", 1, &temp_kp);
	sd_read_array_float("PARAMS", "KI.TXT", 1, &temp_ki);
	sd_read_array_float("PARAMS", "KD.TXT", 1, &temp_kd);
	setGain(temp_kp, temp_ki, temp_kd);

	float temp_kp_slow, temp_ki_slow, temp_kd_slow;
	sd_read_array_float("PARAMS", "KP_SLOW.TXT", 1, &temp_kp_slow);
	sd_read_array_float("PARAMS", "KI_SLOW.TXT", 1, &temp_ki_slow);
	sd_read_array_float("PARAMS", "KD_SLOW.TXT", 1, &temp_kd_slow);
	setGainSlow(temp_kp_slow, temp_ki_slow, temp_kd_slow);

	float temp_velocity, temp_max_velocity, temp_min_velocity, temp_max_velocity2, temp_min_velocity2,
		temp_max_velocity3, temp_min_velocity3, temp_max_velocity4, temp_min_velocity4;
	sd_read_array_float("PARAMS", "TARVEL.TXT", 1, &temp_velocity);
	sd_read_array_float("PARAMS", "TARVEL2.TXT", 1, &temp_max_velocity);
	sd_read_array_float("PARAMS", "MINVEL2.TXT", 1, &temp_min_velocity);
	sd_read_array_float("PARAMS", "TARVEL3.TXT", 1, &temp_max_velocity2);
	sd_read_array_float("PARAMS", "MINVEL3.TXT", 1, &temp_min_velocity2);
	sd_read_array_float("PARAMS", "TARVEL4.TXT", 1, &temp_max_velocity3);
	sd_read_array_float("PARAMS", "MINVEL4.TXT", 1, &temp_min_velocity3);
	sd_read_array_float("PARAMS", "TARVEL5.TXT", 1, &temp_max_velocity4);
	sd_read_array_float("PARAMS", "MINVEL5.TXT", 1, &temp_min_velocity4);

	setTargetVelocity(temp_velocity);
	setMaxVelocity(temp_max_velocity);
	setMinVelocity(temp_min_velocity);
	setMaxVelocity2(temp_max_velocity2);
	setMinVelocity2(temp_min_velocity2);
	setMaxVelocity3(temp_max_velocity3);
	setMinVelocity3(temp_min_velocity3);
	setMaxVelocity4(temp_max_velocity4);
	setMinVelocity4(temp_min_velocity4);

	float temp_acc, temp_dec;
	sd_read_array_float("PARAMS", "ACC.TXT", 1, &temp_acc);
	sd_read_array_float("PARAMS", "DEC.TXT", 1, &temp_dec);
	setMaxAccDec(temp_acc, temp_dec);

	float temp_acc2 = 0, temp_dec2 = 0;
	sd_read_array_float("PARAMS", "ACC2.TXT", 1, &temp_acc2);
	sd_read_array_float("PARAMS", "DEC2.TXT", 1, &temp_dec2);
	setMaxAccDec2(temp_acc2, temp_dec2);

	float temp_acc3 = 0, temp_dec3 = 0;
	sd_read_array_float("PARAMS", "ACC3.TXT", 1, &temp_acc3);
	sd_read_array_float("PARAMS", "DEC3.TXT", 1, &temp_dec3);
	setMaxAccDec3(temp_acc3, temp_dec3);

	float temp_acc4 = 0, temp_dec4 = 0;
	sd_read_array_float("PARAMS", "ACC4.TXT", 1, &temp_acc4);
	sd_read_array_float("PARAMS", "DEC4.TXT", 1, &temp_dec4);
	setMaxAccDec4(temp_acc4, temp_dec4);
}

// ---------------------------------------------------------------------------------------------------//
// ------------------------------- Line following gain------------------------------------------------//
// ---------------------------------------------------------------------------------------------------//
void LineTrace::setGain(float kp, float ki, float kd)
{
	kp_ = kp;
	ki_ = ki;
	kd_ = kd;
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

void LineTrace::setGainSlow(float kp, float ki, float kd)
{
	kp_slow_ = kp;
	ki_slow_ = ki;
	kd_slow_ = kd;
}

float LineTrace::getKpSlow()
{
	return kp_slow_;
}

float LineTrace::getKiSlow()
{
	return ki_slow_;
}

float LineTrace::getKdSlow()
{
	return kd_slow_;
}
// ---------------------------------------------------------------------------------------------------//
// ------------------------------ Velocity setting----------------------------------------------------//
// ---------------------------------------------------------------------------------------------------//

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

void LineTrace::setMinVelocity(float velocity)
{
	min_velocity_ = velocity;
}

void LineTrace::setMaxVelocity2(float velocity)
{
	max_velocity2_ = velocity;
}

void LineTrace::setMinVelocity2(float velocity)
{
	min_velocity2_ = velocity;
}

void LineTrace::setMaxVelocity3(float velocity)
{
	max_velocity3_ = velocity;
}

void LineTrace::setMinVelocity3(float velocity)
{
	min_velocity3_ = velocity;
}

void LineTrace::setMaxVelocity4(float velocity)
{
	max_velocity4_ = velocity;
}

void LineTrace::setMinVelocity4(float velocity)
{
	min_velocity4_ = velocity;
}

float LineTrace::getTargetVelocity()
{
	return target_velocity_;
}

float LineTrace::getMaxVelocity()
{
	return max_velocity_;
}

float LineTrace::getMinVelocity()
{
	return min_velocity_;
}

float LineTrace::getMaxVelocity2()
{
	return max_velocity2_;
}

float LineTrace::getMinVelocity2()
{
	return min_velocity2_;
}

float LineTrace::getMaxVelocity3()
{
	return max_velocity3_;
}

float LineTrace::getMinVelocity3()
{
	return min_velocity3_;
}

float LineTrace::getMaxVelocity4()

{
	return max_velocity4_;
}

float LineTrace::getMinVelocity4()
{
	return min_velocity4_;
}
// ---------------------------------------------------------------------------------------------------//
// ------------------------------ Acceleration setting------------------------------------------------//
// ---------------------------------------------------------------------------------------------------//
void LineTrace::setMaxAccDec(const float acc, const float dec)
{
	max_acc_ = acc;
	max_dec_ = dec;
}

void LineTrace::setMaxAccDec2(const float acc, const float dec)
{
	max_acc2_ = acc;
	max_dec2_ = dec;
}

void LineTrace::setMaxAccDec3(const float acc, const float dec)
{
	max_acc3_ = acc;
	max_dec3_ = dec;
}

void LineTrace::setMaxAccDec4(const float acc, const float dec)
{
	max_acc4_ = acc;
	max_dec4_ = dec;
}

float LineTrace::getMaxAcc()
{
	return max_acc_;
}

float LineTrace::getMaxDec()
{
	return max_dec_;
}

float LineTrace::getMaxAcc2()
{
	return max_acc2_;
}

float LineTrace::getMaxDec2()
{
	return max_dec2_;
}

float LineTrace::getMaxAcc3()
{
	return max_acc3_;
}

float LineTrace::getMaxDec3()
{
	return max_dec3_;
}

float LineTrace::getMaxAcc4()
{
	return max_acc4_;
}

float LineTrace::getMaxDec4()
{
	return max_dec4_;
}

// ---------------------------------------------------------------------------------------------------//
// ---------------------------------------- Flip -----------------------------------------------------//
// ---------------------------------------------------------------------------------------------------//
void LineTrace::flip()
{
	if(excution_flag_ == true){
		// ---- line following processing -----//
		pidTrace();
		//steeringAngleTrace();

		// ---- Target Velocity Updata ------//
		updateTargetVelocity();

		// ----- Processing at regular distances -----//
		if(isTargetDistance(DISTANCE_TO_RECORD) == true){
			// -------- Detect Robot stabilization ------//
			if(isStable() == true && side_sensor_->getStatusL() == false){ // Stabilizing and side sensor is black
				stable_flag_ = true;
			}

			// ---- Store Logs ------//
			storeLogs();

			// ---reset total cnt ---//
			encoder_->clearDistance10mm();
			odometry_->clearPotition();
		}

		// ----- cross line ignore processing ------//
		if(isCrossLine() == true){ //detect cross line
			//side_sensor_->enableIgnore(); //moved to isCrossLine function
			//encoder_->clearCrossLineIgnoreDistance();//moved to isCrossLine function
			// Note: Store cross line distance here.
			//led_.LR(1, -1);
		}

		// ------- Store side line distance or correction distance------//
		//if(stable_flag_ == true && side_sensor_->getStatusL() == true){ //Stabilizing and side sensor is white
		//if((stable_flag_force_ == true || stable_flag_ == true) && side_sensor_->getStatusL() == true && encoder_->getSideLineIgnoreDistance() >= 120){ //Stabilizing and side sensor is white and no ignore side line
		if(stable_flag_ == true && side_sensor_->getStatusL() == true && side_sensor_->getStatusR() == false && side_sensor_->getIgnoreFlag() == false){ //Stabilizing and side sensor is white and no ignore side line
			//correction_check_cnt_ = 0;

			if(mode_selector_ == FIRST_RUNNING){
				store_check_cnt_ = 0;
				storeFirstRunSideLineDistance();
			}
			else{
				store_check_cnt_ = 0;
				correctionTotalDistanceFromSideMarker();
				storeAccDecRunSideLineDistance(); //for correction check
			}

			stable_flag_ = false;
			stable_flag_force_ = false;
			stable_cnt_reset_flag_ = true;
		}



		if(side_sensor_->getIgnoreFlag() == true && encoder_->getSideLineIgnoreDistance() >= 140){ //Ignore Side line
			side_sensor_->disableIgnore();
			//led_.LR(0, -1);
		}


		//if(stable_flag_ == true) led_.LR(-1, 1);
		//else led_.LR(-1, 0);


		// ------ All sideline storing -------//
		/*
		if(all_sideline_flag_ == false && (side_sensor_->getStatus() & 0x02) == 0x02){
			all_sideline_flag_ = true;

			if(mode_selector_ == FIRST_RUNNING){
				storeAllSideLineDistance();
			}
			else{
				//correctionTotalDistanceFromAllSideMarker();
				//correction_check_cnt_ = 0;
			}
		}
		else if(all_sideline_flag_ == true && (~(side_sensor_->getStatus()) & 0x02) == 0x02){
			all_sideline_flag_ = false;
		}
		*/

		// ----- Emergency stop processing------//
		if(line_sensor_->emergencyStop() == true){
			velocity_ctrl_->setTranslationVelocityOnly(0, 0);
			down_force_unit_->off();
			//led_.LR(1, -1);
		}
		else{
			//led_.LR(0, -1);
		}


	}
}

// ---------------------------------------------------------------------------------------------------//
// ---------------------------------- Mode set to stop------------------------------------------------//
// ---------------------------------------------------------------------------------------------------//
void LineTrace::setMode(int16_t mode)
{
	mode_selector_ = mode;
}

void LineTrace::start()
{
	if(mode_selector_ == FIRST_RUNNING)	down_force_unit_->on(DOWN_FORCE_POWER_SEARCHING, DOWN_FORCE_POWER_SEARCHING);
	else	down_force_unit_->on(DOWN_FORCE_POWER, DOWN_FORCE_POWER);
	HAL_Delay(500);

	excution_flag_ = true;
	i_reset_flag_ = true;
	velocity_ctrl_->start();
	side_sensor_->resetWhiteLineCnt();
	odometry_->clearPotition();

	crossline_idx_ = 0;
	crossline_idx2_ = 0;
	sideline_idx_ = 0;
	sideline_idx2_ = 0;
	all_sideline_idx_ = 0;
}


void LineTrace::running()
{
	uint16_t stage = 0;
	bool goal_flag = false;
	bool goal_judge_flag = false;
	start();
	debuggerStart();

	while(goal_flag == false){
		switch(stage){
		case 0:
			if(side_sensor_->getStatusR() == true){
				loggerStart();

				if(mode_selector_ != FIRST_RUNNING){ // Other than first running
					startVelocityPlay();
				}

				encoder_->clearSideLineIgnoreDistance();
				encoder_->clearCrossLineIgnoreDistance();
				encoder_->clearTotalDistance();
				led_.LR(0, -1);
				stage = 5;
			}

			break;

		case 5:
			if(side_sensor_->getStatusR() == false) stage = 10;

			break;
		case 10:
			//if(side_sensor_->getWhiteLineCntR() == 2){
			if(side_sensor_->getStatusL() == true){
				goal_judge_flag = false;
				encoder_->clearGoalJudgeDistance();
				led_.fullColor('B');
			}

			if(goal_judge_flag == false && side_sensor_->getStatusR() == true && encoder_->getGoalJudgeDistance() >= 30){
				goal_judge_flag = true;
				encoder_->clearGoalJudgeDistance();
				ignore_check_cnt_ = 0;

				led_.fullColor('Y');
			}
			else if(goal_judge_flag == true && encoder_->getGoalJudgeDistance() >= 30){
				led_.fullColor('M');
				loggerStop();
				debuggerStop();
				stopVelocityPlay();
				HAL_Delay(100); //Run through after the goal

				setTargetVelocity(0);
				HAL_Delay(500); //Stop for a while on the spot

				goal_flag = true;
				goal_judge_flag = false;

			}

			break;
		}

		// ---------Confirmation when corrected ------------//
		correction_check_cnt_++;
		if(correction_check_cnt_ >= 10000) correction_check_cnt_ = 10000;

		if(correction_check_cnt_ <= 300) led_.fullColor('R');

		store_check_cnt_++;
		if(store_check_cnt_>= 10000) store_check_cnt_ = 10000;

		if(store_check_cnt_ <= 500) led_.LR(1, -1);
		else led_.LR(0, -1);

		ignore_check_cnt_++;
		if(ignore_check_cnt_>= 10000) ignore_check_cnt_= 10000;

		if(ignore_check_cnt_ <= 200) led_.fullColor('Y');
		else led_.fullColor('B');
	}

	stop();
}

void LineTrace::stop()
{
	down_force_unit_->off();

	excution_flag_ = false;
	velocity_ctrl_->stop();

	led_.LR(-1, 1);

	///debugger_->saveLogs("DEBUG", "translation_ratio");
	//debugger2_->saveLogs("DEBUG", "rotation_ratio");
	debugger3_->saveLogs("DEBUG", "current_velocity");
	debugger4_->saveLogs("DEBUG", "target_velocity");


	if(mode_selector_ == FIRST_RUNNING){ //First running
		first_run_distance_logger_->saveLogs("TEST", "first_run_distances");
		first_run_theta_logger_->saveLogs("TEST", "first_run_thetas");
		first_run_crossline_distance_logger_ ->saveLogs("TEST", "first_run_crossline_distances");
		first_run_sideline_distance_logger_ ->saveLogs("TEST", "first_run_sideline_distances");
		total_distance_logger_->saveLogs("TEST", "first_run_total_distances");
	}
	else if(mode_selector_ == SECOND_RUNNING){ //Secondary run
		accdec_run_distance_logger_->saveLogs("TEST", "second_run_distances");
		accdec_run_theta_logger_->saveLogs("TEST", "second_run_thetas");
		accdec_run_crossline_distance_logger_ ->saveLogs("TEST", "second_run_crossline_distances");
		accdec_run_sideline_distance_logger_ ->saveLogs("TEST", "second_run_sideline_distances");
		total_distance_logger_->saveLogs("TEST", "second_run_total_distances");

	}
	else if(mode_selector_ == THIRD_RUNNING){ //Third run
		accdec_run_distance_logger_->saveLogs("TEST", "third_run_distances");
		accdec_run_theta_logger_->saveLogs("TEST", "third_run_thetas");
		accdec_run_crossline_distance_logger_ ->saveLogs("TEST", "third_run_crossline_distances");
		accdec_run_sideline_distance_logger_ ->saveLogs("TEST", "third_run_sideline_distances");
		total_distance_logger_->saveLogs("TEST", "third_run_total_distances");
	}
	else if(mode_selector_ == FOURTH_RUNNING){ //Fourth run
		accdec_run_distance_logger_->saveLogs("TEST", "fourth_run_distances");
		accdec_run_theta_logger_->saveLogs("TEST", "fourth_run_thetas");
		accdec_run_crossline_distance_logger_ ->saveLogs("TEST", "fourth_run_crossline_distances");
		accdec_run_sideline_distance_logger_ ->saveLogs("TEST", "fourth_run_sideline_distances");
		total_distance_logger_->saveLogs("TEST", "fourth_run_total_distances");
	}
	else if(mode_selector_ == FIFTH_RUNNING){ //Fifth run
		accdec_run_distance_logger_->saveLogs("TEST", "fifth_run_distances");
		accdec_run_theta_logger_->saveLogs("TEST", "fifth_run_thetas");
		accdec_run_crossline_distance_logger_ ->saveLogs("TEST", "fifth_run_crossline_distances");
		accdec_run_sideline_distance_logger_ ->saveLogs("TEST", "fifth_run_sideline_distances");
		total_distance_logger_->saveLogs("TEST", "fifth_run_total_distances");
	}
	else{ //Other run
		accdec_run_distance_logger_->saveLogs("TEST", "other_distances");
		accdec_run_theta_logger_->saveLogs("TEST", "other_thetas");
		accdec_run_crossline_distance_logger_ ->saveLogs("TEST", "other_run_crossline_distances");
		accdec_run_sideline_distance_logger_ ->saveLogs("TEST", "other_run_sideline_distances");
	}

	led_.LR(-1, 0);

}

// ---------------------------------------------------------------------------------------------------//
// ------------------------------ Create velocity table-----------------------------------------------//
// ---------------------------------------------------------------------------------------------------//
void LineTrace::createVelocityTabele(bool is_from_sd)
{
	if(is_from_sd == true){
		first_run_distance_logger_->importLatestLogs("TEST", "first_run_distances");
		first_run_theta_logger_->importLatestLogs("TEST", "first_run_thetas");

		first_run_crossline_distance_logger_->importLatestLogs("TEST", "first_run_crossline_distances");
		first_run_sideline_distance_logger_->importLatestLogs("TEST", "first_run_sideline_distances");
	}

	const float *p_distance, *p_theta;
	p_distance = first_run_distance_logger_->getLogsPointer();
	p_theta = first_run_theta_logger_->getLogsPointer();


	float temp_distance, temp_theta;
	for(uint16_t i = 0; i < first_run_distance_logger_->getLogsSize(); i++){
		temp_distance = p_distance[i];
		temp_theta = p_theta[i];

		/*
		if(temp_theta == 0) temp_theta = 0.00001;
		float dtheta= abs(temp_theta / temp_distance);
		velocity_table_[i] = dtheta2Velocity(dtheta);
		*/

		if(temp_theta == 0) temp_theta = 0.00001;
		float radius = abs(temp_distance / temp_theta);
		if(radius >= 5000) radius = 5000;
		velocity_table_[i] = radius2Velocity(radius);

		ref_delta_distances_[i] = p_distance[i]; //copy
	}


	if(mode_selector_ == SECOND_RUNNING){
		velocity_table_[0] = min_velocity_;
		// ----- Decelerate processing -----//
		decelerateProcessing(max_dec_, p_distance);
		// ----- Accelerate processing -----//
		accelerateProcessing(max_acc_, p_distance);

		sd_card_->write("TEST", "second_velocity_table", first_run_distance_logger_->getLogsSize(), velocity_table_);
	}
	else if(mode_selector_ == THIRD_RUNNING){
		velocity_table_[0] = min_velocity2_;
		// ----- Decelerate processing -----//
		decelerateProcessing(max_dec2_, p_distance);
		// ----- Accelerate processing -----//
		accelerateProcessing(max_acc2_, p_distance);

		sd_card_->write("TEST", "third_velocity_table", first_run_distance_logger_->getLogsSize(), velocity_table_);
	}
	else if(mode_selector_ == FOURTH_RUNNING){
		velocity_table_[0] = min_velocity3_;
		// ----- Decelerate processing -----//
		decelerateProcessing(max_dec3_, p_distance);
		// ----- Accelerate processing -----//
		accelerateProcessing(max_acc3_, p_distance);

		sd_card_->write("TEST", "fourth_velocity_table", first_run_distance_logger_->getLogsSize(), velocity_table_);
	}
	else if(mode_selector_ == FIFTH_RUNNING){
		velocity_table_[0] = min_velocity4_;
		// ----- Decelerate processing -----//
		decelerateProcessing(max_dec4_, p_distance);
		// ----- Accelerate processing -----//
		accelerateProcessing(max_acc4_, p_distance);

		sd_card_->write("TEST", "fifth_velocity_table", first_run_distance_logger_->getLogsSize(), velocity_table_);
	}

}

void LineTrace::storeDebugLogs10ms()
{
	//debugger_->storeLogs(velocity_ctrl_->getTranslationRatio());
	//debugger2_->storeLogs(velocity_ctrl_->getRotationRatio());
	debugger3_->storeLogs(velocity_ctrl_->getCurrentVelocity());
	debugger4_->storeLogs(target_velocity_);
}
