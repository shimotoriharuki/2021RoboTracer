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
#include "VelocityCtrl.hpp"
#include "HAL_SDcard_lib.h"
#include "SideSensor.hpp"
#include "Encoder.hpp"
#include "Odometry.hpp"
#include "Logger.hpp"

#define DELTA_T 0.001
#define ANGLE_BETWEEN_SENSORS 0.17104 //[rad]
#define SENSOR_ANGLE 2.0525 //[rad]
#define SENSOR_NUM 13
#define CENTER_NUM 6
#define PI 3.1415926535
#define CENTER_OF_ROTATION_TO_CENTER_OF_SENSOR 0.060 //[m]
#define FIRST_RUNNING 0
#define SECOND_RUNNING 1
#define THIRD_RUNNING 2
#define CROSSLINE_SIZE 100
#define SIDELINE_SIZE 1000
//#define MAX_DEC 5
//#define MAX_ACC 8
#define R_RADIUS 0.05

class LineTrace
{
private:
	Motor *motor_;
	LineSensor *line_sensor_;
	VelocityCtrl *velocity_ctrl_;
	LED led_;
	SideSensor *side_sensor_;
	Encoder *encoder_;
	Odometry *odometry_;
	Logger *logger_;
	IMU *imu_;

	float kp_, kd_, ki_;
	float kp_fast_, kd_fast_, ki_fast_;
	float kp_velo_, kd_velo_, ki_velo_;
	bool excution_flag_;
	bool i_reset_flag_;
	float normal_ratio_;
	float sensor_values_[SENSOR_NUM];
	float sensor_digital_values_[SENSOR_NUM];
	float target_velocity_;
	float target_omega_;
	float max_velocity_;
	float max_velocity2_;
	float min_velocity_;
	float min_velocity2_;
	bool logging_flag_;
	float ref_delta_distances_[LOG_DATA_SIZE_DIS];
	float ref_distance_;
	//float thetas[LOG_DATA_SIZE_DIS];
	float velocity_table_[LOG_DATA_SIZE_DIS];
	bool velocity_play_flag_;
	uint16_t velocity_table_idx_;
	int16_t mode_selector_;
	float crossline_distance_[CROSSLINE_SIZE];
	float sideline_distance_[SIDELINE_SIZE];
	float all_sideline_distance_[SIDELINE_SIZE];
	uint16_t crossline_idx_;
	uint16_t sideline_idx_;
	uint16_t all_sideline_idx_;
	bool ignore_crossline_flag_;
	bool stable_flag_;
	bool stable_cnt_reset_flag_;
	float max_acc_, max_dec_;
	float max_acc2_, max_dec2_;
	uint16_t correction_check_cnt_;
	bool all_sideline_flag_;

	// Sensor angle based line following
	float calcError();
	float calcAngle();
	void getSensorValues();
	void calcStandardAngle(float &, uint16_t &);
	void calcNormalizedSensorValue(const uint16_t, float &, float &);
	void calcDeltaTheta(const float, const float, float &);
	void steeringAngleTrace();

	// Standard line following
	void pidTrace();

	// Logging
	void loggerStart();
	void loggerStop();
	void storeCrossLineDistance();
	void storeSideLineDistance();
	void storeAllSideLineDistance();
	float calcRadius(float, float);

	// position correction
	void correctionTotalDistanceFromCrossLine();
	void correctionTotalDistanceFromSideMarker();
	void correctionTotalDistanceFromAllSideMarker();

	// Acceleration / deceleration processing
	float radius2Velocity(float);
	float radius2VelocityFnc(float);
	void decelerateProcessing(const float, const float *);
	void accelerateProcessing(const float, const float *);
	void startVelocityPlay();
	void stopVelocityPlay();
	void updateTargetVelocity();

	// Status check
	bool isTargetDistance(float);
	bool isCrossLine();
	bool isStable();

public:
	LineTrace(Motor *, LineSensor *, VelocityCtrl *, SideSensor * ,Encoder *, Odometry *, Logger *, IMU *);
	void init();
	void setGain(float, float, float);
	void setGainFast(float, float, float);
	void setVeloGain(float, float, float);
	float getKp();
	float getKi();
	float getKd();
	float getKpFast();
	float getKiFast();
	float getKdFast();
	float getKpV();
	float getKiV();
	float getKdV();
	void setNormalRatio(float);
	void setTargetVelocity(float);
	void setMaxVelocity(float);
	void setMaxVelocity2(float);
	void setMinVelocity(float);
	void setMinVelocity2(float);
	float getTargetVelocity();
	float getMaxVelocity();
	float getMaxVelocity2();
	float getMinVelocity();
	float getMinVelocity2();
	void setMaxAccDec(const float, const float);
	void setMaxAccDec2(const float, const float);
	float getMaxAcc();
	float getMaxDec();
	float getMaxAcc2();
	float getMaxDec2();
	void flip();
	void flip100ns();
	void start();
	void stop();
	void running();
	void storeLogs();
	float getTargetOmega();

	void createVelocityTabele();
	void createVelocityTabeleFromSD();
	void setMode(int16_t);

};



#endif /* INC_LINETRACE_HPP_ */
