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
#include "ESC.hpp"
#include "Logger.hpp"
#include "Logger2.hpp"
#include "SdCard.hpp"
#include "DownForceUnit.hpp"

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
#define FOURTH_RUNNING 3
#define FIFTH_RUNNING 4

#define LOG_SIZE_TIM 35 // Time based size. Can record for 50 seconds every 10 ms. max 3000
#define LOG_SIZE_DIS 60 // Distance based size. Can record for 60 m every 10 mm
#define LOG_CROSSLINE_SIZE 100
#define LOG_SIDELINE_SIZE 100

#define R_RADIUS 0.05
#define DISTANCE_CORRECTION_CONST 1 //0.9663

#define DOWN_FORCE_POWER_SEARCHING 0.5
#define DOWN_FORCE_POWER 0.5

class LineTrace
{
private:
	Motor *motor_;
	LineSensor *line_sensor_;
	VelocityCtrl *velocity_ctrl_;
	LED led_;
	SideSensor *side_sensor_;
	Encoder *encoder_;
	IMU *imu_;
    DownForceUnit *down_force_unit_;
    sdCard *sd_card_;

    Logger2 *debugger_, *debugger2_;
    Logger2 *debugger3_, *debugger4_;
    Logger2 *first_run_distance_logger_, *first_run_theta_logger_;
    Logger2 *accdec_run_distance_logger_, *accdec_run_theta_logger_;
    Logger2 *first_run_crossline_distance_logger_, *first_run_sideline_distance_logger_;
    Logger2 *accdec_run_crossline_distance_logger_, *accdec_run_sideline_distance_logger_;
    Logger2 *total_distance_logger_;

	float kp_, kd_, ki_;
	float kp_slow_, kd_slow_, ki_slow_;
	bool excution_flag_;
	bool i_reset_flag_;
	float normal_ratio_;
	float sensor_values_[SENSOR_NUM];
	float sensor_digital_values_[SENSOR_NUM];
	float target_velocity_;
	float target_omega_;
	float max_velocity_, min_velocity_;
	float max_velocity2_, min_velocity2_;
	float max_velocity3_, min_velocity3_;
	float max_velocity4_, min_velocity4_;
	bool logging_flag_;
	float ref_delta_distances_[LOG_SIZE_DIS];
	float ref_distance_;
	float velocity_table_[LOG_SIZE_DIS];
	bool velocity_play_flag_;
	uint16_t velocity_table_idx_;
	int16_t mode_selector_;
	uint16_t crossline_idx_;
	uint16_t crossline_idx2_;
	uint16_t sideline_idx_;
	uint16_t sideline_idx2_;
	uint16_t all_sideline_idx_;
	bool ignore_crossline_flag_;
	bool stable_flag_;
	bool stable_flag_force_;
	bool stable_cnt_reset_flag_;
	float max_acc_, max_dec_;
	float max_acc2_, max_dec2_;
	float max_acc3_, max_dec3_;
	float max_acc4_, max_dec4_;
	uint16_t correction_check_cnt_;
	uint16_t store_check_cnt_;
	uint16_t ignore_check_cnt_;
	bool all_sideline_flag_;
	bool running_flag_;



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
	void debuggerStart();
	void debuggerStop();
	void storeFirstRunCrossLineDistance();
	void storeAccDecRunCrossLineDistance();
	void storeFirstRunSideLineDistance();
	void storeAccDecRunSideLineDistance();
	void storeLogs();

	// position correction
	void correctionTotalDistanceFromCrossLine();
	void correctionTotalDistanceFromSideMarker();
	//void correctionTotalDistanceFromAllSideMarker();

	// Acceleration / deceleration processing
	float dtheta2Velocity(float);
	float radius2Velocity(float);
	float radius2VelocityFnc(float);
	void shiftVelocityTable(float *, int16_t);
	void decelerateProcessing(const float, const float *);
	void accelerateProcessing(const float, const float *);
	void startVelocityPlay();
	void stopVelocityPlay();
	void updateTargetVelocity();

	// Status check
	bool isTargetDistance(float);
	bool isCrossLine();
	bool isStable();
	float getTargetOmega();
	float calcRadius(float, float);

public:
	LineTrace(Motor *, LineSensor *, VelocityCtrl *, SideSensor * ,Encoder *, IMU *, DownForceUnit*, sdCard *);

	// Initialize
	void init();

	// Line following gain
	void setGain(float, float, float);
	float getKp();
	float getKi();
	float getKd();
	void setGainSlow(float, float, float);
	float getKpSlow();
	float getKiSlow();
	float getKdSlow();


	// Velocity setting
	void setNormalRatio(float);
	void setTargetVelocity(float);
	void setMaxVelocity(float);
	void setMinVelocity(float);
	void setMaxVelocity2(float);
	void setMinVelocity2(float);
	void setMaxVelocity3(float);
	void setMinVelocity3(float);
	void setMaxVelocity4(float);
	void setMinVelocity4(float);
	float getTargetVelocity();
	float getMaxVelocity();
	float getMinVelocity();
	float getMaxVelocity2();
	float getMinVelocity2();
	float getMaxVelocity3();
	float getMinVelocity3();
	float getMaxVelocity4();
	float getMinVelocity4();

	// Acceleration setting
	void setMaxAccDec(const float, const float);
	void setMaxAccDec2(const float, const float);
	void setMaxAccDec3(const float, const float);
	void setMaxAccDec4(const float, const float);
	float getMaxAcc();
	float getMaxDec();
	float getMaxAcc2();
	float getMaxDec2();
	float getMaxAcc3();
	float getMaxDec3();
	float getMaxAcc4();
	float getMaxDec4();

	// Flip
	void flip();

	// mode set to stop
	void setMode(int16_t);
	void start();
	void running();
	void stop();

	// create velocity table
	void createVelocityTabele(bool);
	//void createVelocityTabeleFromSD();

	void storeDebugLogs10ms();

	bool isRunning();
};



#endif /* INC_LINETRACE_HPP_ */
