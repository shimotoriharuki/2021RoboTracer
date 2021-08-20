/*
 * Odometry.hpp
 *
 *  Created on: 2021/08/01
 *      Author: under
 */

#ifndef INC_ODOMETRY_HPP_
#define INC_ODOMETRY_HPP_

#include "Encoder.hpp"
#include "IMU.hpp"
#include "VelocityCtrl.hpp"

class Odometry
{
private:
	Encoder *encoder_;
	IMU *imu_;
	VelocityCtrl *velocity_ctrl_;

	double x_, y_, theta_;
	double delta_theta_;

	void calcPotition();

public:
	Odometry(Encoder *, IMU *, VelocityCtrl *);
	void flip();
	double getX();
	double getY();
	double getTheta();
	double getDeltaTheta();
	void clearPotition();

};




#endif /* INC_ODOMETRY_HPP_ */
