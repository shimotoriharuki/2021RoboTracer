/*
 * PathFollowing.hpp
 *
 *  Created on: Aug 25, 2021
 *      Author: under
 */

#ifndef INC_PATHFOLLOWING_HPP_
#define INC_PATHFOLLOWING_HPP_

#include "rtwtypes.h"
#include "Logger.hpp"

class PathFollowing{

private:
	bool execute_flag_;
	double x_tar_, y_tar_, th_tar_;

	double log_distances_[LOG_DATA_SIZE_DIS];
	double log_delta_thetas_[LOG_DATA_SIZE_DIS];
	uint16_t ref_num;
	//double target_x_, target_y_, target_theta_;

	void calcXY(const double, const double, double &, double &);
	bool isNear(const double, const double, const double);

public:
	PathFollowing();
	void init();
	void setGain(double, double, double);
	double getKxVal();
	double getKyVal();
	double getKtVal();
	bool isTargetNear();
	void setTargetPathSingle(double, double, double);
	void setTargetPathMulti();
	void targetUpdate();
	void setCurrentPath(double, double, double);
	//void getTargetVelocitys(double &, double &);
	void flip();
	double getV();
	double getW();
	void start();
	void stop();
};


#endif /* INC_PATHFOLLOWING_HPP_ */
