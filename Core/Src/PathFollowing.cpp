/*
 * PathFollowing.cpp
 *
 *  Created on: Aug 25, 2021
 *      Author: under
 */

#include "PathFollowing.hpp"
#include "path_following.h"
#include "HAL_SDcard_lib.h"

uint16_t mon_ref_num;
double mon_x, mon_y, mon_th;
double mon_log_dis, mon_log_th;

PathFollowing::PathFollowing() : execute_flag_(false), x_tar_(0), y_tar_(0), th_tar_(0), ref_num(0)
{
	rtParam.kx = 0;
	rtParam.ky = 0;
	rtParam.kt = 0;
	rtU.target_x = 0;
	rtU.target_y = 0;
	rtU.th = 0;
	rtU.x = 0;
	rtU.y = 0;
	rtU.th_cur = 0;
	rtY.V_tar = 0;
	rtY.tar = 0;

}

void PathFollowing::calcXY(const double distance, const double theta, double &x, double &y)
{
	x = x + distance * cos(theta);
	y = y + distance * sin(theta);

}

bool PathFollowing::isNear(const double src_data, const double target_data, const double margin)
{

	if(target_data - margin < src_data && src_data < target_data + margin){
		return true;
	}
	else{
		return false;
	}

}

void PathFollowing::init()
{
	path_following_initialize();
}

void PathFollowing::setGain(double kx, double ky, double kt)
{
	rtParam.kx = kx;
	rtParam.ky = ky;
	rtParam.kt = kt;
}

double PathFollowing::getKxVal()
{
	return rtParam.kx;
}

double PathFollowing::getKyVal()
{
	return rtParam.ky;
}

double PathFollowing::getKtVal()
{
	return rtParam.kt;
}

void PathFollowing::setTargetPathSingle(double x, double y, double th)
{
	rtU.target_x = x;
	rtU.target_y = y;
	rtU.th = th;
}

void PathFollowing::setTargetPathMulti()
{
	sd_read_array_double("Pos", "d_th.txt", LOG_DATA_SIZE_DIS, log_delta_thetas_);
	sd_read_array_double("Pos", "d_dis.txt", LOG_DATA_SIZE_DIS, log_distances_);

	mon_log_dis = log_distances_[1];
	mon_log_th = log_delta_thetas_[1];
}

void PathFollowing::targetUpdate()
{
	if(execute_flag_ == true){
		//if(isNear(rtU.x, x_tar_, 10) == true && isNear(rtU.y, y_tar_, 30) == true && isNear(rtU.th_cur, th_tar_, 1.100) == true){
		if(isNear(rtU.x, x_tar_, 10) == true && isNear(rtU.y, y_tar_, 10) == true && isNear(rtU.th_cur, th_tar_, 3) == true){
			ref_num++;
			x_tar_ = x_tar_ + log_distances_[ref_num] * cos(th_tar_ + log_delta_thetas_[ref_num] / 2);
			y_tar_ = y_tar_ + log_distances_[ref_num] * sin(th_tar_ + log_delta_thetas_[ref_num] / 2);
			th_tar_ = th_tar_ + log_delta_thetas_[ref_num];
		}
		if(ref_num >= LOG_DATA_SIZE_DIS) ref_num = LOG_DATA_SIZE_DIS;

	}

	mon_ref_num = ref_num;
	mon_x = x_tar_;
	mon_y = y_tar_;
	mon_th = th_tar_;

}


void PathFollowing::setCurrentPath(double x, double y, double th)
{
	rtU.x= x;
	rtU.y = y;
	rtU.th_cur = th;
}

void PathFollowing::getTargetVelocitys(double &v, double &omega)
{
	v = rtY.V_tar;
	omega = rtY.tar;

}
void PathFollowing::flip()
{
	if(execute_flag_ == true){
		path_following_step();
	}
}

void PathFollowing::start()
{
	execute_flag_ = true;
}

void PathFollowing::stop()
{
	execute_flag_ = false;
	ref_num = 0;
	x_tar_ = 0;
	y_tar_ = 0;
	th_tar_ = 0;
}

