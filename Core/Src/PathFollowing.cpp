/*
 * PathFollowing.cpp
 *
 *  Created on: Aug 25, 2021
 *      Author: under
 */

#include "PathFollowing.hpp"

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

void PathFollowing::setTargetPath()
{
	target_x_ = rtU.target_x;
	target_y_ = rtU.target_y;
	target_th_ = rtU.th;

}

void getVelocitys(double &v, double &omega)
{
	v = rtY.V_tar;
	omega = rtY.tar;

}
void PathFollowing::flip()
{
	path_following_step();
}

