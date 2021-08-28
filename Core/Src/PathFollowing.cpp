/*
 * PathFollowing.cpp
 *
 *  Created on: Aug 25, 2021
 *      Author: under
 */

#include "PathFollowing.hpp"
#include "path_following.h"

PathFollowing::PathFollowing()
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

void PathFollowing::setTargetPath(double x, double y, double th)
{
	rtU.target_x = x;
	rtU.target_y = y;
	rtU.th = th;
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
	path_following_step();
}

