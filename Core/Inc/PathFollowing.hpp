/*
 * PathFollowing.hpp
 *
 *  Created on: Aug 25, 2021
 *      Author: under
 */

#ifndef INC_PATHFOLLOWING_HPP_
#define INC_PATHFOLLOWING_HPP_

#include "rtwtypes.h"

class PathFollowing{

private:
	bool execute_flag_;

public:
	PathFollowing();
	void init();
	void setGain(double, double, double);
	void setTargetPath(double, double, double);
	void setCurrentPath(double, double, double);
	void getTargetVelocitys(double &, double &);
	void flip();
	void start();
	void stop();
};


#endif /* INC_PATHFOLLOWING_HPP_ */
