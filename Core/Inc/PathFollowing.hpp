/*
 * PathFollowing.hpp
 *
 *  Created on: Aug 25, 2021
 *      Author: under
 */

#ifndef INC_PATHFOLLOWING_HPP_
#define INC_PATHFOLLOWING_HPP_

#include "path_following.h"

class PathFollowing{

private:
	double target_x_, target_y_, target_th_;

public:

	void init();
	void setGain(double, double, double);
	void setTargetPath();
	void getVelocitys(double &, double &);
	void flip();
};


#endif /* INC_PATHFOLLOWING_HPP_ */
