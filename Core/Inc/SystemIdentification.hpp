/*
 * SystemIdentification.hpp
 *
 *  Created on: 2021/11/26
 *      Author: under
 */

#ifndef INC_SYSTEMIDENTIFICATION_HPP_
#define INC_SYSTEMIDENTIFICATION_HPP_

#include <vector>
#include "Logger.hpp"
#include "Motor.hpp"

#define MSIG_SIZE 250

class SystemIdentification
{

private:
	Logger *logger_;
	Motor *motor_;
	int16_t msigArray_[MSIG_SIZE] = {-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,-1,1,-1,1,-1,1,-1,-1,1,1,-1,-1,1,1,1,-1,1,1,1,-1,1,-1,-1,1,-1,1,1,-1,-1,-1,1,1,-1,1,1,1,1,-1,1,1,-1,1,-1,1,1,-1,1,1,-1,-1,1,-1,-1,1,-1,-1,-1,1,1,1,-1,-1,-1,-1,1,-1,1,1,1,1,1,-1,-1,1,-1,1,-1,1,1,1,-1,-1,1,1,-1,1,-1};

	uint16_t msigArrayIdx_;
	float inputVal_, inputRatio_;
	bool processing_flag_;

public:

	SystemIdentification(Logger *, Motor *);
	void init();
	void inOutputStore(float);
	void inOutputSave();
	void updateMsig();
	void setInputRatio(float);
	void start();
	void stop();

};



#endif /* INC_SYSTEMIDENTIFICATION_HPP_ */
