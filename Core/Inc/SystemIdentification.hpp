/*
 * SystemIdentification.hpp
 *
 *  Created on: 2021/11/26
 *      Author: under
 */

#ifndef INC_SYSTEMIDENTIFICATION_HPP_
#define INC_SYSTEMIDENTIFICATION_HPP_

#include <vector>
#include "Logger2.hpp"
#include "Motor.hpp"
#include "sdCard.hpp"

#define MSIG_SIZE 250
#define LOG_SIZE 8500

class SystemIdentification
{

private:
	Logger2 *logger1_;
	Logger2 *logger2_;
	sdCard *sd_card_;
	Motor *motor_;

	int16_t msigArray_[MSIG_SIZE] = {-1,-1,-1,-1,-1,-1,1,1,1,1,1,1,1,-1,1,-1,1,-1,1,-1,-1,1,1,-1,-1,1,1,1,-1,1,1,1,-1,1,-1,-1,1,-1,1,1,-1,-1,-1,1,1,-1,1,1,1,1,-1,1,1,-1,1,-1,1,1,-1,1,1,-1,-1,1,-1,-1,1,-1,-1,-1,1,1,1,-1,-1,-1,-1,1,-1,1,1,1,1,1,-1,-1,1,-1,1,-1,1,1,1,-1,-1,1,1,-1,1,-1};

	uint16_t msigArrayIdx_;
	float inputVal_, inputRatio_;
	bool processing_flag_;

public:

	SystemIdentification(sdCard *, Motor *);
	void init();
	void inOutputStore(float);
	void inOutputSave();
	void updateMsig();
	void setInputRatio(float);
	void start();
	void stop();

};



#endif /* INC_SYSTEMIDENTIFICATION_HPP_ */
