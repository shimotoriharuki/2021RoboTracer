/*
 * SystemIdentification.cpp
 *
 *  Created on: 2021/11/26
 *      Author: under
 */

#include "SystemIdentification.hpp"

float mon_msig;

SystemIdentification::SystemIdentification(Logger *logger, Motor *motor) : msigArrayIdx_(0), inputVal_(0), processing_flag_(false)
{
	logger_ = logger;
	motor_ = motor;
	//msigItr_ = msigArray_.begin();
}

void SystemIdentification::init()
{

}

void SystemIdentification::inOutputStore(float output)
{
	if(processing_flag_ == true){
		//logger_->storeLog(output);
		//logger_->storeLog2(inputVal_);
	}

}

void SystemIdentification::inOutputSave()
{
	logger_->saveLogs("sysident", "MSIGRES.txt");
	logger_->saveLogs2("sysident", "INPUT.txt");
}

void SystemIdentification::updateMsig()
{
	if(processing_flag_ == true){
		inputVal_ = inputRatio_ * msigArray_[msigArrayIdx_];
		msigArrayIdx_++;
		mon_msig = inputVal_;

		if(msigArrayIdx_ >= MSIG_SIZE) msigArrayIdx_ = MSIG_SIZE;

		motor_->setRatio(inputVal_, inputVal_);

	}

}
void SystemIdentification::setInputRatio(float ratio)
{
	inputRatio_ = ratio;
}

void SystemIdentification::start()
{
	//logger_->resetLogs();
	logger_->start();
	processing_flag_ = true;
}

void SystemIdentification::stop()
{
	logger_->stop();
	processing_flag_ = false;
	msigArrayIdx_ = 0;
	motor_->setRatio(0, 0);
}

