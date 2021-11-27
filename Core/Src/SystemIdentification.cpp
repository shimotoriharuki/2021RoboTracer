/*
 * SystemIdentification.cpp
 *
 *  Created on: 2021/11/26
 *      Author: under
 */

#include "SystemIdentification.hpp"

float mon_msig;

SystemIdentification::SystemIdentification(Logger *logger) : inputVal_(0), processing_flag_(false)
{
	logger_ = logger;
	msigItr_ = msigArray_.begin();
}

void SystemIdentification::init()
{

}

void SystemIdentification::outputStore(float output)
{
	if(processing_flag_ == true){
		logger_->storeLog(output);
	}

}

void SystemIdentification::outputSave()
{
	logger_->saveLogs("sysident", "angvel.txt");
}

void SystemIdentification::updateMsig()
{
	if(processing_flag_ == true){
		inputVal_ = inputRatio_ * *msigItr_;
		msigItr_++;
		mon_msig = inputVal_;

		if(msigItr_ == msigArray_.end()) msigItr_ = msigArray_.begin();
	}

}
void SystemIdentification::setInputRatio(float ratio)
{
	inputRatio_ = ratio;
}

void SystemIdentification::start()
{
	logger_->start();
	processing_flag_ = true;
}

void SystemIdentification::stop()
{
	logger_->stop();
	processing_flag_ = false;
	msigItr_ = msigArray_.begin();
}

