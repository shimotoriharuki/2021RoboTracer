/*
 * SystemIdentification.cpp
 *
 *  Created on: 2021/11/26
 *      Author: under
 */

#include "SystemIdentification.hpp"

float mon_msig;

SystemIdentification::SystemIdentification(sdCard *sd_card, Motor *motor) : msigArrayIdx_(0), inputVal_(0), processing_flag_(false)
{
	sd_card_ = sd_card;
	logger1_= new Logger2(sd_card_, LOG_SIZE);
	logger2_= new Logger2(sd_card_, LOG_SIZE);
	motor_ = motor;
}

void SystemIdentification::init()
{

}

void SystemIdentification::inOutputStore(float output)
{
	if(processing_flag_ == true){
		logger1_->storeLogs(output);
		logger2_->storeLogs(inputVal_);
	}

}

void SystemIdentification::inOutputSave()
{
	logger1_->saveLogs("SYSIDENT", "response");
	logger2_->saveLogs("SYSIDENT", "input");
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
	logger1_->clearLogs();
	logger1_->start();
	logger2_->clearLogs();
	logger2_->start();

	processing_flag_ = true;
}

void SystemIdentification::stop()
{
	logger1_->stop();
	logger2_->stop();

	processing_flag_ = false;
	msigArrayIdx_ = 0;
	motor_->setRatio(0, 0);
}

