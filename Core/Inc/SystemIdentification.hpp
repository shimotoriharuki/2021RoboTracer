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

class SystemIdentification
{

private:
	Logger *logger_;
	std::vector<int8_t> msigArray_{1, 1, -1, 1, -1, 1};
	std::vector<int8_t>::iterator msigItr_;
	int16_t inputVal_, inputRatio_;
	bool processing_flag_;

public:

	SystemIdentification(Logger *);
	void init();
	void outputStore(float);
	void outputSave();
	void updateMsig();
	void setInputRatio(float);
	void start();
	void stop();

};



#endif /* INC_SYSTEMIDENTIFICATION_HPP_ */
