/*
 * Logger2.hpp
 *
 *  Created on: 2022/10/16
 *      Author: Haruki Shimotori
 */

#ifndef INC_LOGGER2_HPP_
#define INC_LOGGER2_HPP_

#include "sdCard.hpp"

class Logger2{

private:
	sdCard *sd_card_;
	uint16_t log_idx_;
	float *logs_;
	bool recording_flag_;
	uint16_t max_log_size_;

public:
	Logger2(sdCard *, uint16_t);

	void storeLogs(float);
	void saveLogs(const char *, const char *);
	void clearLogs();

};



#endif /* INC_LOGGER2_HPP_ */
