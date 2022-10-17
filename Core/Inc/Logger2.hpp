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
	uint16_t log_idx_;
	sdCard *sd_card_;
	float buff_[];

public:
	Logger2(sdCard *, uint16_t);

	void storeLogs(float);
	void saveLogs();
	void clearLogs();

};



#endif /* INC_LOGGER2_HPP_ */
