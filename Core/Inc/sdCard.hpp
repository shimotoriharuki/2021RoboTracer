/*
 * sdCard.hpp
 *
 *  Created on: 2022/10/16
 *      Author: under
 */

#ifndef INC_SDCARD_HPP_
#define INC_SDCARD_HPP_

#include "fatfs.h"

#define BUFF_SIZE 128
#define OVER_WRITE 0	//	上書き
#define ADD_WRITE 1		//	追加書き

class sdCard{
private:
	FATFS fs_;
	FIL fil_;
	char buffer_[BUFF_SIZE];
	UINT br_, bw_;

	char filepath_[256];
	char dirpath_[256];

	void openFile(const char *, const char *);
	void clearBuff();

public:

	sdCard();

	bool mount_();
	bool unmout_();
	void userFopen_(const char *, const char *);
	void userFclose_();
	void write_(const char *, const char * , short, float *, char);
	void read_(const char *, const char * , short, float *);
};

#endif /* INC_SDCARD_HPP_ */
