/*
 * sdCard.cpp
 *
 *  Created on: 2022/10/16
 *      Author: under
 */

#include "sdCard.hpp"
#include "AQM0802.h"
#include <stdio.h>
#include "string.h"
#include <iostream>

char mon_cc[32];

void sdCard::openFile(const char *p_directory_name, const char *p_file_name)
{
	static int num;
	sprintf(dirpath_, "%s", p_directory_name);
	sprintf(filepath_, "%s", p_file_name);
	num++;

	f_mkdir(dirpath_);
	f_chdir(dirpath_);
	f_open(&fil_, filepath_, FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
	f_chdir("..");
}

void sdCard::clearBuff()
{
	for(int i = 0; i < BUFF_SIZE; i++){
		buffer_[i] = '\0';
	}
}

sdCard::sdCard() : buffer_{0}, filepath_{0}, dirpath_{0}{}

bool sdCard::init()
{
	if(mount() == 1){
		return true;

	}
	else{
		return false;
	}

}


bool sdCard::mount()
{
	bool ret = false;

	if(f_mount(&fs_, "", 1) == FR_OK) ret = true;
	else ret = false;

	return ret;
}

bool sdCard::unmout()
{
	bool ret = false;

	if(f_mount(NULL, "", 1) == FR_OK) ret = true;
	else ret = false;

	return ret;

}
void sdCard::userFopen(const char *p_directory_name, const char *p_file_name)
{
	openFile(p_directory_name, p_file_name);
}
void sdCard::userFclose()
{
	f_close(&fil_);	//ファイル閉じる
}
void sdCard::write(const char *p_folder_name, const char *p_file_name, uint16_t size, float *data, char state)
{
	//openFile(p_folder_name, p_file_name);

	sprintf(dirpath_, "%s", p_folder_name);
	sprintf(filepath_, "%s", p_file_name);

	strcpy(mon_cc, filepath_);

	if(state == OVER_WRITE){
		//f_chdir(dirpath_);
		//f_unlink(filepath_);	//	一回消す
		//f_chdir("..");
	}

	f_mkdir(dirpath_);
	f_chdir(dirpath_);

	FRESULT res;
	res = f_open(&fil_, filepath_, FA_CREATE_NEW| FA_READ | FA_WRITE);
	if(res == FR_EXIST){
		printf("soiya");
		//res = f_open(&fil_, filepath_, FA_CREATE_NEW| FA_READ | FA_WRITE);
	}
	else{
		printf("SOIYA");
	}
	f_chdir("..");

	for(short i = 0 ; i < size; i++){
		snprintf(buffer_, BUFF_SIZE, "%f\n", *(data + i));	//floatをstringに変換

		f_lseek(&fil_, f_size(&fil_));	//	ファイルの最後に移動
		f_write(&fil_, buffer_, strlen(buffer_), &bw_);	//	書き込む

		clearBuff();	//	書き込み用のバッファをクリア
	}

	f_close(&fil_);	//	ファイル閉じる

}
void sdCard::read(const char *p_folder_name, const char *p_file_name, uint16_t size, float *data)
{
	short i = 0;

	openFile(p_folder_name, p_file_name);

	while(f_gets(buffer_, sizeof(buffer_), &fil_) != NULL){
		sscanf(buffer_, "%f", data + i);
		i++;
		if(i >= size) i = size - 1;

	}

	clearBuff();

	f_close(&fil_);

}

