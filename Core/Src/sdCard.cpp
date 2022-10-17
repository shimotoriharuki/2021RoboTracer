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

void sdCard::openFile(const char *p_directory_name, const char *p_file_name)
{
	sprintf(dirpath_, "%s", p_directory_name);
	sprintf(filepath_, "%s", p_file_name);

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

sdCard::sdCard()
{
	if(mount_() == 1){
	  lcd_clear();
	  lcd_locate(0,0);
	  lcd_printf("SD mount");
	  lcd_locate(0,1);
	  lcd_printf("Success");
	  HAL_Delay(500);

	}
	else{
	  lcd_clear();
	  lcd_locate(0,0);
	  lcd_printf("SD mount");
	  lcd_locate(0,1);
	  lcd_printf("Fail");
	  HAL_Delay(1000);
	}
}

bool sdCard::mount_()
{
	bool ret = false;

	if(f_mount(&fs_, "", 1) == FR_OK) ret = true;
	else ret = false;

	return ret;
}

bool sdCard::unmout_()
{
	bool ret = false;

	if(f_mount(NULL, "", 1) == FR_OK) ret = true;
	else ret = false;

	return ret;

}
void sdCard::userFopen_(const char *p_directory_name, const char *p_file_name)
{
	openFile(p_directory_name, p_file_name);
}
void sdCard::userFclose_()
{

	f_close(&fil_);	//ファイル閉じる


}
void sdCard::write_(const char *p_folder_name, const char *p_file_name, uint16_t size, float *data, char state)
{
	openFile(p_folder_name, p_file_name);

	if(state == OVER_WRITE){
		f_chdir(dirpath_);
		f_unlink(filepath_);	//	一回消す
		f_chdir("..");
	}


	for(short i = 0 ; i < size; i++){
		snprintf(buffer_, BUFF_SIZE, "%f\n", *(data + i));	//floatをstringに変換

		f_lseek(&fil_, f_size(&fil_));	//	ファイルの最後に移動
		f_write(&fil_, buffer_, strlen(buffer_), &bw_);	//	書き込む

		clearBuff();	//	書き込み用のバッファをクリア
	}

	f_close(&fil_);	//	ファイル閉じる

}
void sdCard::read_(const char *p_folder_name, const char *p_file_name, uint16_t size, float *data)
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



