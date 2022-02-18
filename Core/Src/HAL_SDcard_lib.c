/*
 * HAL_SDcard_lib.c
 *
 *  Created on: Jun 4, 2021
 *      Author: under
 */

#include <stdio.h>
#include "fatfs.h"
#include "string.h"

#include "HAL_SDcard_lib.h"


#define BUFF_SIZE 128
#define OVER_WRITE 0	//	上書き
#define ADD_WRITE 1		//	追加書き

FATFS fs;	// ファイルシステムのやつ
FIL fil;	// ファイルのやつ
FRESULT fresult;
char buffer[BUFF_SIZE];
UINT br, bw;

FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;

char filepath[256];
char dirpath[256];

//************************************************************************/
//* 役割　：　fopenする
//* 引数　：　char, float *: short　: フォルダ名、ファイル名
//* 戻り値：　FRESULT:
//* 備考 : なし
//************************************************************************/
FRESULT user_fopen(const char *p_folder_name, const char *p_file_name){

	FRESULT ret = 0;

	create_path(p_folder_name, p_file_name);

	fopen_folder_and_file();	//書き込むファイルを選択

	return ret;
}

//************************************************************************/
//* 役割　：　fcloseする
//* 引数　：　void
//* 戻り値：　FRESULT:
//* 備考 : なし
//************************************************************************/
FRESULT user_fclose(){
	FRESULT ret = 0;

	f_close(&fil);	//ファイル閉じる

	return ret;
}


//************************************************************************/
//* 役割　：　SDに書き込む
//* 引数　：　short, float *, char : 変数の数、データのポインタ、追加か上書きか
//* 戻り値：　FRESULT:
//* 備考 : なし
//************************************************************************/
FRESULT sd_write_float(short size, float *data, char state){
	FRESULT ret = 0;

	for(short i = 0 ; i < size; i++){
		snprintf(buffer, BUFF_SIZE, "%f\n", *(data + i));	//floatをstringに変換

		if(state == ADD_WRITE){
			f_lseek(&fil, f_size(&fil));	//ファイルの最後に移動
		}
		else{
			f_lseek(&fil, 0);	//ファイルの最初に移動
		}

		f_write(&fil, buffer, strlen(buffer), &bw);	//書き込む

		bufclear();	//書き込み用のバッファをクリア
	}
	return ret;
}

//************************************************************************/
//* 役割　：　SDに書き込む
//* 引数　：　short, long*, char : 変数の数、データのポインタ、追加か上書きか
//* 戻り値：　FRESULT:
//* 備考 : なし
//************************************************************************/
FRESULT sd_write_long(short size, long *data, char state){
	FRESULT ret = 0;

	for(short i = 0 ; i < size; i++){
		snprintf(buffer, BUFF_SIZE, "%ld\n", *(data + i));	//floatをstringに変換

		if(state == ADD_WRITE){
			f_lseek(&fil, f_size(&fil));	//ファイルの最後に移動
		}
		else{
			f_lseek(&fil, 0);	//ファイルの最初に移動
		}

		f_write(&fil, buffer, strlen(buffer), &bw);	//書き込む

		bufclear();	//書き込み用のバッファをクリア
	}
	return ret;
}
//************************************************************************/
//* 役割　：　SDから読み込む
//* 引数　：　short, float *　:変数の数、データのポインタ
//* 戻り値：　FRESULT:
//* 備考 : なし
//************************************************************************/
FRESULT sd_read_float(short size, float *data){
	FRESULT ret = 0;
	short i = 0;

	while(f_gets(buffer, sizeof(buffer), &fil) != NULL){
		sscanf(buffer, "%f", data + i);
		i++;
		if(i >= size) i = size - 1;

	}

	bufclear();	//書き込み用のバッファをクリア

	return ret;
}

//************************************************************************/
//* 役割　：　SDに書き込む
//* 引数　：　short, double*, char : 変数の数、データのポインタ、追加か上書きか
//* 戻り値：　FRESULT:
//* 備考 : なし
//************************************************************************/
FRESULT sd_write_double(short size, double *data, char state){
	FRESULT ret = 0;

	for(short i = 0 ; i < size; i++){
		snprintf(buffer, BUFF_SIZE, "%lf\n", *(data + i));	//floatをstringに変換

		if(state == ADD_WRITE){
			f_lseek(&fil, f_size(&fil));	//ファイルの最後に移動
		}
		else{
			f_lseek(&fil, 0);	//ファイルの最初に移動
		}

		f_write(&fil, buffer, strlen(buffer), &bw);	//書き込む

		bufclear();	//書き込み用のバッファをクリア
	}
	return ret;
}
//************************************************************************/
//* 役割　：　SDから読み込む
//* 引数　：　short, float *　:変数の数、データのポインタ
//* 戻り値：　FRESULT:
//* 備考 : なし
//************************************************************************/
FRESULT sd_read_double(short size, double *data){
	FRESULT ret = 0;
	short i = 0;

	while(f_gets(buffer, sizeof(buffer), &fil) != NULL){
		sscanf(buffer, "%lf", data + i);
		i++;
		if(i >= size) i = size - 1;

	}

	bufclear();	//書き込み用のバッファをクリア

	return ret;
}
//************************************************************************/
//* 役割　：　SDに書き込む
//* 引数　：　char *, char *, short, float *, char: フォルダ名、ファイル名、変数の数、データのポインタ、追加か上書きか
//* 戻り値：　FRESULT:
//* 備考 : なし
//************************************************************************/
FRESULT sd_write_array_float(const char *p_folder_name, const char *p_file_name, short size, float *data, char state){
	FRESULT ret = 0;

	create_path(p_folder_name, p_file_name);

	if(state == OVER_WRITE){
		f_chdir(dirpath);
		f_unlink(filepath);	//	一回消す
		f_chdir("..");
	}

	fopen_folder_and_file();	//	書き込むファイルを選択

	for(short i = 0 ; i < size; i++){
		snprintf(buffer, BUFF_SIZE, "%f\n", *(data + i));	//floatをstringに変換

		f_lseek(&fil, f_size(&fil));	//	ファイルの最後に移動
		f_write(&fil, buffer, strlen(buffer), &bw);	//	書き込む

		bufclear();	//	書き込み用のバッファをクリア
	}

	f_close(&fil);	//	ファイル閉じる

	return ret;
}

//************************************************************************/
//* 役割　：　SDから読み込む
//* 引数　：　char *, char *, short, float *: フォルダ名、ファイル名、変数の数、データのポインタ
//* 戻り値：　FRESULT:
//* 備考 : なし
//************************************************************************/
FRESULT sd_read_array_float(const char *p_folder_name, const char *p_file_name, short size, float *data){
	FRESULT ret = 0;
	short i = 0;

	create_path(p_folder_name, p_file_name);
	fopen_folder_and_file();	//書き込むファイルを選択

	while(f_gets(buffer, sizeof(buffer), &fil) != NULL){
		sscanf(buffer, "%f", data + i);
		i++;
		if(i >= size) i = size - 1;

	}

	bufclear();	//書き込み用のバッファをクリア

	f_close(&fil);	//ファイル閉じる

	return ret;
}

//************************************************************************/
//* 役割　：　SDに書き込む
//* 引数　：　char *, char *, short, double *, char: フォルダ名、ファイル名、変数の数、データのポインタ、追加か上書きか
//* 戻り値：　FRESULT:
//* 備考 : なし
//************************************************************************/
FRESULT sd_write_array_double(const char *p_folder_name, const char *p_file_name, short size, double *data, char state){
	FRESULT ret = 0;

	create_path(p_folder_name, p_file_name);

	if(state == OVER_WRITE){
		f_chdir(dirpath);
		f_unlink(filepath);	//	一回消す
		f_chdir("..");
	}

	fopen_folder_and_file();	//	書き込むファイルを選択

	for(short i = 0 ; i < size; i++){
		snprintf(buffer, BUFF_SIZE, "%lf\n", *(data + i));	//doubleをstringに変換

		f_lseek(&fil, f_size(&fil));	//	ファイルの最後に移動
		f_write(&fil, buffer, strlen(buffer), &bw);	//	書き込む

		bufclear();	//	書き込み用のバッファをクリア
	}

	f_close(&fil);	//	ファイル閉じる

	return ret;
}

//************************************************************************/
//* 役割　：　SDから読み込む
//* 引数　：　char *, char *, short, double *: フォルダ名、ファイル名、変数の数、データのポインタ
//* 戻り値：　FRESULT:
//* 備考 : なし
//************************************************************************/
FRESULT sd_read_array_double(const char *p_folder_name, const char *p_file_name, short size, double *data){
	FRESULT ret = 0;
	short i = 0;

	create_path(p_folder_name, p_file_name);
	fopen_folder_and_file();	//書き込むファイルを選択

	while(f_gets(buffer, sizeof(buffer), &fil) != NULL){
		sscanf(buffer, "%lf", data + i);
		i++;
		if(i >= size) i = size - 1;

	}

	bufclear();	//書き込み用のバッファをクリア

	f_close(&fil);	//ファイル閉じる

	return ret;
}

//************************************************************************/
//* 役割　：　SDに書き込む
//* 引数　：　char *, char *, short, float *, char: フォルダ名、ファイル名、変数の数、データのポインタ、追加か上書きか
//* 戻り値：　FRESULT:
//* 備考 : なし
//************************************************************************/
FRESULT sd_write_array_int(const char *p_folder_name, const char *p_file_name, short size, int16_t *data, char state){
	FRESULT ret = 0;

	create_path(p_folder_name, p_file_name);

	if(state == OVER_WRITE){
		f_chdir(dirpath);
		f_unlink(filepath);	//一回消す
		f_chdir("..");
	}

	fopen_folder_and_file();	//書き込むファイルを選択

	for(short i = 0 ; i < size; i++){
		snprintf(buffer, BUFF_SIZE, "%d\n", *(data + i));	//floatをstringに変換
/*
		if(state == ADD_WRITE){
			f_lseek(&fil, f_size(&fil));	//ファイルの最後に移動
		}
		else{
			f_lseek(&fil, 0);	//ファイルの最初に移動
		}
*/
		f_lseek(&fil, f_size(&fil));	//ファイルの最後に移動
		f_write(&fil, buffer, strlen(buffer), &bw);	//書き込む

		bufclear();	//書き込み用のバッファをクリア
	}

	f_close(&fil);	//ファイル閉じる

	return ret;
}

//************************************************************************/
//* 役割　：　SDから読み込む
//* 引数　：　char *, char *, short, float *: フォルダ名、ファイル名、変数の数、データのポインタ
//* 戻り値：　FRESULT:
//* 備考 : なし
//************************************************************************/
FRESULT sd_read_array_int(const char  *p_folder_name, const char *p_file_name, short size, int16_t *data){
	FRESULT ret = 0;
	short i = 0;

	create_path(p_folder_name, p_file_name);
	fopen_folder_and_file();	//書き込むファイルを選択

	while(f_gets(buffer, sizeof(buffer), &fil) != NULL){
		sscanf(buffer, "%d", data + i);
		i++;
		if(i >= size) i = size - 1;

	}

	bufclear();	//書き込み用のバッファをクリア

	f_close(&fil);	//ファイル閉じる

	return ret;
}

//************************************************************************/
//* 役割　：　SDカードをマウント
//* 引数　：　void:
//* 戻り値：　FRESULT:
//* 備考 : なし
//************************************************************************/
FRESULT sd_mount(){
	FRESULT ret = 0;

	if(f_mount(&fs, "", 1) == FR_OK) ret = 1;
	else ret = 0;

	return ret;
}

//************************************************************************/
//* 役割　：　SDカードをアンマウント
//* 引数　：　void:
//* 戻り値：　FRESULT:
//* 備考 : なし
//************************************************************************/
FRESULT sd_unmount(){
	FRESULT ret = 0;

	if(f_mount(NULL, "", 1) == FR_OK) ret = 1;
	else ret = 0;

	return ret;
}

//************************************************************************/
//* 役割　：　操作するパスの文字列を作る
//* 引数　：　char, char: フォルダ名, ファイル名
//* 戻り値：　void:
//* 備考 : なし
//************************************************************************/
void create_path(char const *p_folder_name, char const *p_file_name){

	sprintf(dirpath, "%s", p_folder_name);

	sprintf(filepath, "%s", p_file_name);

}

//************************************************************************/
//* 役割　：　操作するファイルを選択する_
//* 引数　：　char: ファイル選択
//* 戻り値：　char: 状態チェック	0(マウント失敗) or 1(成功)
//* 備考 : なし
//************************************************************************/
void fopen_folder_and_file(){	//mkdir

	f_mkdir(dirpath);

	f_chdir(dirpath);

	f_open(&fil, filepath, FA_OPEN_ALWAYS | FA_READ | FA_WRITE);

	f_chdir("..");


}

//************************************************************************/
//* 役割　：　バッファをクリア
//* 引数　：　void:
//* 戻り値：　void:
//* 備考 : なし
//************************************************************************/
void bufclear(void){
	for(int i = 0; i < BUFF_SIZE; i++){
		buffer[i] = '\0';
	}
}


