/*
 * HAL_SDcard_lib.h
 *
 *  Created on: Jun 4, 2021
 *      Author: under
 */

#ifndef INC_HAL_SDCARD_LIB_H_
#define INC_HAL_SDCARD_LIB_H_

#include "fatfs.h"

#ifdef __cplusplus
extern "C" {
#endif

void fopen_folder_and_file(void);
void create_path(char const *, char const *);

FRESULT sd_mount(void);   					//SDをマウント
FRESULT sd_unmount(void);					  //SDをアンマウント
FRESULT user_fopen(char *, char *);
FRESULT user_fclose(void);
FRESULT sd_write(short, float *, char);
FRESULT sd_read(short, float *);

FRESULT sd_write_array(char *, char * , short, float *, char); 	 //SDに書き込み
FRESULT sd_read_array(char *, char * , short, float *);		//SDから読み込み

FRESULT sd_write_array_int(char const *, char const * , short, int *, char); 	 //SDに書き込み
FRESULT sd_read_array_int(char const *, char const * , short, int *);		//SDから読み込み

void bufclear(void);

#ifdef __cplusplus
}
#endif


#endif /* INC_HAL_SDCARD_LIB_H_ */
