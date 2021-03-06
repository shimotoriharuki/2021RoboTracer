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
FRESULT user_fopen(const char *, const char *);
FRESULT user_fclose(void);
FRESULT sd_write_float(short, float *, char);
FRESULT sd_write_long(short, long *, char);
FRESULT sd_read_float(short, float *);
FRESULT sd_write_double(short, double *, char);
FRESULT sd_read_double(short, double *);

FRESULT sd_write_array_float(const char *, const char * , short, float *, char); 	 //SDに書き込み
FRESULT sd_read_array_float(const char *, const char * , short, float *);		//SDから読み込み

FRESULT sd_write_array_double(const char *, const char * , short, double *, char); 	 //SDに書き込み
FRESULT sd_read_array_double(const char *, const char * , short, double *);		//SDから読み込み

FRESULT sd_write_array_int(const char *, const char * , short, int16_t *, char); 	 //SDに書き込み
FRESULT sd_read_array_int(const char *, const char * , short, int16_t *);		//SDから読み込み

void bufclear(void);

#ifdef __cplusplus
}
#endif


#endif /* INC_HAL_SDCARD_LIB_H_ */
