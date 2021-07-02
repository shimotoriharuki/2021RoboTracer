/*
 * AQM0802.h
 *
 *  Created on: Jun 2, 2021
 *      Author: under
 */

#ifndef INC_AQM0802_H_
#define INC_AQM0802_H_

#include "main.h"
#include <stdarg.h>

#ifdef __cplusplus
extern "C" {
#endif

void lcd_cmd(uint8_t);
void lcd_data(uint8_t);
void lcd_init(void);
void lcd_clear(void);
void lcd_locate(int,int);
void lcd_print(const char *);
short lcd_printf(const char *, ...);

#ifdef __cplusplus
}
#endif

#endif /* INC_AQM0802_H_ */
