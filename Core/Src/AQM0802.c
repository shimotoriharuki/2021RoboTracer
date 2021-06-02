/*
 * AQM0802.c
 *
 *  Created on: Jun 2, 2021
 *      Author: under
 */

#include "AQM0802.h"
#include <stdio.h>

extern I2C_HandleTypeDef hi2c1;

#define SLAVEADRESS (0x3E<<1)

void lcd_cmd(uint8_t cmd) {
	uint8_t Txcmd[2] = { 0x00 , cmd };
	HAL_I2C_Master_Transmit(&hi2c1,SLAVEADRESS,Txcmd,2,100);
}

void lcd_data(uint8_t data) {
	uint8_t Txdata[2] = { 0x40 , data };
	HAL_I2C_Master_Transmit(&hi2c1,SLAVEADRESS,Txdata,2,100);
}

void lcd_init(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);//lcd_reset_pin
	HAL_Delay(40);
	lcd_cmd(0x38);
	HAL_Delay(1);
	lcd_cmd(0x39);
	HAL_Delay(1);
	lcd_cmd(0x14);
	HAL_Delay(1);
	lcd_cmd(0x70);
	HAL_Delay(1);
	lcd_cmd(0x56);
	HAL_Delay(1);
	lcd_cmd(0x6C);
	HAL_Delay(200);
	lcd_cmd(0x38);
	HAL_Delay(1);
	lcd_cmd(0x0C);
	HAL_Delay(1);
	lcd_cmd(0x01);
	HAL_Delay(1);
}

void lcd_clear(){
	lcd_cmd(0x01);
	HAL_Delay(1);
	lcd_cmd(0x02);
	HAL_Delay(1);
}

void lcd_locate(int x, int y) {
	lcd_cmd(0x80 + y*0x40 + x);
}

void lcd_print(const char *str) {
	while(*str != '\0')
	{
			lcd_data(*str);
			str++;
	}
}

short lcd_printf(const char *format, ...) {
	va_list argptr;
	char lcd_bff[20];
	short ret;

  va_start(argptr, format);
  ret = vsprintf(lcd_bff, format, argptr);
	va_end(argptr);

	if(ret>0) {
		lcd_print(lcd_bff);
	}

	return ret;
}
