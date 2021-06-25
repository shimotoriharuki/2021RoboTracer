/*
 * ICM20648.c
 *
 *  Created on: 2021/06/25
 *      Author: under
 */
#include "ICM_20648.h"

volatile int16_t xa, ya, za; // 加速度(16bitデータ)
volatile int16_t xg, yg, zg;	// 角加速度(16bitデータ)

uint8_t read_byte( uint8_t reg ) {
	uint8_t ret,val;

	ret = reg | 0x80;
	CS_RESET;
	HAL_SPI_Transmit(&hspi2,&ret,1,100);
	HAL_SPI_Receive(&hspi2,&val,1,100);
	CS_SET;

	return val;
}

void write_byte( uint8_t reg, uint8_t val )  {
	uint8_t ret;

	ret = reg & 0x7F;
	CS_RESET;
	HAL_SPI_Transmit(&hspi2,&ret,1,100);
	HAL_SPI_Transmit(&hspi2,&val,1,100);
	CS_SET;
}

uint16_t IMU_init() {
	uint8_t who_am_i;
	uint16_t ret = 0;

	who_am_i = read_byte(0x00);	//IMU動作確認　0xE0が送られてくればおｋ
	if ( who_am_i == 0xE0 ) {
		ret = 1;
		write_byte(0x06,0x01);	//PWR_MGMT_1	スリープﾓｰﾄﾞ解除
		write_byte(0x03,0x10);	//USER_CTRL	諸々機能無効　SPIonly
		write_byte(0x7F,0x20);	//USER_BANK2
		write_byte(0x01,0x06);	//レンジ±2000dps
		//2:1 GYRO_FS_SEL[1:0] 00:±250	01:±500 10:±1000 11:±2000
		write_byte(0x14,0x06);	//レンジ±16g
		//2:1 ACCEL_FS_SEL[1:0] 00:±2	01:±4 10:±8 11:±16
		write_byte(0x7F,0x00);	//USER_BANK0
	}
	return ret;
}

void read_gyro_data() {
	xg = ((uint16_t)read_byte(0x33) << 8) | ((uint16_t)read_byte(0x34));
	yg = ((uint16_t)read_byte(0x35) << 8) | ((uint16_t)read_byte(0x36));
	zg = ((uint16_t)read_byte(0x37) << 8) | ((uint16_t)read_byte(0x38));
}

void read_accel_data() {
	xa = ((uint16_t)read_byte(0x2D) << 8) | ((uint16_t)read_byte(0x2E));
	ya = ((uint16_t)read_byte(0x2F) << 8) | ((uint16_t)read_byte(0x30));
	za = ((uint16_t)read_byte(0x31) << 8) | ((uint16_t)read_byte(0x32));
}



