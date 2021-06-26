/*
 * INA260.c
 *
 *  Created on: Jun 26, 2021
 *      Author: under
 */

#include "INA260.h"

#define SLAVEADRESS1 (0x44<<1)

unsigned short INA260_read(uint8_t pointer_byte) {
	uint8_t Rxdata[2];
	unsigned short val;
	HAL_I2C_Master_Transmit(&hi2c2,SLAVEADRESS1,&pointer_byte,1,100);
	HAL_I2C_Master_Receive(&hi2c2,SLAVEADRESS1,Rxdata,2,100);
	val = ((unsigned short)Rxdata[0] << 8) | (unsigned short)Rxdata[1];
	return val;
}

void INA260_write(uint8_t pointer_byte , uint8_t data_msbyte , uint8_t data_lsbyte) {
	uint8_t Txcmd[3] = { pointer_byte , data_msbyte , data_lsbyte };
	HAL_I2C_Master_Transmit(&hi2c2,SLAVEADRESS1,Txcmd,3,100);
}

void setConfig(uint8_t msbyte , uint8_t lsbyte) {
	INA260_write(0x00 , msbyte , lsbyte);
}

void INA260_init() {
	setConfig(0x00,0xDF);//AVG=1,BusVoltageConversionTime=588u,ShuntCurrentConversionTime=588u,mode=BusVoltageContinuous
}


