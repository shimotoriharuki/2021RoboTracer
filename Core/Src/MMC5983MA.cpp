/*
 * MMC5983MA.cpp
 *
 *  Created on: 2023/04/23
 *      Author: Haruki SHIMOTORI
 */

#include <MMC5983MA.hpp>
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_def.h>
#include <stm32f4xx_hal_i2c.h>
#include <sys/_stdint.h>

#define MAG_SLAVEADRESS 0x60
#define WRITE 0
#define READ 1

I2C_HandleTypeDef hi2c1;

uint8_t mon_data[2];
uint16_t mon_xout, mon_yout;
int16_t mon_xout_calib, mon_yout_calib;
HAL_StatusTypeDef mon_ret;

int16_t mon_offset_x, mon_offset_y, mon_offset_z;


//------private-------//
void MMC5983MA::send(uint8_t *cmd, uint16_t size)
{
	HAL_I2C_Master_Transmit(&hi2c1, MAG_SLAVEADRESS, cmd, size, 100);

}

void MMC5983MA::receive(uint8_t *received_data, uint16_t size)
{
	HAL_I2C_Master_Receive(&hi2c1, MAG_SLAVEADRESS, received_data, size, 100);
}



//------public--------//

MMC5983MA::MMC5983MA() : enable_flag_(false)
{
	offset_.x = 0;
	offset_.y = 0;
	offset_.z = 0;

	gauss_.x = 0;
	gauss_.y = 0;
	gauss_.z = 0;

}

void MMC5983MA::write(uint8_t address, uint8_t *write_data, uint16_t write_data_size)
{
	uint8_t cmd[write_data_size + 1];
	cmd[0] = address;

	for(uint16_t i = 0; i < write_data_size; i++){
		cmd[i + 1] = write_data[i];
	}

	send(cmd, write_data_size + 1);
	HAL_Delay(1);
}

void MMC5983MA::read(uint8_t address, uint8_t *read_data, uint16_t read_data_size)
{
	send(&address, 1);
	HAL_Delay(1);
	receive(read_data, read_data_size);
	HAL_Delay(1);
}

void MMC5983MA::measurementStartOnce()
{
	uint8_t write_data;

	write_data = 0x01;
	write(INTERNAL_CONTROL0_ADDRESS, &write_data, 1); //Measument start



}
void MMC5983MA::measurementStartContinuous()
{
	enable_flag_ = true;

	uint8_t write_data;

	write_data = 0x21; //0010 0001
	write(INTERNAL_CONTROL0_ADDRESS, &write_data, 1);

	write_data = 0xBD; //1011 1101
	write(INTERNAL_CONTROL2_ADDRESS, &write_data, 1);
}

void MMC5983MA::measurementStop()
{

}

void MMC5983MA::calibration()
{
	uint8_t read_x_out[2], read_y_out[2], read_z_out[2];

	//Get values when device is set mode;
	uint8_t set_cmd = 0x08;
	write(INTERNAL_CONTROL0_ADDRESS, &set_cmd, 1); //set

	measurementStartOnce();
	read(X_OUT0_ADDRESS, read_x_out, 2);
	read(X_OUT0_ADDRESS, read_y_out, 2);
	read(X_OUT0_ADDRESS, read_z_out, 2);

	uint16_t x_out_set, y_out_set, z_out_set;
	x_out_set = (read_x_out[0] << 8) | (read_x_out[1]);
	y_out_set = (read_y_out[0] << 8) | (read_y_out[1]);
	z_out_set = (read_z_out[0] << 8) | (read_z_out[1]);

	//Get values when device is reset mode;
	uint8_t reset_cmd = 0x10;
	write(INTERNAL_CONTROL0_ADDRESS, &reset_cmd, 1); //reset

	measurementStartOnce();
	read(X_OUT0_ADDRESS, read_x_out, 2);
	read(X_OUT0_ADDRESS, read_y_out, 2);
	read(X_OUT0_ADDRESS, read_z_out, 2);

	uint16_t x_out_reset, y_out_reset, z_out_reset;
	x_out_reset = (read_x_out[0] << 8) | (read_x_out[1]);
	y_out_reset = (read_y_out[0] << 8) | (read_y_out[1]);
	z_out_reset = (read_z_out[0] << 8) | (read_z_out[1]);

	//Calucurate true value excluding offset
	int32_t x_out_H, y_out_H, z_out_H;
	x_out_H = (x_out_set - x_out_reset) / 2;
	y_out_H = (y_out_set - y_out_reset) / 2;
	z_out_H = (z_out_set - z_out_reset) / 2;

	//Calucurate offset
	offset_.x = x_out_set - x_out_H;
	offset_.y = y_out_set - y_out_H;
	offset_.z = z_out_set - z_out_H;

	mon_offset_x = offset_.x;
	mon_offset_y = offset_.y;
	mon_offset_z = offset_.z;
}

void MMC5983MA::updateData()
{
	//if(enable_flag_ == true){
		uint8_t xout_data[2], yout_data[2];

		//measurementStart();
		read(X_OUT0_ADDRESS, xout_data, 2); // read xout
		gauss_.x = (xout_data[0] << 8 | xout_data[1]) - offset_.x;
		mon_data[0] = xout_data[0];
		mon_data[1] = xout_data[1];
		mon_xout_calib = gauss_.x;

		//measurementStart();
		read(Y_OUT0_ADDRESS, yout_data, 2); // read yout
		gauss_.y = (yout_data[0] << 8 | yout_data[1]) - offset_.y;
		mon_yout_calib = gauss_.y;
	//}

}

int32_t MMC5983MA::getGaussXData()
{
	return gauss_.x;

}

int32_t MMC5983MA::getGaussYData()
{
	return gauss_.y;

}

int32_t MMC5983MA::getGaussZData()
{
	return gauss_.z;

}
