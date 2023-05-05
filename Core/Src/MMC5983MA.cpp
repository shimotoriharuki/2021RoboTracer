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
//uint16_t mon_xout, mon_yout;
int16_t mon_xout_calib, mon_yout_calib;
HAL_StatusTypeDef mon_ret;

int16_t mon_offset_x, mon_offset_y, mon_offset_z;

uint8_t mon_xout0, mon_xout1;
uint8_t mon_yout0, mon_yout1;
uint8_t mon_xyzout2;

static uint8_t receive_buff[MAG_BUFF_SIZE];
static uint8_t receive_buff_size;
static bool receive_waiting_flag;

static uint16_t store_xout;

//------private-------//
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(receive_waiting_flag == true){
		receive_waiting_flag = false;

		HAL_I2C_Master_Receive_IT(&hi2c1, MAG_SLAVEADRESS, receive_buff, receive_buff_size);
		/*
		MMC5983MA mmc5983ma;
		mmc5983ma.receive_IT(receive_buff, receive_buff_size);
		*/
	}

}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	store_xout = (receive_buff[0] << 8 | receive_buff[1]);
	/*
	MMC5983MA mmc5983ma;
	mmc5983ma.clearBuff();
	*/
}

void MMC5983MA::setInterruptReceiveDataSize(uint8_t size)
{
	receive_buff_size = size;
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

	clearBuff();

}

void MMC5983MA::send(uint8_t *cmd, uint16_t size)
{
	HAL_I2C_Master_Transmit(&hi2c1, MAG_SLAVEADRESS, cmd, size, 100);
}

void MMC5983MA::receive(uint8_t *received_data, uint16_t size)
{
	HAL_I2C_Master_Receive(&hi2c1, MAG_SLAVEADRESS, received_data, size, 100);
}

void MMC5983MA::send_IT(uint8_t *cmd, uint16_t size)
{
	HAL_I2C_Master_Transmit_IT(&hi2c1, MAG_SLAVEADRESS, cmd, size);
}

void MMC5983MA::receive_IT(uint8_t *received_data, uint16_t size)
{
	HAL_I2C_Master_Receive_IT(&hi2c1, MAG_SLAVEADRESS, received_data, size);
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

void MMC5983MA::write_IT(uint8_t address, uint8_t *write_data, uint16_t write_data_size)
{
	uint8_t cmd[write_data_size + 1];
	cmd[0] = address;

	for(uint16_t i = 0; i < write_data_size; i++){
		cmd[i + 1] = write_data[i];
	}

	send_IT(cmd, write_data_size + 1);
}

void MMC5983MA::read_IT(uint8_t address, uint16_t read_data_size)
{
	receive_waiting_flag = true;

	setInterruptReceiveDataSize(read_data_size);
	send_IT(&address, 1);

	//receive_IT(read_data, read_data_size);
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
	measurementStartOnce();
	read(Y_OUT0_ADDRESS, read_y_out, 2);
	measurementStartOnce();
	read(Z_OUT0_ADDRESS, read_z_out, 2);

	uint16_t x_out_set, y_out_set, z_out_set;
	x_out_set = (read_x_out[0] << 8) | (read_x_out[1]);
	y_out_set = (read_y_out[0] << 8) | (read_y_out[1]);
	z_out_set = (read_z_out[0] << 8) | (read_z_out[1]);

	HAL_Delay(10);

	//Get values when device is reset mode;
	uint8_t reset_cmd = 0x10;
	write(INTERNAL_CONTROL0_ADDRESS, &reset_cmd, 1); //reset

	measurementStartOnce();
	read(X_OUT0_ADDRESS, read_x_out, 2);
	measurementStartOnce();
	read(Y_OUT0_ADDRESS, read_y_out, 2);
	measurementStartOnce();
	read(Z_OUT0_ADDRESS, read_z_out, 2);

	uint16_t x_out_reset, y_out_reset, z_out_reset;
	x_out_reset = (read_x_out[0] << 8) | (read_x_out[1]);
	y_out_reset = (read_y_out[0] << 8) | (read_y_out[1]);
	z_out_reset = (read_z_out[0] << 8) | (read_z_out[1]);

	//Calucurate true value excluding offset
	int16_t x_out_H, y_out_H, z_out_H;
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
		uint8_t xout0_data, yout0_data;
		uint8_t xout1_data, yout1_data;
		uint8_t xyzout2_data;

		measurementStartOnce();
		read(X_OUT0_ADDRESS, &xout0_data, 1); // read xout
		measurementStartOnce();
		read(X_OUT1_ADDRESS, &xout1_data, 1); // read xout

		measurementStartOnce();
		read(Y_OUT0_ADDRESS, &yout0_data, 1); // read yout
		measurementStartOnce();
		read(Y_OUT1_ADDRESS, &yout1_data, 1); // read yout

		measurementStartOnce();
		read(XYZ_OUT2_ADDRESS, &xyzout2_data, 1); // read xyzout

		gauss_.x = (xout0_data << 8 | xout1_data) - offset_.x;
		//gauss_.x = (xout0_data << 8 | xout1_data);
		mon_xout_calib = gauss_.x;

		mon_xout0 = xout0_data;
		mon_xout1 = xout1_data;

		gauss_.y = (yout0_data << 8 | yout1_data) - offset_.y;
		//gauss_.y = (yout0_data << 8 | yout1_data);
		mon_yout_calib = gauss_.y;

		mon_yout0 = yout0_data;
		mon_yout1 = yout1_data;

		mon_xyzout2 = xyzout2_data;

	//}

}

int16_t MMC5983MA::getGaussXData()
{
	return gauss_.x;

}

int16_t MMC5983MA::getGaussYData()
{
	return gauss_.y;

}

int16_t MMC5983MA::getGaussZData()
{
	return gauss_.z;

}

void MMC5983MA::softwareReset()
{
	uint8_t write_data = 0x08;
	write(INTERNAL_CONTROL0_ADDRESS, &write_data, 1);
	HAL_Delay(20);
}

void MMC5983MA::clearBuff()
{
	for(uint16_t i = 0; i < MAG_BUFF_SIZE; i++){
		receive_buff[i] = 0;
	}
}
