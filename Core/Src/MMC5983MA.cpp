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

struct Queue{
	uint8_t addr;
	uint8_t size;
};

struct StoreData{
	int32_t xout;
	int32_t yout;
	int32_t zout;
};

I2C_HandleTypeDef hi2c1;

uint8_t mon_data[2];
//uint16_t mon_xout, mon_yout;
HAL_StatusTypeDef mon_ret;

uint8_t mon_xout0, mon_xout1;
uint8_t mon_yout0, mon_yout1;
uint8_t mon_xyzout2;

uint8_t receive_buff[MAG_BUFF_SIZE];

Queue queue_data[MAG_QUEUE_SIZE]; //[address][read data size]
uint8_t queue_idx;

StoreData store_data;

//------private-------//
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(queue_idx >= 1){
		MMC5983MA mmc5983ma;
		mmc5983ma.receive_IT(receive_buff, queue_data[0].size);
	}

}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	uint16_t temp0 = receive_buff[0];
	uint16_t temp1 = receive_buff[1];

	if(queue_data[0].addr== X_OUT0_ADDRESS){
		store_data.xout = (temp0 << 8 | temp1);
		//store_data.xout = (receive_buff[0] << 8 | receive_buff[1]);
	}
	else if(queue_data[0].addr == Y_OUT0_ADDRESS){
		store_data.yout = (temp0 << 8 | temp1);
		//store_data.yout = (receive_buff[0] << 8 | receive_buff[1]);
	}
	else if(queue_data[0].addr == Z_OUT0_ADDRESS){
		store_data.zout = (temp0 << 8 | temp1);
		//store_data.zout = (receive_buff[0] << 8 | receive_buff[1]);
	}

	MMC5983MA mmc5983ma;
	mmc5983ma.shiftQueue();
	mmc5983ma.clearBuff();

	queue_idx--;

	if(queue_idx >= 1){
		MMC5983MA mmc5983ma;
		mmc5983ma.send_IT(&queue_data[0].addr, 1);
	}
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
	setQueue(address, read_data_size);

	if(queue_idx == 1){
		send_IT(&queue_data[0].addr, 1);
	}

}

void MMC5983MA::measurementStartOnce()
{
	uint8_t write_data;

	write_data = 0x01;
	write(INTERNAL_CONTROL0_ADDRESS, &write_data, 1); //Measument start

	queue_idx = 0;

}

void MMC5983MA::measurementStartContinuous()
{
	enable_flag_ = true;

	uint8_t write_data;

	write_data = 0x21; //0010 0001
	write(INTERNAL_CONTROL0_ADDRESS, &write_data, 1);

	write_data = 0xBD; //1011 1101
	write(INTERNAL_CONTROL2_ADDRESS, &write_data, 1);

	queue_idx = 0;
}

void MMC5983MA::measurementStop()
{

}

void MMC5983MA::calibration()
{
	uint8_t read_x_out[2], read_y_out[2], read_z_out[2];
	uint16_t temp_x[2];
	uint16_t temp_y[2];
	uint16_t temp_z[2];

	//Get values when device is set mode;
	uint8_t set_cmd = 0x08;
	write(INTERNAL_CONTROL0_ADDRESS, &set_cmd, 1); //set

	read(X_OUT0_ADDRESS, read_x_out, 2);
	read(Y_OUT0_ADDRESS, read_y_out, 2);
	read(Z_OUT0_ADDRESS, read_z_out, 2);


	int32_t x_out_set, y_out_set, z_out_set;
	temp_x[0] = read_x_out[0];
	temp_x[1] = read_x_out[1];
	temp_y[0] = read_y_out[0];
	temp_y[1] = read_y_out[1];
	temp_z[0] = read_z_out[0];
	temp_z[1] = read_z_out[1];

	x_out_set = (temp_x[0] << 8) | (temp_x[1]);
	y_out_set = (temp_y[0] << 8) | (temp_y[1]);
	z_out_set = (temp_z[0] << 8) | (temp_z[1]);

	HAL_Delay(10);

	//Get values when device is reset mode;
	uint8_t reset_cmd = 0x10;
	write(INTERNAL_CONTROL0_ADDRESS, &reset_cmd, 1); //reset

	read(X_OUT0_ADDRESS, read_x_out, 2);
	read(Y_OUT0_ADDRESS, read_y_out, 2);
	read(Z_OUT0_ADDRESS, read_z_out, 2);

	int32_t x_out_reset, y_out_reset, z_out_reset;
	temp_x[0] = read_x_out[0];
	temp_x[1] = read_x_out[1];
	temp_y[0] = read_y_out[0];
	temp_y[1] = read_y_out[1];
	temp_z[0] = read_z_out[0];
	temp_z[1] = read_z_out[1];
	x_out_reset = (temp_x[0] << 8) | (temp_x[1]);
	y_out_reset = (temp_y[0] << 8) | (temp_y[1]);
	z_out_reset = (temp_z[0] << 8) | (temp_z[1]);

	//Calucurate true value excluding offset
	int32_t x_out_H, y_out_H, z_out_H;
	x_out_H = (x_out_set - x_out_reset) / 2;
	y_out_H = (y_out_set - y_out_reset) / 2;
	z_out_H = (z_out_set - z_out_reset) / 2;

	//Calucurate offset
	offset_.x = x_out_set - x_out_H;
	offset_.y = y_out_set - y_out_H;
	offset_.z = z_out_set - z_out_H;

}

void MMC5983MA::updateData()
{
	//if(enable_flag_ == true){
		gauss_.x = store_data.xout - offset_.x;
		gauss_.y = store_data.yout - offset_.y;
		gauss_.z = store_data.zout - offset_.z;
		/*
		gauss_.x = store_data.xout;
		gauss_.y = store_data.yout;
		gauss_.z = store_data.zout;
		*/
	//}

}

void MMC5983MA::requestDataReading()
{
	read_IT(X_OUT0_ADDRESS, 2);
	read_IT(Y_OUT0_ADDRESS, 2);
	//read_IT(Z_OUT0_ADDRESS, 2);

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

void MMC5983MA::setQueue(uint8_t data, uint8_t size)
{
	queue_data[queue_idx].addr = data;
	queue_data[queue_idx].size = size;

	queue_idx++;
	if(queue_idx >= MAG_QUEUE_SIZE) queue_idx = MAG_QUEUE_SIZE - 1;

}

void MMC5983MA::shiftQueue()
{
	for(uint8_t idx = 0; idx < MAG_QUEUE_SIZE - 1; idx++){
		queue_data[idx] = queue_data[idx + 1];
	}
}
