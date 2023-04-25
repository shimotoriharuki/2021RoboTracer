/*
 * MMC5983MA.cpp
 *
 *  Created on: 2023/04/23
 *      Author: Haruki SHIMOTORI
 */

#include "MMC5983MA.hpp"

#define MAG_SLAVEADRESS 0x60
#define WRITE 0
#define READ 1

uint8_t mon_data[2];
uint16_t mon_xout, mon_yout;
HAL_StatusTypeDef mon_ret;

I2C_HandleTypeDef hi2c1;

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

MMC5983MA::MMC5983MA()
{

}

void MMC5983MA::write(uint8_t address, uint8_t *write_data, uint16_t write_data_size)
{
	uint8_t cmd[write_data_size + 1];
	cmd[0] = address;

	for(uint16_t i = 0; i < write_data_size; i++){
		cmd[i + 1] = write_data[i];
	}

	send(cmd, write_data_size + 1);
}

void MMC5983MA::read(uint8_t address, uint8_t *read_data, uint16_t read_data_size)
{
	send(&address, 1);
	HAL_Delay(1);
	receive(read_data, read_data_size);
}

void MMC5983MA::start()
{
	uint8_t address = 0x09;
	uint8_t write_data[1] = {0x01};

	write(address, write_data, 1);
	HAL_Delay(1);

	address = 0x08;
	read(address, mon_data, 1); // Appear Status Register data on SDA line. 6-7
	HAL_Delay(1);

	HAL_Delay(1000);

	address = 0x00;
	read(address, mon_data, 2); // Appear Xout data on SDA line. 10-11
	HAL_Delay(1);
	mon_xout = mon_data[0] <<8 | mon_data[1];

	address = 0x02;
	read(address, mon_data, 2); // Appear Yout data on SDA line. 10-11
	HAL_Delay(1);
	mon_yout = mon_data[0] <<8 | mon_data[1];

	HAL_Delay(1000);


}

void MMC5983MA::stop()
{

}

void MMC5983MA::updateData()
{

}

void MMC5983MA::getData()
{

}
