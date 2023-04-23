/*
 * MMC5983MA.cpp
 *
 *  Created on: 2023/04/23
 *      Author: Haruki SHIMOTORI
 */

#include "MMC5983MA.hpp"

#define SLAVEADRESS 0x60
#define WRITE 0
#define READ 1

uint8_t mon_data;

//------private-------//
void MMC5983MA::send(uint8_t mode, uint8_t *cmd, uint16_t size)
{
	HAL_I2C_Master_Transmit(&hi2c1, SLAVEADRESS | mode, cmd, size, 100);

}

void MMC5983MA::read(uint8_t data)
{
}



//------public--------//

MMC5983MA::MMC5983MA()
{

}

void MMC5983MA::start()
{
	uint8_t cmd[2];

	cmd[0] = 0x09;
	cmd[1] = 0x01;
	send(WRITE, cmd, 2); // Request a write to Internal Control Resister 0 and write initiate measurement command. 1-3
	HAL_Delay(1);

	cmd[0] = {0x08};
	send(WRITE, cmd, 1); // Request a write to Status Resister. 4-5
	HAL_Delay(1);

	send(READ, cmd, 0); // Appear Status Register data on SDA line. 6-7
	HAL_Delay(1);
	HAL_I2C_Master_Receive(&hi2c1, SLAVEADRESS, &mon_data, 1, 100);

	HAL_Delay(1000);

	cmd[0] = {0x00};
	send(WRITE, cmd, 1); //Request a write to Xout. 8-9
	HAL_Delay(1);

	send(READ, cmd, 0); // Appear Xout data on SDA line. 10-11
	HAL_Delay(1);
	HAL_I2C_Master_Receive(&hi2c1, SLAVEADRESS, &mon_data, 1, 100);
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
