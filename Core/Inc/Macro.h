/*
 * Macro.h
 *
 *  Created on: 2021/06/10
 *      Author: under
 */

#ifndef INC_MACRO_H_
#define INC_MACRO_H_

//SD card
#define DATA_SIZE 1
#define OVER_WRITE 0	// over write
#define ADD_WRITE 1	// add write

// ADC
#define AD_DATA_SIZE 14

// Joy stick
#define JOY_U 0x08
#define JOY_D 0x04
#define JOY_L 0x01
#define JOY_R 0x10
#define JOY_C 0x02

//voltage and current sensor
#define CURRENT_VOLTAGE_SENSOR_ADRESS_LEFT (0x44<<1)
#define CURRENT_VOLTAGE_SENSOR_ADRESS_RIGHT (0x40<<1)


#endif /* INC_MACRO_H_ */
