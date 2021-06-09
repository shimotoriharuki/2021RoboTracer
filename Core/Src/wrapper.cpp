/*
 * wrapper.cpp
 *
 *  Created on: Jun 9, 2021
 *      Author: under
 */

#include "wrapper.hpp"
#include "LineSensor.hpp"

void cpploop(void) {
    LineSensor instance;

    instance.ADCStart();
}


