/*
 * wrapper.cpp
 *
 *  Created on: Jun 9, 2021
 *      Author: under
 */

#include "wrapper.hpp"
#include "LineSensor.hpp"

void ADCStart(void) {
    LineSensor instance;

    instance.ADCStart();
}


