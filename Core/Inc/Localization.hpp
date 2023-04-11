/*
 * Localization.hpp
 *
 *  Created on: 2023/03/04
 *      Author: under
 */

#ifndef INC_LOCALIZATION_HPP_
#define INC_LOCALIZATION_HPP_


#include "GetSelfLocation.h"
#include "GetSelfLocation_terminate.h"
#include <cmath>
#include "stm32f4xx_hal.h"

class Localization{
private:
	bool execute_flag_, initialize_flag_;
	float translation_velocity_, anguler_velocity_;
	float measured_x_, measured_y_, measured_theta_;
	float observed_theta_;
	float estimated_x_, estimated_y_, estimated_theta_;

	float pre_position_[3], pre_pt_[9];

	float qt_, tred_, dt_, *error_parameter_;

	void convertErrorParameter(double result[4]);
	void convertTargetVelocityData(double result_data[], int result_size[2]);
	void convertMeasuredPositionData(double result_data[], int result_size[2]);
	void convertObservdThetaData(double result_data[], int result_size[2]);
	void convertPrePositionData(double result[3]);
	void convertPrePtData(double result[9]);
	double argInit_real_T(void);

	float calcTargetAngularVelocity(float);
	void initializePreData();

public:

	Localization(float, float, float, float *);

	void setTargetVelocity(float, float);
	void setMeasuredPosition(float, float, float);
	void setObservdTheta(float);

	void estimatePositionFlip();
	void getEstimatedPosition(float *, float *, float *);

	void enableEstimating();
	void disableEstimating();
};


#endif /* INC_LOCALIZATION_HPP_ */
