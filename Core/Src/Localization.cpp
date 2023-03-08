/*
 * Localization.cpp
 *
 *  Created on: 2023/03/04
 *      Author: Haruki SHIMOTORI
 */

#include "Localization.hpp"


// ------------------------------//
// ------------private-----------//
// ------------------------------//

/*
 * Arguments    : double result[4]
 * Return Type  : void
 */
void Localization::argInit_1x4_real_T(double result[4])
{
  int idx1;
  /* Loop over the array to initialize each element. */
  for (idx1 = 0; idx1 < 4; idx1++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx1] = argInit_real_T();
  }
}

/*
 * Arguments    : double result_data[]
 *                int result_size[2]
 * Return Type  : void
 */
void Localization::convertTargetVelocityData(double result_data[], int result_size[2])
{
  /* Set the size of the array.
Change this size to the value that the application requires. */
  result_size[0] = 1;
  result_size[1] = 2;

  result_data[0] = translation_velocity_;
  result_data[1] = anguler_velocity_;
}

/*
 * Arguments    : double result_data[]
 *                int result_size[2]
 * Return Type  : void
 */
void Localization::convertMeasuredPositionData(double result_data[], int result_size[2])
{
  /* Set the size of the array.
Change this size to the value that the application requires. */
  result_size[0] = 1;
  result_size[1] = 3;

  result_data[0] = measured_x_;
  result_data[1] = measured_y_;
  result_data[2] = measured_theta_;
}

/*
 * Arguments    : double result_data[]
 *                int result_size[2]
 * Return Type  : void
 */
void Localization::convertObservdThetaData(double result_data[], int result_size[2])
{
  /* Set the size of the array.
Change this size to the value that the application requires. */
  result_size[0] = 1;
  result_size[1] = 1;

  result_data[0] = observed_theta_;
}
/*
 * Arguments    : double result[3]
 * Return Type  : void
 */
void Localization::convertPrePositionData(double result[3])
{
  int idx0;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 3; idx0++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx0] = pre_position_[idx0];
  }
}

/*
 * Arguments    : double result[9]
 * Return Type  : void
 */
void Localization::convertPrePtData(double result[9])
{
  int i;
  /* Loop over the array to initialize each element. */
  for (i = 0; i < 9; i++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[i] = pre_pt_[i];
  }
}

/*
 * Arguments    : void
 * Return Type  : double
 */
double Localization::argInit_real_T(void)
{
  return 0.0;
}

float Localization::calcTargetAngularVelocity(float line_sensor_diff)
{

	return 0;
}

void Localization::initializePreData()
{
	for(uint16_t i = 0; i < 3; i++){
		pre_position_[i] = 0;
	}
	for(uint16_t i = 0; i < 9; i++){
		pre_pt_[i] = 0;
	}

}

// ------------------------------//
// ------------public------------//
// ------------------------------//

Localization::Localization(float qt, float tred, float dt, float *error_parameter) : execute_flag_(false), initialize_flag_(false), translation_velocity_(0), anguler_velocity_(0),
		measured_x_(0), measured_y_(0), measured_theta_(0),
		estimated_x_(0), estimated_y_(0), estimated_theta_(0)
{
	qt_ = qt;
	tred_ = tred;
	dt_ = dt;
	error_parameter_ = error_parameter;

	initializePreData();

}

void Localization::setTargetVelocity(float translation_velocity, float rotation_ratio)
{
	translation_velocity_ = translation_velocity;

	anguler_velocity_ = calcTargetAngularVelocity(rotation_ratio);

}

void Localization::setMeasuredPosition(float x, float y, float theta)
{
	measured_x_ = x;
	measured_y_ = y;
	measured_theta_ = theta;
}

void Localization::setObservdTheta(float theta)
{
	observed_theta_ = theta;
}

void Localization::estimatePositionFlip()
{
	if(execute_flag_ == true){
		if(initialize_flag_ == true){ //Initialize
			initialize_flag_ = false;
			initializePreData();
		}

		double EstPt[9];
		double PrePt[9];
		double EstPosition_data[6];
		double ErrorParameter[4];
		double MeasuredPosition_data[3];
		double TargetVelo_data[3];
		double PrePosition[3];
		double ObsZt_data[2];
		double Qt;
		double Tred;
		double dt;
		int EstPosition_size[2];
		int MeasuredPosition_size[2];
		int ObsZt_size[2];
		int TargetVelo_size[2];

		/*------ Initialize function 'GetSelfLocation' input arguments. -------*/
		/* Initialize function input argument 'MeasuredPosition'. */
		convertMeasuredPositionData(MeasuredPosition_data, MeasuredPosition_size);

		/* Initialize function input argument 'ObsZt'. */
		convertObservdThetaData(ObsZt_data, ObsZt_size);

		/* Initialize function input argument 'TargetVelo'. */
		convertTargetVelocityData(TargetVelo_data, TargetVelo_size);

		Qt = qt_;
		Tred = tred_; //100.6mm
		dt = dt_; //10ms

		/* Call the entry-point 'GetSelfLocation'. */
		convertPrePositionData(PrePosition);
		convertPrePtData(PrePt);
		//argInit_1x4_real_T(ErrorParameter);

		GetSelfLocation(MeasuredPosition_data, MeasuredPosition_size, ObsZt_data,
					  ObsZt_size, TargetVelo_data, TargetVelo_size, PrePosition, PrePt, ErrorParameter,
					  Qt, Tred, dt, EstPosition_data, EstPosition_size,
					  EstPt);

		for(uint16_t i = 0; i < 3; i++){
			pre_position_[i] = EstPosition_data[i];
		}
		for(uint16_t i = 0; i < 9; i++){
			pre_pt_[i] = EstPt[i];
		}
	}

}

void Localization::getEstimatedPosition(float *x, float *y, float *theta)
{
	*x = estimated_x_;
	*y = estimated_y_;
	*theta = estimated_theta_;
}

void Localization::enableEstimating()
{
	execute_flag_ = true;
	initialize_flag_ = true;
}

void Localization::disableEstimating()
{
	execute_flag_ = false;
}
