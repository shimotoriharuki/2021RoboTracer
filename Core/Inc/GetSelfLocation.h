/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: GetSelfLocation.h
 *
 * MATLAB Coder version            : 5.5
 * C/C++ source code generated on  : 23-Feb-2023 16:53:43
 */

#ifndef GETSELFLOCATION_H
#define GETSELFLOCATION_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void GetSelfLocation(const double MeasuredPosition_data[],
                            const int MeasuredPosition_size[2],
                            const double ObsZt_data[], const int ObsZt_size[2],
                            const double TargetVelo_data[],
                            const int TargetVelo_size[2],
                            const double PrePosition[3], const double PrePt[9],
                            const double ErrerParameter[4], double Qt,
                            double Tred, double dt, double EstPosition_data[],
                            int EstPosition_size[2], double EstPt[9]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for GetSelfLocation.h
 *
 * [EOF]
 */
