/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: GetSelfLocation.c
 *
 * MATLAB Coder version            : 5.5
 * C/C++ source code generated on  : 23-Feb-2023 16:53:43
 */

/* Include Files */
#include "GetSelfLocation.h"
#include <math.h>
#include <string.h>

/* Custom Source Code */
/* Author: SHIMOTORI, Haruki */
/* Function Declarations */
static void binary_expand_op(double in1_data[], int in1_size[2],
                             const double in2_data[], const int *in2_size,
                             const double in3[3], const double in4_data[],
                             const int in4_size[2], double in5);

/* Function Definitions */
/*
 * Arguments    : double in1_data[]
 *                int in1_size[2]
 *                const double in2_data[]
 *                const int *in2_size
 *                const double in3[3]
 *                const double in4_data[]
 *                const int in4_size[2]
 *                double in5
 * Return Type  : void
 */
static void binary_expand_op(double in1_data[], int in1_size[2],
                             const double in2_data[], const int *in2_size,
                             const double in3[3], const double in4_data[],
                             const int in4_size[2], double in5)
{
  double b_in2_data[6];
  double b_in4_data[2];
  int i;
  int in2_size_idx_1;
  int in4_size_idx_1;
  int loop_ub;
  in4_size_idx_1 = in4_size[1];
  loop_ub = in4_size[1];
  for (i = 0; i < loop_ub; i++) {
    b_in4_data[i] = in4_data[i] - in5;
  }
  in1_size[0] = 3;
  in1_size[1] = in4_size_idx_1;
  for (i = 0; i < in4_size_idx_1; i++) {
    double d;
    d = b_in4_data[i];
    in1_data[3 * i] = in3[0] * d;
    in1_data[3 * i + 1] = in3[1] * d;
    in1_data[3 * i + 2] = in3[2] * d;
  }
  in2_size_idx_1 = in1_size[1];
  in4_size_idx_1 = (*in2_size != 1);
  loop_ub = in1_size[1];
  for (i = 0; i < loop_ub; i++) {
    int in2_data_tmp;
    b_in2_data[3 * i] = in2_data[0] + in1_data[3 * i];
    in2_data_tmp = 3 * i + 1;
    b_in2_data[in2_data_tmp] =
        in2_data[in4_size_idx_1] + in1_data[in2_data_tmp];
    in2_data_tmp = 3 * i + 2;
    b_in2_data[in2_data_tmp] =
        in2_data[in4_size_idx_1 << 1] + in1_data[in2_data_tmp];
  }
  in1_size[0] = 3;
  in1_size[1] = in2_size_idx_1;
  for (i = 0; i < in2_size_idx_1; i++) {
    in1_data[3 * i] = b_in2_data[3 * i];
    in4_size_idx_1 = 3 * i + 1;
    in1_data[in4_size_idx_1] = b_in2_data[in4_size_idx_1];
    in4_size_idx_1 = 3 * i + 2;
    in1_data[in4_size_idx_1] = b_in2_data[in4_size_idx_1];
  }
}

/*
 * --------------------Init------------------%
 *
 * Arguments    : const double MeasuredPosition_data[]
 *                const int MeasuredPosition_size[2]
 *                const double ObsZt_data[]
 *                const int ObsZt_size[2]
 *                const double TargetVelo_data[]
 *                const int TargetVelo_size[2]
 *                const double PrePosition[3]
 *                const double PrePt[9]
 *                const double ErrerParameter[4]
 *                double Qt
 *                double Tred
 *                double dt
 *                double EstPosition_data[]
 *                int EstPosition_size[2]
 *                double EstPt[9]
 * Return Type  : void
 */
void GetSelfLocation(const double MeasuredPosition_data[],
                     const int MeasuredPosition_size[2],
                     const double ObsZt_data[], const int ObsZt_size[2],
                     const double TargetVelo_data[],
                     const int TargetVelo_size[2], const double PrePosition[3],
                     const double PrePt[9], const double ErrerParameter[4],
                     double Qt, double Tred, double dt,
                     double EstPosition_data[], int EstPosition_size[2],
                     double EstPt[9])
{
  static const signed char a[3] = {0, 0, 1};
  static const signed char iv[3] = {0, 0, 1};
  double At[9];
  double HatPt[9];
  double b_At[9];
  double Wt[6];
  double b_Wt[6];
  double Kt[3];
  double b_MeasuredPosition_data[3];
  double ErrerParameter_idx_0;
  double dSl;
  double dSr;
  double u_idx_0;
  double u_idx_1;
  int ObsZt_size_idx_1;
  int i;
  int i1;
  int loop_ub;
  (void)TargetVelo_size;
  /* ---------------------- Calclation start----------------------------% */
  /*  Calclate dS & dTh */
  /*      global Tred */
  /*      global dt */
  dSr = (TargetVelo_data[1] * Tred + 2.0 * TargetVelo_data[0]) / 2.0 * dt;
  dSl = (-TargetVelo_data[1] * Tred + 2.0 * TargetVelo_data[0]) / 2.0 * dt;
  u_idx_0 = (dSr + dSl) / 2.0;
  u_idx_1 = (dSr - dSl) / Tred;
  /*  The process noise covariance matrix */
  /* ----------------------- Forecast step----------------------% */
  /*  Calclation At & Wt */
  At[0] = 1.0;
  At[3] = 0.0;
  dSr = PrePosition[2] + u_idx_1 / 2.0;
  dSl = sin(dSr);
  At[6] = -u_idx_0 * dSl;
  At[1] = 0.0;
  At[4] = 1.0;
  dSr = cos(dSr);
  At[7] = u_idx_0 * dSr;
  At[2] = 0.0;
  At[5] = 0.0;
  At[8] = 1.0;
  Wt[0] = dSr;
  Wt[3] = -u_idx_0 / 2.0 * dSl;
  Wt[1] = dSl;
  Wt[4] = u_idx_0 / 2.0 * dSr;
  Wt[2] = 0.0;
  Wt[5] = 1.0;
  /*  Calclation estmation errors covariance matrix */
  dSl = u_idx_0 * u_idx_0;
  dSr = u_idx_1 * u_idx_1;
  ErrerParameter_idx_0 = ErrerParameter[0] * dSl + ErrerParameter[1] * dSr;
  dSr = ErrerParameter[2] * dSl + ErrerParameter[3] * dSr;
  for (i = 0; i < 3; i++) {
    i1 = (int)At[i];
    ObsZt_size_idx_1 = (int)At[i + 3];
    dSl = At[i + 6];
    for (loop_ub = 0; loop_ub < 3; loop_ub++) {
      b_At[i + 3 * loop_ub] =
          ((double)i1 * PrePt[3 * loop_ub] +
           (double)ObsZt_size_idx_1 * PrePt[3 * loop_ub + 1]) +
          dSl * PrePt[3 * loop_ub + 2];
    }
    dSl = Wt[i + 3];
    u_idx_1 = Wt[i];
    b_Wt[i] = u_idx_1 * ErrerParameter_idx_0 + dSl * 0.0;
    b_Wt[i + 3] = u_idx_1 * 0.0 + dSl * dSr;
    dSl = b_At[i];
    u_idx_1 = b_At[i + 3];
    u_idx_0 = b_At[i + 6];
    for (i1 = 0; i1 < 3; i1++) {
      HatPt[i + 3 * i1] =
          (dSl * At[i1] + u_idx_1 * At[i1 + 3]) + u_idx_0 * At[i1 + 6];
    }
  }
  for (i = 0; i < 3; i++) {
    dSl = b_Wt[i];
    u_idx_1 = b_Wt[i + 3];
    for (i1 = 0; i1 < 3; i1++) {
      At[i + 3 * i1] = dSl * Wt[i1] + u_idx_1 * Wt[i1 + 3];
    }
  }
  for (i = 0; i < 9; i++) {
    HatPt[i] += At[i];
  }
  /* ------------ Update step------------% */
  /*  Get Robot's angle for gyro or geomagnetism */
  /*      ObsZt = GetAngleForIMU(PreZt, TargetVelo(2), Qt, dt);  */
  /*  Covariance of observation residuals */
  dSr = 0.0;
  for (i = 0; i < 3; i++) {
    dSr += ((0.0 * HatPt[3 * i] + 0.0 * HatPt[3 * i + 1]) + HatPt[3 * i + 2]) *
           (double)iv[i];
  }
  dSr += Qt;
  /*  Calclation Kalman constant */
  for (i = 0; i < 3; i++) {
    Kt[i] = ((HatPt[i] * 0.0 + HatPt[i + 3] * 0.0) + HatPt[i + 6]) / dSr;
  }
  /*  Transpose matrix */
  loop_ub = MeasuredPosition_size[1];
  if (loop_ub - 1 >= 0) {
    memcpy(&b_MeasuredPosition_data[0], &MeasuredPosition_data[0],
           (unsigned int)loop_ub * sizeof(double));
  }
  /*  Calclation estimation position */
  dSr = (0.0 * b_MeasuredPosition_data[0] + 0.0 * b_MeasuredPosition_data[1]) +
        b_MeasuredPosition_data[2];
  if (MeasuredPosition_size[1] == 3) {
    double b_ObsZt_data[2];
    int MeasuredPosition_size_idx_1;
    ObsZt_size_idx_1 = ObsZt_size[1];
    loop_ub = ObsZt_size[1];
    for (i = 0; i < loop_ub; i++) {
      b_ObsZt_data[i] = ObsZt_data[i] - dSr;
    }
    for (i = 0; i < ObsZt_size_idx_1; i++) {
      dSl = b_ObsZt_data[i];
      EstPosition_data[3 * i] = Kt[0] * dSl;
      EstPosition_data[3 * i + 1] = Kt[1] * dSl;
      EstPosition_data[3 * i + 2] = Kt[2] * dSl;
    }
    MeasuredPosition_size_idx_1 = ObsZt_size[1];
    loop_ub = ObsZt_size[1];
    for (i = 0; i < loop_ub; i++) {
      for (i1 = 0; i1 < 3; i1++) {
        ObsZt_size_idx_1 = i1 + 3 * i;
        Wt[ObsZt_size_idx_1] =
            b_MeasuredPosition_data[i1] + EstPosition_data[ObsZt_size_idx_1];
      }
    }
    EstPosition_size[0] = 3;
    EstPosition_size[1] = ObsZt_size[1];
    for (i = 0; i < MeasuredPosition_size_idx_1; i++) {
      EstPosition_data[3 * i] = Wt[3 * i];
      ObsZt_size_idx_1 = 3 * i + 1;
      EstPosition_data[ObsZt_size_idx_1] = Wt[ObsZt_size_idx_1];
      ObsZt_size_idx_1 = 3 * i + 2;
      EstPosition_data[ObsZt_size_idx_1] = Wt[ObsZt_size_idx_1];
    }
  } else {
    binary_expand_op(EstPosition_data, EstPosition_size,
                     b_MeasuredPosition_data, &MeasuredPosition_size[1], Kt,
                     ObsZt_data, ObsZt_size, dSr);
  }
  /*  Updata estmation errors covariance matrix */
  memset(&At[0], 0, 9U * sizeof(double));
  At[0] = 1.0;
  At[4] = 1.0;
  At[8] = 1.0;
  dSl = Kt[0];
  u_idx_1 = Kt[1];
  u_idx_0 = Kt[2];
  for (i = 0; i < 3; i++) {
    ObsZt_size_idx_1 = a[i];
    b_At[3 * i] = At[3 * i] - dSl * (double)ObsZt_size_idx_1;
    loop_ub = 3 * i + 1;
    b_At[loop_ub] = At[loop_ub] - u_idx_1 * (double)ObsZt_size_idx_1;
    loop_ub = 3 * i + 2;
    b_At[loop_ub] = At[loop_ub] - u_idx_0 * (double)ObsZt_size_idx_1;
  }
  for (i = 0; i < 3; i++) {
    dSl = b_At[i];
    u_idx_1 = b_At[i + 3];
    u_idx_0 = b_At[i + 6];
    for (i1 = 0; i1 < 3; i1++) {
      EstPt[i + 3 * i1] = (dSl * HatPt[3 * i1] + u_idx_1 * HatPt[3 * i1 + 1]) +
                          u_idx_0 * HatPt[3 * i1 + 2];
    }
  }
}

/*
 * File trailer for GetSelfLocation.c
 *
 * [EOF]
 */
