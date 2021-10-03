/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: path_following.c
 *
 * Code generated for Simulink model 'path_following'.
 *
 * Model version                  : 3.120
 * Simulink Coder version         : 9.5 (R2021a) 14-Nov-2020
 * C/C++ source code generated on : Wed Aug 25 14:46:48 2021
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: STMicroelectronics->ST10/Super10
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

double mon1, mon2;

#include "path_following.h"

/* Block signals and states (default storage) */
DW rtDW;

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

Param rtParam;

/* Real-time model */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;
static void CalcError1(void);

/* Output and update for atomic system: '<S1>/CalcError1' */
static void CalcError1(void)
{
  real_T Add4_tmp;
  real_T Add4_tmp_0;
  real_T rtb_Add1;
  real_T rtb_Add2;

  /* Sum: '<S2>/Add1' incorporates:
   *  Inport: '<Root>/Py'
   *  Inport: '<Root>/y_cur'
   */
  rtb_Add1 = rtU.target_y - rtU.y;

  /* Sum: '<S2>/Add2' incorporates:
   *  Inport: '<Root>/Px'
   *  Inport: '<Root>/x_cur'
   */
  rtb_Add2 = rtU.target_x - rtU.x;

  /* Trigonometry: '<S2>/Trigonometric Function1' incorporates:
   *  Inport: '<Root>/th_cur'
   *  Trigonometry: '<S2>/Trigonometric Function2'
   */
  Add4_tmp = sin(rtU.th_cur);

  /* Trigonometry: '<S2>/Trigonometric Function4' incorporates:
   *  Inport: '<Root>/th_cur'
   *  Trigonometry: '<S2>/Trigonometric Function3'
   */
  Add4_tmp_0 = cos(rtU.th_cur);

  /* Sum: '<S2>/Add4' incorporates:
   *  Product: '<S2>/Product'
   *  Product: '<S2>/Product1'
   *  Trigonometry: '<S2>/Trigonometric Function1'
   *  Trigonometry: '<S2>/Trigonometric Function4'
   */
  rtDW.Add4 = rtb_Add2 * Add4_tmp_0 + rtb_Add1 * Add4_tmp;

  /* Sum: '<S2>/Add5' incorporates:
   *  Gain: '<S2>/Gain4'
   *  Product: '<S2>/Product2'
   *  Product: '<S2>/Product3'
   */
  rtDW.Add5 = rtb_Add2 * -Add4_tmp + rtb_Add1 * Add4_tmp_0;

  /* Sum: '<S2>/Add3' incorporates:
   *  Inport: '<Root>/Pth'
   *  Inport: '<Root>/th_cur'
   */
  rtDW.Add3 = rtU.th - rtU.th_cur;
}

/* Model step function */
void path_following_step(void)
{
  real_T rtb_Diff;
  real_T rtb_Square;
  real_T rtb_TSamp;
  real_T rtb_TSamp_gl;
  real_T rtb_Uk1;

  /* Outputs for Atomic SubSystem: '<Root>/path_following' */
  /* SampleTimeMath: '<S5>/TSamp' incorporates:
   *  Inport: '<Root>/Px'
   *
   * About '<S5>/TSamp':
   *  y = u * K where K = 1 / ( w * Ts )
   */
  rtb_TSamp = rtU.target_x * 1000.0;

  /* Sum: '<S5>/Diff' incorporates:
   *  UnitDelay: '<S5>/UD'
   *
   * Block description for '<S5>/Diff':
   *
   *  Add in CPU
   *
   * Block description for '<S5>/UD':
   *
   *  Store in Global RAM
   */
  rtb_Uk1 = rtb_TSamp - rtDW.UD_DSTATE;
  //mon2 = rtb_Uk1;
  mon1 = rtb_TSamp;
  mon2 = rtDW.UD_DSTATE;

  /* Math: '<S1>/Square' */
  rtb_Square = rtb_Uk1 * rtb_Uk1;

  /* SampleTimeMath: '<S6>/TSamp' incorporates:
   *  Inport: '<Root>/Py'
   *
   * About '<S6>/TSamp':
   *  y = u * K where K = 1 / ( w * Ts )
   */
  rtb_TSamp_gl = rtU.target_y * 1000.0;

  /* Sum: '<S6>/Diff' incorporates:
   *  UnitDelay: '<S6>/UD'
   *
   * Block description for '<S6>/Diff':
   *
   *  Add in CPU
   *
   * Block description for '<S6>/UD':
   *
   *  Store in Global RAM
   */
  rtb_Uk1 = rtb_TSamp_gl - rtDW.UD_DSTATE_o;

  /* Sqrt: '<S1>/Sqrt' incorporates:
   *  Math: '<S1>/Square1'
   *  Sum: '<S1>/Add5'
   */
  rtb_Uk1 = sqrt(rtb_Uk1 * rtb_Uk1 + rtb_Square);

  /* Outputs for Atomic SubSystem: '<S1>/CalcError1' */
  CalcError1();

  /* End of Outputs for SubSystem: '<S1>/CalcError1' */

  /* SampleTimeMath: '<S4>/TSamp' incorporates:
   *  Inport: '<Root>/Pth'
   *
   * About '<S4>/TSamp':
   *  y = u * K where K = 1 / ( w * Ts )
   */
  rtb_Square = rtU.th * 1000.0;

  /* Sum: '<S4>/Diff' incorporates:
   *  UnitDelay: '<S4>/UD'
   *
   * Block description for '<S4>/Diff':
   *
   *  Add in CPU
   *
   * Block description for '<S4>/UD':
   *
   *  Store in Global RAM
   */
  rtb_Diff = rtb_Square - rtDW.UD_DSTATE_d;

  /* Update for UnitDelay: '<S5>/UD'
   *
   * Block description for '<S5>/UD':
   *
   *  Store in Global RAM
   */
  rtDW.UD_DSTATE = rtb_TSamp;

  /* Update for UnitDelay: '<S6>/UD'
   *
   * Block description for '<S6>/UD':
   *
   *  Store in Global RAM
   */
  rtDW.UD_DSTATE_o = rtb_TSamp_gl;

  /* Update for UnitDelay: '<S4>/UD'
   *
   * Block description for '<S4>/UD':
   *
   *  Store in Global RAM
   */
  rtDW.UD_DSTATE_d = rtb_Square;

  /* Outputs for Atomic SubSystem: '<S1>/ClacTarget_V��1' */
  /* Outport: '<Root>/V_tar' incorporates:
   *  Gain: '<S3>/Gain'
   *  Product: '<S3>/Product1'
   *  Sum: '<S3>/Add5'
   *  Trigonometry: '<S3>/Trigonometric Function2'
   */
  rtY.V_tar = rtParam.kx * rtDW.Add4 + cos(rtDW.Add3) * rtb_Uk1;

  /* Outport: '<Root>/��_tar' incorporates:
   *  Gain: '<S3>/Gain1'
   *  Gain: '<S3>/Gain2'
   *  Product: '<S3>/Product2'
   *  Sum: '<S3>/Add1'
   *  Sum: '<S3>/Add2'
   *  Trigonometry: '<S3>/Trigonometric Function1'
   */
  rtY.tar = (rtParam.ky * rtDW.Add5 + rtParam.kt * sin(rtDW.Add3)) * rtb_Uk1 + rtb_Diff;


  /* End of Outputs for SubSystem: '<S1>/ClacTarget_V��1' */
  /* End of Outputs for SubSystem: '<Root>/path_following' */
}

/* Model initialize function */
void path_following_initialize(void)
{
  /* (no initialization code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
