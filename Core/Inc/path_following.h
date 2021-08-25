/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: path_following.h
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

#ifndef RTW_HEADER_path_following_h_
#define RTW_HEADER_path_following_h_
#include <math.h>
#ifndef path_following_COMMON_INCLUDES_
#define path_following_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#endif                                 /* path_following_COMMON_INCLUDES_ */

/* Model Code Variants */

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM RT_MODEL;

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  real_T Add4;                         /* '<S2>/Add4' */
  real_T Add5;                         /* '<S2>/Add5' */
  real_T Add3;                         /* '<S2>/Add3' */
  real_T UD_DSTATE;                    /* '<S5>/UD' */
  real_T UD_DSTATE_o;                  /* '<S6>/UD' */
  real_T UD_DSTATE_d;                  /* '<S4>/UD' */
} DW;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T target_x;                     /* '<Root>/Px' */
  real_T target_y;                     /* '<Root>/Py' */
  real_T th;                           /* '<Root>/Pth' */
  real_T x;                            /* '<Root>/x_cur' */
  real_T y;                            /* '<Root>/y_cur' */
  real_T th_cur;                       /* '<Root>/th_cur' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T V_tar;                        /* '<Root>/V_tar' */
  real_T tar;                          /* '<Root>/É÷_tar' */
} ExtY;

/* Real-time Model Data Structure */
struct tag_RTM {
  const char_T * volatile errorStatus;
};

/* Block signals and states (default storage) */
extern DW rtDW;

/* External inputs (root inport signals with default storage) */
extern ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY rtY;

/* Model entry point functions */
extern void path_following_initialize(void);
extern void path_following_step(void);

/* Real-time Model object */
extern RT_MODEL *const rtM;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S4>/Data Type Duplicate' : Unused code path elimination
 * Block '<S5>/Data Type Duplicate' : Unused code path elimination
 * Block '<S6>/Data Type Duplicate' : Unused code path elimination
 * Block '<S1>/Vr_scope' : Unused code path elimination
 * Block '<S2>/Rate Transition' : Eliminated since input and output rates are identical
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Note that this particular code originates from a subsystem build,
 * and has its own system numbers different from the parent model.
 * Refer to the system hierarchy for this subsystem below, and use the
 * MATLAB hilite_system command to trace the generated code back
 * to the parent model.  For example,
 *
 * hilite_system('path_following_test/path_following')    - opens subsystem path_following_test/path_following
 * hilite_system('path_following_test/path_following/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'path_following_test'
 * '<S1>'   : 'path_following_test/path_following'
 * '<S2>'   : 'path_following_test/path_following/CalcError1'
 * '<S3>'   : 'path_following_test/path_following/ClacTarget_VÉ÷1'
 * '<S4>'   : 'path_following_test/path_following/Discrete Derivative'
 * '<S5>'   : 'path_following_test/path_following/Discrete Derivative1'
 * '<S6>'   : 'path_following_test/path_following/Discrete Derivative2'
 */
#endif                                 /* RTW_HEADER_path_following_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
