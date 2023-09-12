/*
 * File: Balanced_Infantry.h
 *
 * Code generated for Simulink model :Balanced_Infantry.
 *
 * Model version      : 1.67
 * Simulink Coder version    : 9.3 (R2020a) 18-Nov-2019
 * TLC version       : 9.3 (Jan 23 2020)
 * C/C++ source code generated on  : Fri Jan  7 13:43:24 2022
 *
 * Target selection: stm32.tlc
 * Embedded hardware selection: STM32CortexM
 * Code generation objectives: Unspecified
 * Validation result: Not run
 *
 *
 *
 * ******************************************************************************
 * * attention
 * *
 * * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 * *
 * ******************************************************************************
 */

#ifndef RTW_HEADER_Balanced_Infantry_h_
#define RTW_HEADER_Balanced_Infantry_h_
#include <math.h>
#include "STM32_Config.h"
#include "Balanced_Infantry_External_Functions.h"
#ifndef Balanced_Infantry_COMMON_INCLUDES_
# define Balanced_Infantry_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* Balanced_Infantry_COMMON_INCLUDES_ */

#include "Balanced_Infantry_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T LQR_enable;                   /* '<Root>/LQR_enable' */
  real_T Suspension_enable;            /* '<Root>/Suspension_enable' */
  real_T R_angular_speed_feedback;     /* '<Root>/R_angular_speed_feedback' */
  real_T R_angle_feedback;             /* '<Root>/R_angle_feedback' */
  real_T L_angular_speed_feedback;     /* '<Root>/L_angular_speed_feedback' */
  real_T L_angle_feedback;             /* '<Root>/L_angle_feedback' */
  real_T speed_L;                      /* '<Root>/speed_L' */
  real_T speed_R;                      /* '<Root>/speed_R' */
  real_T angle_L;                      /* '<Root>/angle_L' */
  real_T angle_R;                      /* '<Root>/angle_R' */
  real_T Suspension_press;             /* '<Root>/Suspension_press' */
  real_T Suspension_rebound;           /* '<Root>/Suspension_rebound' */
  real_T Suspension_x;                 /* '<Root>/Suspension_x' */
  real_T Suspension_K2;                /* '<Root>/Suspension_K2' */
  real_T Suspension_K1;                /* '<Root>/Suspension_K1' */
  real_T Suspension_K3;                /* '<Root>/Suspension_K3' */
  real_T PITCH_LQR_K1;                 /* '<Root>/PITCH_LQR_K1' */
  real_T PITCH_LQR_K2;                 /* '<Root>/PITCH_LQR_K2' */
  real_T R_angle_set;                  /* '<Root>/R_angle_set' */
  real_T L_angle_set;                  /* '<Root>/L_angle_set' */
  real_T Roll_stabilize_enable;        /* '<Root>/Roll_stabilize_enable' */
  real_T Roll_angular_speed_feedback; /* '<Root>/Roll_angular_speed_feedback' */
  real_T Roll_angle_feedback;          /* '<Root>/Roll_angle_feedback' */
  real_T ROLL_LQR_K1;                  /* '<Root>/ROLL_LQR_K1' */
  real_T ROLL_LQR_K2;                  /* '<Root>/ROLL_LQR_K2' */
  real_T ROLL_LIMIT;                   /* '<Root>/ROLL_LIMIT' */
} ExtU_Balanced_Infantry;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T R_chassis_current;            /* '<Root>/R_chassis_current' */
  real_T L_chassis_current;            /* '<Root>/L_chassis_current' */
  real_T out_L;                        /* '<Root>/out_L' */
  real_T out_R;                        /* '<Root>/out_R' */
  real_T Roll_offset_out;              /* '<Root>/Roll_offset_out' */
} ExtY_Balanced_Infantry;

/* Real-time Model Data Structure */
struct tag_RTM_Balanced_Infantry {
  const char_T *errorStatus;
};

/* External inputs (root inport signals with default storage) */
extern ExtU_Balanced_Infantry Balanced_Infantry_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_Balanced_Infantry Balanced_Infantry_Y;

/* Model entry point functions */
extern void Balanced_Infantry_initialize(void);
extern void Balanced_Infantry_step(void);

/* Real-time Model object */
extern RT_MODEL_Balanced_Infantry *const Balanced_Infantry_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'Balanced_Infantry'
 * '<S1>'   : 'Balanced_Infantry/If Action Subsystem'
 * '<S2>'   : 'Balanced_Infantry/If Action Subsystem1'
 * '<S3>'   : 'Balanced_Infantry/Land_LQR'
 * '<S4>'   : 'Balanced_Infantry/If Action Subsystem/MATLAB Function'
 * '<S5>'   : 'Balanced_Infantry/If Action Subsystem/MATLAB Function1'
 * '<S6>'   : 'Balanced_Infantry/If Action Subsystem/MATLAB Function2'
 * '<S7>'   : 'Balanced_Infantry/If Action Subsystem/MATLAB Function3'
 * '<S8>'   : 'Balanced_Infantry/If Action Subsystem1/MATLAB Function'
 */
#endif                                 /* RTW_HEADER_Balanced_Infantry_h_ */

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF] Balanced_Infantry.h
 */
