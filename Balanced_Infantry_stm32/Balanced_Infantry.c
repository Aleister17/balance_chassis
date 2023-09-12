/*
 * File: Balanced_Infantry.c
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

#include "Balanced_Infantry.h"
#include "Balanced_Infantry_private.h"

/* External inputs (root inport signals with default storage) */
ExtU_Balanced_Infantry Balanced_Infantry_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_Balanced_Infantry Balanced_Infantry_Y;

/* Real-time model */
RT_MODEL_Balanced_Infantry Balanced_Infantry_M_;
RT_MODEL_Balanced_Infantry *const Balanced_Infantry_M = &Balanced_Infantry_M_;

/*
 * Output and update for atomic system:
 *    '<S1>/MATLAB Function'
 *    '<S1>/MATLAB Function2'
 */
void Balanced_Infantry_MATLABFunction(real_T rtu_speed, real_T rtu_angle, real_T
  *rty_y1, real_T *rty_y2)
{
  if (rtu_speed * rtu_angle > 0.0) {
    *rty_y1 = rtu_speed;
    *rty_y2 = 0.0;
  } else {
    *rty_y2 = rtu_speed;
    *rty_y1 = 0.0;
  }
}

/*
 * Output and update for atomic system:
 *    '<S1>/MATLAB Function1'
 *    '<S1>/MATLAB Function3'
 */
void Balanced_Infantry_MATLABFunction1(real_T rtu_angle, real_T rtu_k1, real_T
  rtu_k2, real_T rtu_k3, real_T rtu_x, real_T *rty_out)
{
  *rty_out = 0.0;
  if ((rtu_angle > 0.0) && (rtu_angle < rtu_x)) {
    *rty_out = rtu_angle * rtu_k1;
  } else if (rtu_angle <= 0.0) {
    *rty_out = rtu_angle * rtu_k3;
  } else {
    if (rtu_angle >= rtu_x) {
      *rty_out = (rtu_angle - rtu_x) * rtu_k2 + rtu_k1 * rtu_x;
    }
  }
}

/* Model step function */
void Balanced_Infantry_step(void)
{
  real_T rtb_y1;
  real_T rtb_out;
  real_T rtb_y2;
  if (Balanced_Infantry_U.LQR_enable == 1.0) {
    Balanced_Infantry_Y.L_chassis_current =
      -((Balanced_Infantry_U.L_angle_feedback - Balanced_Infantry_U.L_angle_set)
        * Balanced_Infantry_U.PITCH_LQR_K1 + Balanced_Infantry_U.PITCH_LQR_K2 *
        Balanced_Infantry_U.L_angular_speed_feedback);
    Balanced_Infantry_Y.R_chassis_current =
      -((Balanced_Infantry_U.R_angle_feedback - Balanced_Infantry_U.R_angle_set)
        * Balanced_Infantry_U.PITCH_LQR_K1 + Balanced_Infantry_U.PITCH_LQR_K2 *
        Balanced_Infantry_U.R_angular_speed_feedback);
  }

  if (Balanced_Infantry_U.Suspension_enable == 1.0) {
    Balanced_Infantry_MATLABFunction(Balanced_Infantry_U.speed_L,
      Balanced_Infantry_U.angle_L, &rtb_y1, &rtb_out);
    Balanced_Infantry_MATLABFunction1(Balanced_Infantry_U.angle_L,
      Balanced_Infantry_U.Suspension_K1, Balanced_Infantry_U.Suspension_K2,
      Balanced_Infantry_U.Suspension_K3, Balanced_Infantry_U.Suspension_x,
      &rtb_y2);
    Balanced_Infantry_Y.out_L = ((0.0 - rtb_y1 *
      Balanced_Infantry_U.Suspension_press) - rtb_out *
      Balanced_Infantry_U.Suspension_rebound) - rtb_y2;
    Balanced_Infantry_MATLABFunction(Balanced_Infantry_U.speed_R,
      Balanced_Infantry_U.angle_R, &rtb_y1, &rtb_y2);
    Balanced_Infantry_MATLABFunction1(Balanced_Infantry_U.angle_R,
      Balanced_Infantry_U.Suspension_K1, Balanced_Infantry_U.Suspension_K2,
      Balanced_Infantry_U.Suspension_K3, Balanced_Infantry_U.Suspension_x,
      &rtb_out);
    Balanced_Infantry_Y.out_R = ((0.0 - rtb_y1 *
      Balanced_Infantry_U.Suspension_press) - rtb_y2 *
      Balanced_Infantry_U.Suspension_rebound) - rtb_out;
  }

  if (Balanced_Infantry_U.Roll_stabilize_enable == 1.0) {
    rtb_y1 = Balanced_Infantry_U.ROLL_LQR_K1 *
      Balanced_Infantry_U.Roll_angle_feedback + Balanced_Infantry_U.ROLL_LQR_K2 *
      Balanced_Infantry_U.Roll_angular_speed_feedback;
    Balanced_Infantry_Y.Roll_offset_out = 0.0;
    if (fabs(-rtb_y1) <= Balanced_Infantry_U.ROLL_LIMIT) {
      Balanced_Infantry_Y.Roll_offset_out = -rtb_y1;
    } else if (-rtb_y1 > Balanced_Infantry_U.ROLL_LIMIT) {
      Balanced_Infantry_Y.Roll_offset_out = Balanced_Infantry_U.ROLL_LIMIT;
    } else {
      if (-rtb_y1 < -Balanced_Infantry_U.ROLL_LIMIT) {
        Balanced_Infantry_Y.Roll_offset_out = -Balanced_Infantry_U.ROLL_LIMIT;
      }
    }
  }
}

/* Model initialize function */
void Balanced_Infantry_initialize(void)
{
  /* (no initialization code required) */
}

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF] Balanced_Infantry.c
 */
