/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: CellKalman.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 17-Mar-2024 11:09:30
 */

/* Include Files */
#include "CellKalman.h"
#include "dischargeCell_array.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * z_k1 - the measurement. for example:[Vb] [Vb T]
 *  Q diag([0.1^2 0.001^2 0.02^2]);  the covariance of the process (model) noise
 *  R diag([0.01^2]) the covariance of the observation noise
 *
 * Arguments    : const float paramsBatt[38]
 *                const float stateBatt_k_k[31]
 *                float P_k_k
 *                float Ta
 *                double t
 *                float dt
 *                double i_in
 *                float Q
 *                float R
 *                double z_k1
 *                float eps1
 *                float stateBatt_k1_k1[31]
 * Return Type  : float
 */
float CellKalman(const float paramsBatt[38], const float stateBatt_k_k[31],
                 float P_k_k, float Ta, double t, float dt, double i_in,
                 float Q, float R, double z_k1, float eps1,
                 float stateBatt_k1_k1[31])
{
  double epsVecHelp[31];
  float b_stateBatt_k1_0[31];
  float b_stateBatt_k1_k_x_eps[31];
  float stateBatt_k1_0[31];
  float stateBatt_k1_k_x_eps[31];
  float P_k1_k;
  float P_k1_k1;
  float S_k1;
  float W_k1;
  int i;
  /*  id2X  = [1 2 3 5 6 7 10 15 16 17 18 20 21 22]';%x -
   * Temp,it,i_lp_n,DOD_n,DOD_n_m1,DOD_n_m2,Q_Ta_n,Tn,T_lp_n_m1,T_lp_n,Vb,R_T_n,Eb,SOC
   */
  /* [1 2]';%x - Temp,it;%[1 2 18]';%x - Temp,it,Vb */
  /*  Nx = length(id2X); */
  /*  h - Vb, (in the future Temp) */
  /*  x_k_k = stateBatt_k_k(id2X); */
  /*  P_k_k  */
  /*  eps1 = 1e-6; */
  memcpy(&stateBatt_k1_0[0], &stateBatt_k_k[0], 31U * sizeof(float));
  b_dischargeCell_array(paramsBatt, stateBatt_k1_0, Ta, t, dt, i_in);
  memset(&epsVecHelp[0], 0, 31U * sizeof(double));
  epsVecHelp[1] = eps1;
  for (i = 0; i < 31; i++) {
    stateBatt_k1_k_x_eps[i] = stateBatt_k_k[i] + (float)epsVecHelp[i];
  }
  b_dischargeCell_array(paramsBatt, stateBatt_k1_k_x_eps, Ta, t, dt, i_in);
  /*  eps1 = 1e-6; */
  memcpy(&b_stateBatt_k1_0[0], &stateBatt_k_k[0], 31U * sizeof(float));
  b_dischargeCell_array(paramsBatt, b_stateBatt_k1_0, Ta, t, dt, i_in);
  memset(&epsVecHelp[0], 0, 31U * sizeof(double));
  epsVecHelp[1] = eps1;
  for (i = 0; i < 31; i++) {
    b_stateBatt_k1_k_x_eps[i] = stateBatt_k_k[i] + (float)epsVecHelp[i];
  }
  b_dischargeCell_array(paramsBatt, b_stateBatt_k1_k_x_eps, Ta, t, dt, i_in);
  W_k1 = (b_stateBatt_k1_k_x_eps[17] - b_stateBatt_k1_0[17]) / eps1;
  P_k1_k = (stateBatt_k1_k_x_eps[1] - stateBatt_k1_0[1]) / eps1;
  P_k1_k = P_k1_k * P_k_k * P_k1_k + Q;
  S_k1 = W_k1 * P_k_k * W_k1 + R;
  W_k1 = P_k1_k * W_k1 * (1.0F / S_k1);
  P_k1_k1 = P_k1_k - W_k1 * S_k1 * W_k1;
  memcpy(&stateBatt_k1_k1[0], &stateBatt_k_k[0], 31U * sizeof(float));
  b_dischargeCell_array(paramsBatt, stateBatt_k1_k1, Ta, t, dt, i_in);
  stateBatt_k1_k1[1] += W_k1 * ((float)z_k1 - stateBatt_k1_k1[17]);
  /*  stateBatt_k1_k1 = stateBatt_k1_k1';%only for backward compatibily */
  /* stateBatt =
   * dischargeCell_array(paramsBatt,stateBatt,Ta,t,dt,i_in,Sel,AgingStepFlag) */
  return P_k1_k1;
}

/*
 * File trailer for CellKalman.c
 *
 * [EOF]
 */
