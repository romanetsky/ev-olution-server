/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: InitKalman.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 17-Mar-2024 11:09:30
 */

/* Include Files */
#include "InitKalman.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * UNTITLED3 Summary of this function goes here
 *    Detailed explanation goes here
 * Ta- repmat(Ta,Nbat,1);
 *
 * Arguments    : float tV1[16]
 *                float BatState[496]
 *                const float BatParams[608]
 *                const float itStart[16]
 *                float Ta
 *                double Vbat
 *                struct22_T *KalmanStruct
 * Return Type  : void
 */
void InitKalman(float tV1[16], float BatState[496], const float BatParams[608],
                const float itStart[16], float Ta, double Vbat,
                struct22_T *KalmanStruct)
{
  int i;
  KalmanStruct->eps1 = 1.0E-10F;
  KalmanStruct->Qkalman = 0.0001F;
  /* diag([0.1^2 0.01^2 0.02^2]);% diag([0.1^2 0.001^2 0.02^2]);  the covariance
   * of the process (model) noise */
  KalmanStruct->Rkalman = 0.0001F;
  /* diag([0.01^2]) the covariance of the observation noise */
  /*  KalmanStruct.P_k_k   = zeros(1,1,Nbat); */
  for (i = 0; i < 16; i++) {
    tV1[i] /= 3600.0F;
    KalmanStruct->P_k_k[i] = 0.0001F;
    BatState[i + 32] = 0.0F;
  }
  for (i = 0; i < 48; i++) {
    BatState[i + 224] = Ta;
  }
  for (i = 0; i < 16; i++) {
    BatState[i + 272] = (float)Vbat;
    BatState[i + 288] = tV1[i];
    BatState[i + 16] = itStart[i];
  }
  memcpy(&KalmanStruct->BatState_k_k[0], &BatState[0], 496U * sizeof(float));
  memcpy(&KalmanStruct->BatParams[0], &BatParams[0], 608U * sizeof(float));
  KalmanStruct->N_bat = 16;
}

/*
 * File trailer for InitKalman.c
 *
 * [EOF]
 */
