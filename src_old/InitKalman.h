/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: InitKalman.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 17-Mar-2024 11:09:30
 */

#ifndef INITKALMAN_H
#define INITKALMAN_H

/* Include Files */
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void InitKalman(float tV1[16], float BatState[496], const float BatParams[608],
                const float itStart[16], float Ta, double Vbat,
                struct22_T *KalmanStruct);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for InitKalman.h
 *
 * [EOF]
 */
