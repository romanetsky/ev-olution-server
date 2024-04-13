/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: CellKalman.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 17-Mar-2024 11:09:30
 */

#ifndef CELLKALMAN_H
#define CELLKALMAN_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
float CellKalman(const float paramsBatt[38], const float stateBatt_k_k[31],
                 float P_k_k, float Ta, double t, float dt, double i_in,
                 float Q, float R, double z_k1, float eps1,
                 float stateBatt_k1_k1[31]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for CellKalman.h
 *
 * [EOF]
 */
