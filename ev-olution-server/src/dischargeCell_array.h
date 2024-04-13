/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: dischargeCell_array.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 17-Mar-2024 11:09:30
 */

#ifndef DISCHARGECELL_ARRAY_H
#define DISCHARGECELL_ARRAY_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void b_dischargeCell_array(const float paramsBatt[38], float stateBatt[31],
                           float Ta, double t, float dt, double i_in);

void dischargeCell_array(const float paramsBatt[38], float stateBatt[31],
                         float Ta);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for dischargeCell_array.h
 *
 * [EOF]
 */
