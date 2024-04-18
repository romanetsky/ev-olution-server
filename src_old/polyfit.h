/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: polyfit.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 17-Mar-2024 11:09:30
 */

#ifndef POLYFIT_H
#define POLYFIT_H

/* Include Files */
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void b_polyfit(const double x_data[], const int x_size[2],
               const double y_data[], const int y_size[2], double p[2]);

void polyfit(const emxArray_real_T *x, const emxArray_real_T *y, double p[2]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for polyfit.h
 *
 * [EOF]
 */
