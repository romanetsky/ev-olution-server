/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: polyfit.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 09-Apr-2024 22:23:00
 */

#ifndef POLYFIT_H
#define POLYFIT_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void b_polyfit(const double x_data[], const int x_size[2],
               const double y_data[], const int y_size[2], double p[2]);

void polyfit(const double x_data[], int x_size, const double y_data[],
             int y_size, double p[2]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for polyfit.h
 *
 * [EOF]
 */
