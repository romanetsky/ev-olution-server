/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: sum.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 08-Apr-2024 21:02:32
 */

#ifndef SUM_H
#define SUM_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void b_sum(const double x_data[], const int x_size[2], double y_data[],
           int y_size[2]);

double c_sum(const double x_data[], const int x_size[2]);

double sum(const double x_data[], const int x_size[4]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for sum.h
 *
 * [EOF]
 */
