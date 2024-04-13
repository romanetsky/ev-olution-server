/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: sort.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 08-Apr-2024 21:02:32
 */

#ifndef SORT_H
#define SORT_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void b_sort(short x_data[], const int *x_size);

int c_sort(double x_data[], const int *x_size, int idx_data[]);

int d_sort(double x_data[], const int *x_size, int idx_data[]);

void e_sort(double x_data[], const int *x_size);

void sort(short x[2]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for sort.h
 *
 * [EOF]
 */
