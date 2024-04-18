/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: sort.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 17-Mar-2024 11:09:30
 */

#ifndef SORT_H
#define SORT_H

/* Include Files */
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void b_sort(short x_data[], const int *x_size);

void c_sort(emxArray_real_T *x, emxArray_int32_T *idx);

void d_sort(emxArray_real_T *x, emxArray_int32_T *idx);

void e_sort(emxArray_real_T *x);

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
