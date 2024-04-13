/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: sortIdx.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Apr-2024 17:01:48
 */

#ifndef SORTIDX_H
#define SORTIDX_H

/* Include Files */
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void b_merge(int idx_data[], double x_data[], int offset, int np, int nq,
             int iwork_data[], double xwork_data[]);

void merge_block(emxArray_int32_T *idx, short x_data[], int offset, int n,
                 int preSortLevel, emxArray_int32_T *iwork, short xwork_data[]);

int sortIdx(double x_data[], const int *x_size, int idx_data[]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for sortIdx.h
 *
 * [EOF]
 */
