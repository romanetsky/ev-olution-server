/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: CalcStructKalman.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 31-Mar-2024 07:04:52
 */

#ifndef CALCSTRUCTKALMAN_H
#define CALCSTRUCTKALMAN_H

/* Include Files */
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void CalcStructKalman(struct22_T *StructKalman, float Ta, emxArray_real_T *t,
                      const emxArray_real_T *i_in,
                      const emxArray_real_T *v_out);

void b_CalcStructKalman(struct22_T *StructKalman, float Ta, double t_data[],
                        const int t_size[2], const double i_in_data[],
                        const double v_out_data[]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for CalcStructKalman.h
 *
 * [EOF]
 */
