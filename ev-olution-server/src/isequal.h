/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: isequal.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 08-Apr-2024 21:02:32
 */

#ifndef ISEQUAL_H
#define ISEQUAL_H

/* Include Files */
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
boolean_T b_isequal(const emxArray_real_T *varargin_1,
                    const emxArray_real_T *varargin_2);

boolean_T isequal(const double varargin_1_data[], const int varargin_1_size[2],
                  const double varargin_2_data[], int varargin_2_size);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for isequal.h
 *
 * [EOF]
 */
