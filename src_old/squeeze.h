/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: squeeze.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 17-Mar-2024 11:09:30
 */

#ifndef SQUEEZE_H
#define SQUEEZE_H

/* Include Files */
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void b_squeeze(const emxArray_real_T *a, emxArray_real_T *b);

void c_squeeze(const emxArray_uint8_T *a, emxArray_uint8_T *b);

void squeeze(const emxArray_real_T *a, emxArray_real_T *b);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for squeeze.h
 *
 * [EOF]
 */
