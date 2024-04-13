/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: squeeze.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 08-Apr-2024 21:02:32
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
void b_squeeze(const double a_data[], const int a_size[4], emxArray_real_T *b);

void c_squeeze(const unsigned char a_data[], const int a_size[3],
               unsigned char b_data[], int b_size[2]);

void squeeze(const double a_data[], const int a_size[3], double b_data[],
             int b_size[2]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for squeeze.h
 *
 * [EOF]
 */
