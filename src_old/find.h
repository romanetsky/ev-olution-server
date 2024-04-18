/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: find.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 31-Mar-2024 07:04:52
 */

#ifndef FIND_H
#define FIND_H

/* Include Files */
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
int b_eml_find(const boolean_T x[6], int i_data[]);

int c_eml_find(const boolean_T x_data[], int x_size, int i_data[]);

int d_eml_find(const emxArray_boolean_T *x, int i_data[]);

void e_eml_find(const boolean_T x_data[], const int x_size[2], int i_data[],
                int i_size[2]);

int eml_find(const boolean_T x[257], int i_data[]);

void f_eml_find(const emxArray_boolean_T *x, int i_data[], int i_size[2]);

int g_eml_find(const boolean_T x_data[], int x_size, int i_data[]);

int h_eml_find(const boolean_T x_data[], int x_size, int i_data[]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for find.h
 *
 * [EOF]
 */
