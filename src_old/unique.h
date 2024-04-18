/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: unique.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 17-Mar-2024 11:09:30
 */

#ifndef UNIQUE_H
#define UNIQUE_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void unique_rows(const double a_data[], const int a_size[2], double b_data[],
                 int b_size[2]);

int unique_vector(const short a_data[], int a_size, short b_data[],
                  int ndx_data[], int *ndx_size, int pos_data[], int *pos_size);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for unique.h
 *
 * [EOF]
 */
