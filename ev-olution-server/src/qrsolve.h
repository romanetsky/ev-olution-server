/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: qrsolve.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 09-Apr-2024 22:23:00
 */

#ifndef QRSOLVE_H
#define QRSOLVE_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
int qrsolve(const double A_data[], const int A_size[2], const double B_data[],
            int B_size, double Y[2]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for qrsolve.h
 *
 * [EOF]
 */
