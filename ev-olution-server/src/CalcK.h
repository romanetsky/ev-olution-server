/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: CalcK.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 03-Apr-2024 21:30:55
 */

#ifndef CALCK_H
#define CALCK_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void CalcK(const double VdebugN_data[], const int VdebugN_size[2],
           const double VdebugVec_data[], const int VdebugVec_size[2],
           const signed char VIpacId[16], const unsigned char Pac2Vid0[16],
           const double VmKp184Test_data[], int VmKp184Test_size,
           double k_data[], int k_size[2]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for CalcK.h
 *
 * [EOF]
 */
