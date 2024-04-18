/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: CalcK.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 31-Mar-2024 07:04:52
 */

#ifndef CALCK_H
#define CALCK_H

/* Include Files */
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void CalcK(const emxArray_real_T *VdebugN, const double VdebugVec_data[],
           const int VdebugVec_size[2], const signed char VIpacId[16],
           const unsigned char Pac2Vid0[16], const double VmKp184Test_data[],
           int VmKp184Test_size, emxArray_real_T *k);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for CalcK.h
 *
 * [EOF]
 */
