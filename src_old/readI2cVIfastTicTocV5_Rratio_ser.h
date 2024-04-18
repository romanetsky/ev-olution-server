/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: readI2cVIfastTicTocV5_Rratio_ser.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 17-Mar-2024 11:09:30
 */

#ifndef READI2CVIFASTTICTOCV5_RRATIO_SER_H
#define READI2CVIFASTTICTOCV5_RRATIO_SER_H

/* Include Files */
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void c_readI2cVIfastTicTocV5_Rratio_(const signed char VIpacId[16],
                                     short N_bat1, emxArray_real_T *V,
                                     emxArray_real_T *b_I, double tV[16],
                                     double tI[16], double err_data[],
                                     int err_size[2], emxArray_real_T *Vdebug);

void d_readI2cVIfastTicTocV5_Rratio_(const signed char VIpacId[16],
                                     short N_bat1, emxArray_real_T *V,
                                     emxArray_real_T *b_I, double tV[16],
                                     double tI[16], double err_data[],
                                     int err_size[2], emxArray_real_T *Vdebug);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for readI2cVIfastTicTocV5_Rratio_ser.h
 *
 * [EOF]
 */
