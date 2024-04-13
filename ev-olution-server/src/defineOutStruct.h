/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: defineOutStruct.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 17-Mar-2024 11:09:30
 */

#ifndef DEFINEOUTSTRUCT_H
#define DEFINEOUTSTRUCT_H

/* Include Files */
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
boolean_T defineOutStruct(short NmaxBrd, short NmaxBat,
                          emxArray_real_T *outStruct_VbusTest,
                          emxArray_real_T *outStruct_VmKp184Test,
                          boolean_T *outStruct_VresetFlag,
                          double *outStruct_Rint);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for defineOutStruct.h
 *
 * [EOF]
 */
