/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_emxAPI.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 17-Mar-2024 11:09:30
 */

#ifndef CHR_DIS_KP184_KA6005P_JUNTEK_ESP32SER_20240229_EMXAPI_H
#define CHR_DIS_KP184_KA6005P_JUNTEK_ESP32SER_20240229_EMXAPI_H

/* Include Files */
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern emxArray_real_T *emxCreateND_real_T(int numDimensions, const int *size);

extern emxArray_real_T *
emxCreateWrapperND_real_T(double *data, int numDimensions, const int *size);

extern emxArray_real_T *emxCreateWrapper_real_T(double *data, int rows,
                                                int cols);

extern emxArray_real_T *emxCreate_real_T(int rows, int cols);

extern void emxDestroyArray_real_T(emxArray_real_T *emxArray);

extern void emxDestroy_struct30_T(struct30_T emxArray);

extern void emxInit_struct30_T(struct30_T *pStruct);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_emxAPI.h
 *
 * [EOF]
 */
