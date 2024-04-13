/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: padArr.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 08-Apr-2024 21:02:32
 */

#ifndef PADARR_H
#define PADARR_H

/* Include Files */
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void b_padArr(unsigned char A, double N1, double N2, emxArray_real_T *B);

void padArr(const double A_data[], int A_size, double N1, double N2,
            emxArray_real_T *B);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for padArr.h
 *
 * [EOF]
 */
