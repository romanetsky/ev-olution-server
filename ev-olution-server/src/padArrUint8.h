/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: padArrUint8.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 13-Apr-2024 21:34:04
 */

#ifndef PADARRUINT8_H
#define PADARRUINT8_H

/* Include Files */
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void b_padArrUint8(const emxArray_uint8_T *Ain, unsigned char B[256]);

void c_padArrUint8(const unsigned char Ain_data[], int Ain_size, short N1,
                   short N2, emxArray_uint8_T *B);

void d_padArrUint8(const unsigned char Ain[2], short N1, short N2,
                   emxArray_uint8_T *B);

void e_padArrUint8(const unsigned char Ain[256], double N1, double N2,
                   emxArray_uint8_T *B);

void f_padArrUint8(unsigned char Ain, double N1, double N2,
                   emxArray_uint8_T *B);

void padArrUint8(unsigned char Ain, short N1, short N2, emxArray_uint8_T *B);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for padArrUint8.h
 *
 * [EOF]
 */
