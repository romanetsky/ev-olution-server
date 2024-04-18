/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: ReadVI.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 17-Mar-2024 11:09:30
 */

#ifndef READVI_H
#define READVI_H

/* Include Files */
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
double ReadVI(signed char ProjectFlag, const signed char VIpacId[16],
              const emxArray_uint8_T *Pac2Vid0All, short N_bat1, double N_bat,
              const float pIacs758[2], signed char Iacs758Flag,
              const emxArray_real_T *pIshunt, const emxArray_real32_T *Rwire,
              emxArray_real32_T *Vbat, emxArray_real_T *Vbat0,
              emxArray_real_T *Ibat, emxArray_real32_T *VbatMat,
              emxArray_real_T *VbatMat0, emxArray_real_T *Ishunt,
              emxArray_real_T *tV, emxArray_real_T *tV1, emxArray_real_T *tI,
              emxArray_real_T *tI1, emxArray_real_T *Vdebug0,
              double errI2C_data[], int errI2C_size[2], boolean_T *keepMeas);

boolean_T
b_ReadVI(double Nina219, signed char ProjectFlag, const signed char VIpacId[32],
         const emxArray_uint8_T *Pac2Vid0All, short N_bat1, double N_bat,
         const emxArray_real32_T *pIacs758, signed char Iacs758Flag,
         const emxArray_real_T *pIshunt, const emxArray_real32_T *Rwire,
         emxArray_real32_T *Vbat, emxArray_real_T *Vbat0, emxArray_real_T *Ibat,
         emxArray_real32_T *VbatMat, emxArray_real_T *VbatMat0,
         emxArray_real_T *Ishunt, emxArray_real_T *tV, emxArray_real_T *tV1,
         emxArray_real_T *tI, emxArray_real_T *tI1, emxArray_real_T *Iacs758,
         emxArray_real_T *Vdebug0, double errI2C_data[], int errI2C_size[2]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for ReadVI.h
 *
 * [EOF]
 */
