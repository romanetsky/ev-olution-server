/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: BuildBitCnfg.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 17-Mar-2024 11:09:30
 */

#ifndef BUILDBITCNFG_H
#define BUILDBITCNFG_H

/* Include Files */
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
short BuildBitCnfg(short N_bat1, short N_bat2, double N_bat, short NbatMax,
                   emxArray_uint8_T *bitCnfg, char bitCnfgStr[2]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for BuildBitCnfg.h
 *
 * [EOF]
 */
