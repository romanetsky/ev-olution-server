/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: readI2cVIfastTicTocV5_Rratio_ser.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 10-Apr-2024 21:46:28
 */

#ifndef READI2CVIFASTTICTOCV5_RRATIO_SER_H
#define READI2CVIFASTTICTOCV5_RRATIO_SER_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void c_readI2cVIfastTicTocV5_Rratio_(
    const signed char VIpacId[16], short N_bat1, double V_data[], int V_size[2],
    double I_data[], int I_size[2], double tV_data[], int tV_size[2],
    double tI_data[], int tI_size[2], double err_data[], int err_size[2],
    double Vdebug_data[], int Vdebug_size[2]);

void d_readI2cVIfastTicTocV5_Rratio_(
    const signed char VIpacId[16], short N_bat1, double V_data[], int V_size[2],
    double I_data[], int I_size[2], double tV_data[], int tV_size[2],
    double tI_data[], int tI_size[2], double err_data[], int err_size[2],
    double Vdebug_data[], int Vdebug_size[2]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for readI2cVIfastTicTocV5_Rratio_ser.h
 *
 * [EOF]
 */
