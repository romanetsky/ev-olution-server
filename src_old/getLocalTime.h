/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: getLocalTime.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 17-Mar-2024 11:09:30
 */

#ifndef GETLOCALTIME_H
#define GETLOCALTIME_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
double getLocalTime(double *t_tm_sec, double *t_tm_min, double *t_tm_hour,
                    double *t_tm_mday, double *t_tm_mon, double *t_tm_year,
                    boolean_T *t_tm_isdst);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for getLocalTime.h
 *
 * [EOF]
 */
