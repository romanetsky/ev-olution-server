/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: times.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 17-Mar-2024 11:09:30
 */

/* Include Files */
#include "times.h"
#include "rt_nonfinite.h"
#include "split.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : const creal_T a
 * Return Type  : creal_T
 */
creal_T times(const creal_T a)
{
  creal_T c;
  creal_T da;
  double ahi;
  double b;
  double shi_tmp;
  double slo;
  da = split(a.re);
  shi_tmp = a.re * 8.64E+7;
  slo = da.im * 8.64E+7 + (da.re * 8.64E+7 - shi_tmp);
  if (rtIsNaN(slo)) {
    slo = 0.0;
  }
  b = a.im * 8.64E+7;
  ahi = shi_tmp;
  if (b != 0.0) {
    slo += b;
    ahi = shi_tmp + slo;
    slo -= ahi - shi_tmp;
  }
  if (rtIsNaN(slo)) {
    slo = 0.0;
  }
  c.re = ahi;
  c.im = slo;
  return c;
}

/*
 * File trailer for times.c
 *
 * [EOF]
 */
