/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: split.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 17-Mar-2024 11:09:30
 */

/* Include Files */
#include "split.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : double a
 * Return Type  : creal_T
 */
creal_T split(double a)
{
  creal_T aout;
  double slo;
  double temp;
  if (fabs(a) <= 6.69692879491417E+299) {
    temp = 1.34217729E+8 * a;
    temp -= temp - a;
    aout.re = temp;
    aout.im = a - temp;
  } else if ((!rtIsInf(a)) && (!rtIsNaN(a))) {
    a *= 3.7252902984619141E-9;
    temp = 1.34217729E+8 * a;
    temp -= temp - a;
    slo = a - temp;
    temp *= 2.68435456E+8;
    slo *= 2.68435456E+8;
    aout.re = temp;
    aout.im = slo;
  } else {
    aout.re = a;
    aout.im = 0.0;
  }
  return aout;
}

/*
 * File trailer for split.c
 *
 * [EOF]
 */
