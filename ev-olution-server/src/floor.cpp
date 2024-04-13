/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: floor.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 17-Mar-2024 11:09:30
 */

/* Include Files */
#include "floor.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : const creal_T a
 * Return Type  : creal_T
 */
creal_T b_floor(const creal_T a)
{
  creal_T b;
  double ahi;
  double alo;
  double b_b;
  double b_tmp;
  b_tmp = floor(a.re);
  b.re = b_tmp;
  b.im = 0.0;
  if (b_tmp == a.re) {
    b_b = floor(a.im);
    alo = 0.0;
    ahi = b_tmp;
    if (b_b != 0.0) {
      ahi = b_tmp + b_b;
      alo = b_b - (ahi - b_tmp);
    }
    if (rtIsNaN(alo)) {
      alo = 0.0;
    }
    b.re = ahi;
    b.im = alo;
  }
  return b;
}

/*
 * File trailer for floor.c
 *
 * [EOF]
 */
