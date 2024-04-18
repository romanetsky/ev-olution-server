/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: minus.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 17-Mar-2024 11:09:30
 */

/* Include Files */
#include "minus.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : const creal_T a
 *                const creal_T b
 * Return Type  : creal_T
 */
creal_T minus(const creal_T a, const creal_T b)
{
  creal_T cout;
  double ahi;
  double b_slo;
  double bb;
  double shi;
  double shi_tmp;
  double slo;
  shi_tmp = a.re - b.re;
  bb = shi_tmp - a.re;
  slo = (a.re - (shi_tmp - bb)) - (b.re + bb);
  if (rtIsNaN(slo)) {
    slo = 0.0;
  }
  shi = a.im - b.im;
  bb = shi - a.im;
  b_slo = (a.im - (shi - bb)) - (b.im + bb);
  if (rtIsNaN(b_slo)) {
    b_slo = 0.0;
  }
  ahi = shi_tmp;
  if (shi != 0.0) {
    slo += shi;
    ahi = shi_tmp + slo;
    slo -= ahi - shi_tmp;
  }
  if (rtIsNaN(slo)) {
    slo = 0.0;
  }
  bb = ahi;
  if (b_slo != 0.0) {
    slo += b_slo;
    bb = ahi + slo;
    slo -= bb - ahi;
  }
  if (rtIsNaN(slo)) {
    slo = 0.0;
  }
  cout.re = bb;
  cout.im = slo;
  return cout;
}

/*
 * File trailer for minus.c
 *
 * [EOF]
 */
