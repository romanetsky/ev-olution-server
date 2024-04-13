/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: mod.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 17-Mar-2024 11:09:30
 */

/* Include Files */
#include "mod.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : double x
 *                float y
 * Return Type  : float
 */
float b_mod(double x, float y)
{
  float q;
  float r;
  boolean_T rEQ0;
  r = (float)x;
  if (y == 0.0F) {
    if (x == 0.0) {
      r = y;
    }
  } else if (rtIsNaN(x) || rtIsNaNF(y) || rtIsInf(x)) {
    r = rtNaNF;
  } else if (x == 0.0) {
    r = 0.0F / y;
  } else if (rtIsInfF(y)) {
    if ((y < 0.0F) != (x < 0.0)) {
      r = y;
    }
  } else {
    r = (float)fmod((float)x, y);
    rEQ0 = (r == 0.0F);
    if ((!rEQ0) && (y > (float)floor(y))) {
      q = (float)fabs((float)x / y);
      rEQ0 = !((float)fabs(q - (float)floor(q + 0.5F)) > 1.1920929E-7F * q);
    }
    if (rEQ0) {
      r = y * 0.0F;
    } else if ((x < 0.0) != (y < 0.0F)) {
      r += y;
    }
  }
  return r;
}

/*
 * File trailer for mod.c
 *
 * [EOF]
 */
