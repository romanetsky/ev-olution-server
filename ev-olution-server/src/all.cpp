/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: all.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 31-Mar-2024 07:04:52
 */

/* Include Files */
#include "all.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : const boolean_T x_data[]
 *                int x_size
 * Return Type  : boolean_T
 */
boolean_T all(const boolean_T x_data[], int x_size)
{
  int ix;
  boolean_T exitg1;
  boolean_T y;
  y = true;
  ix = 1;
  exitg1 = false;
  while ((!exitg1) && (ix <= x_size)) {
    if (!x_data[ix - 1]) {
      y = false;
      exitg1 = true;
    } else {
      ix++;
    }
  }
  return y;
}

/*
 * File trailer for all.c
 *
 * [EOF]
 */
