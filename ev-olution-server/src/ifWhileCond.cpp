/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: ifWhileCond.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 31-Mar-2024 07:04:52
 */

/* Include Files */
#include "ifWhileCond.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : const boolean_T x_data[]
 *                int x_size
 * Return Type  : boolean_T
 */
boolean_T ifWhileCond(const boolean_T x_data[], int x_size)
{
  int k;
  boolean_T exitg1;
  boolean_T y;
  y = (x_size != 0);
  if (y) {
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k <= x_size - 1)) {
      if (!x_data[k]) {
        y = false;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }
  return y;
}

/*
 * File trailer for ifWhileCond.c
 *
 * [EOF]
 */
