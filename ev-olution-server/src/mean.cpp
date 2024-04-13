/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: mean.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 08-Apr-2024 21:02:32
 */

/* Include Files */
#include "mean.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : const double x_data[]
 *                const int x_size[4]
 * Return Type  : double
 */
double b_mean(const double x_data[], const int x_size[4])
{
  double y;
  int k;
  int vlen;
  vlen = x_size[3];
  if (x_size[3] == 0) {
    y = 0.0;
  } else {
    y = x_data[0];
    for (k = 2; k <= vlen; k++) {
      y += x_data[k - 1];
    }
  }
  y /= (double)x_size[3];
  return y;
}

/*
 * Arguments    : const double x_data[]
 *                const int x_size[2]
 * Return Type  : double
 */
double mean(const double x_data[], const int x_size[2])
{
  double y;
  int k;
  int vlen;
  vlen = x_size[1];
  if (x_size[1] == 0) {
    y = 0.0;
  } else {
    y = x_data[0];
    for (k = 2; k <= vlen; k++) {
      y += x_data[k - 1];
    }
  }
  y /= (double)x_size[1];
  return y;
}

/*
 * File trailer for mean.c
 *
 * [EOF]
 */
