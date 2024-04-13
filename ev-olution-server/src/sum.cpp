/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: sum.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 08-Apr-2024 21:02:32
 */

/* Include Files */
#include "sum.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const double x_data[]
 *                const int x_size[2]
 *                double y_data[]
 *                int y_size[2]
 * Return Type  : void
 */
void b_sum(const double x_data[], const int x_size[2], double y_data[],
           int y_size[2])
{
  int i;
  int k;
  int vlen;
  int xi;
  int xpageoffset;
  vlen = x_size[0];
  if ((x_size[0] == 0) || (x_size[1] == 0)) {
    y_size[0] = 1;
    y_size[1] = (signed char)x_size[1];
    vlen = (signed char)x_size[1];
    if (vlen - 1 >= 0) {
      memset(&y_data[0], 0, (unsigned int)vlen * sizeof(double));
    }
  } else {
    i = x_size[1];
    y_size[0] = 1;
    y_size[1] = x_size[1];
    for (xi = 0; xi < i; xi++) {
      xpageoffset = xi * x_size[0];
      y_data[xi] = x_data[xpageoffset];
      for (k = 2; k <= vlen; k++) {
        y_data[xi] += x_data[(xpageoffset + k) - 1];
      }
    }
  }
}

/*
 * Arguments    : const double x_data[]
 *                const int x_size[2]
 * Return Type  : double
 */
double c_sum(const double x_data[], const int x_size[2])
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
  return y;
}

/*
 * Arguments    : const double x_data[]
 *                const int x_size[4]
 * Return Type  : double
 */
double sum(const double x_data[], const int x_size[4])
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
  return y;
}

/*
 * File trailer for sum.c
 *
 * [EOF]
 */
