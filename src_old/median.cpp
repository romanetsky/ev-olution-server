/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: median.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 31-Mar-2024 07:04:52
 */

/* Include Files */
#include "median.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const double x_data[]
 *                const int x_size[2]
 *                double y_data[]
 * Return Type  : int
 */
int median(const double x_data[], const int x_size[2], double y_data[])
{
  double xv_data[3];
  int exitg1;
  int j;
  int loop_ub;
  int n;
  int y_size;
  if (x_size[1] == 0) {
    y_size = 1;
    y_data[0] = rtNaN;
  } else {
    y_size = 1;
    y_data[0] = 0.0;
    loop_ub = x_size[1];
    n = x_size[1];
    memcpy(&xv_data[0], &x_data[0], (unsigned int)loop_ub * sizeof(double));
    for (j = 0; j < 1; j++) {
      loop_ub = 0;
      do {
        exitg1 = 0;
        if (loop_ub <= n - 1) {
          if (rtIsNaN(xv_data[loop_ub])) {
            y_data[0] = rtNaN;
            exitg1 = 1;
          } else {
            loop_ub++;
          }
        } else {
          if (n == 1) {
            y_data[0] = xv_data[0];
          } else if (n == 2) {
            if (rtIsInf(xv_data[0])) {
              y_data[0] = (xv_data[0] + xv_data[1]) / 2.0;
            } else {
              y_data[0] = xv_data[0] + (xv_data[1] - xv_data[0]) / 2.0;
            }
          } else {
            if (xv_data[0] < xv_data[1]) {
              if (xv_data[1] < xv_data[2]) {
                loop_ub = 1;
              } else if (xv_data[0] < xv_data[2]) {
                loop_ub = 2;
              } else {
                loop_ub = 0;
              }
            } else if (xv_data[0] < xv_data[2]) {
              loop_ub = 0;
            } else if (xv_data[1] < xv_data[2]) {
              loop_ub = 2;
            } else {
              loop_ub = 1;
            }
            y_data[0] = xv_data[loop_ub];
          }
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }
  }
  return y_size;
}

/*
 * File trailer for median.c
 *
 * [EOF]
 */
