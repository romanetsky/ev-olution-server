/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: polyfit.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 09-Apr-2024 22:23:00
 */

/* Include Files */
#include "polyfit.h"
#include "qrsolve.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : const double x_data[]
 *                const int x_size[2]
 *                const double y_data[]
 *                const int y_size[2]
 *                double p[2]
 * Return Type  : void
 */
void b_polyfit(const double x_data[], const int x_size[2],
               const double y_data[], const int y_size[2], double p[2])
{
  double V_data[32];
  int V_size[2];
  int k;
  int rr;
  V_size[0] = x_size[1];
  V_size[1] = 2;
  if (x_size[1] != 0) {
    rr = x_size[1];
    for (k = 0; k < rr; k++) {
      V_data[k + V_size[0]] = 1.0;
      V_data[k] = x_data[k];
    }
  }
  qrsolve(V_data, V_size, y_data, y_size[1], p);
}

/*
 * Arguments    : const double x_data[]
 *                int x_size
 *                const double y_data[]
 *                int y_size
 *                double p[2]
 * Return Type  : void
 */
void polyfit(const double x_data[], int x_size, const double y_data[],
             int y_size, double p[2])
{
  double V_data[4096];
  int V_size[2];
  int k;
  V_size[0] = x_size;
  V_size[1] = 2;
  if (x_size != 0) {
    for (k = 0; k < x_size; k++) {
      V_data[k + x_size] = 1.0;
      V_data[k] = x_data[k];
    }
  }
  qrsolve(V_data, V_size, y_data, y_size, p);
}

/*
 * File trailer for polyfit.c
 *
 * [EOF]
 */
