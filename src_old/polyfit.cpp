/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: polyfit.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 17-Mar-2024 11:09:30
 */

/* Include Files */
#include "polyfit.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_emxutil.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h"
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
  emxArray_real_T b_V_data;
  emxArray_real_T b_y_data;
  double V_data[254];
  int V_size[2];
  int k;
  int rr;
  int y;
  V_size[0] = x_size[1];
  V_size[1] = 2;
  if (x_size[1] != 0) {
    rr = x_size[1];
    for (k = 0; k < rr; k++) {
      V_data[k + V_size[0]] = 1.0;
      V_data[k] = x_data[k];
    }
  }
  y = y_size[1];
  b_V_data.data = &V_data[0];
  b_V_data.size = &V_size[0];
  b_V_data.allocatedSize = 254;
  b_V_data.numDimensions = 2;
  b_V_data.canFreeData = false;
  b_y_data.data = (double *)&y_data[0];
  b_y_data.size = &y;
  b_y_data.allocatedSize = -1;
  b_y_data.numDimensions = 1;
  b_y_data.canFreeData = false;
  qrsolve(&b_V_data, &b_y_data, p);
}

/*
 * Arguments    : const emxArray_real_T *x
 *                const emxArray_real_T *y
 *                double p[2]
 * Return Type  : void
 */
void polyfit(const emxArray_real_T *x, const emxArray_real_T *y, double p[2])
{
  emxArray_real_T *V;
  const double *x_data;
  double *V_data;
  int k;
  int rr;
  x_data = x->data;
  emxInit_real_T(&V, 2);
  rr = V->size[0] * V->size[1];
  V->size[0] = x->size[0];
  V->size[1] = 2;
  emxEnsureCapacity_real_T(V, rr);
  V_data = V->data;
  if (x->size[0] != 0) {
    rr = x->size[0];
    for (k = 0; k < rr; k++) {
      V_data[k + V->size[0]] = 1.0;
      V_data[k] = x_data[k];
    }
  }
  qrsolve(V, y, p);
  emxFree_real_T(&V);
}

/*
 * File trailer for polyfit.c
 *
 * [EOF]
 */
