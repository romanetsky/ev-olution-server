/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: combineVectorElements.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 08-Apr-2024 21:02:32
 */

/* Include Files */
#include "combineVectorElements.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_emxutil.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : const emxArray_real_T *x
 *                emxArray_real_T *y
 * Return Type  : void
 */
void combineVectorElements(const emxArray_real_T *x, emxArray_real_T *y)
{
  const double *x_data;
  double *y_data;
  int k;
  int vlen;
  int vstride;
  int xj;
  int xoffset;
  x_data = x->data;
  vlen = x->size[1];
  if ((x->size[0] == 0) || (x->size[1] == 0)) {
    xj = y->size[0];
    y->size[0] = x->size[0];
    emxEnsureCapacity_real_T(y, xj);
    y_data = y->data;
    xoffset = x->size[0];
    for (xj = 0; xj < xoffset; xj++) {
      y_data[xj] = 0.0;
    }
  } else {
    vstride = x->size[0];
    xj = y->size[0];
    y->size[0] = x->size[0];
    emxEnsureCapacity_real_T(y, xj);
    y_data = y->data;
    for (xj = 0; xj < vstride; xj++) {
      y_data[xj] = x_data[xj];
    }
    for (k = 2; k <= vlen; k++) {
      xoffset = (k - 1) * vstride;
      for (xj = 0; xj < vstride; xj++) {
        y_data[xj] += x_data[xoffset + xj];
      }
    }
  }
}

/*
 * File trailer for combineVectorElements.c
 *
 * [EOF]
 */
