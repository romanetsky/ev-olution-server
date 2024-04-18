/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: useConstantDim.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 17-Mar-2024 11:09:30
 */

/* Include Files */
#include "useConstantDim.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : emxArray_real_T *varargin_2
 * Return Type  : void
 */
void b_useConstantDim(emxArray_real_T *varargin_2)
{
  double *varargin_2_data;
  int b_k;
  int i;
  int i1;
  int k;
  varargin_2_data = varargin_2->data;
  if (varargin_2->size[0] != 0) {
    i = (unsigned short)(varargin_2->size[1] - 1);
    for (k = 0; k < i; k++) {
      i1 = varargin_2->size[0];
      for (b_k = 0; b_k < i1; b_k++) {
        varargin_2_data[b_k + varargin_2->size[0] * (k + 1)] +=
            varargin_2_data[b_k + varargin_2->size[0] * k];
      }
    }
  }
}

/*
 * Arguments    : emxArray_real_T *varargin_2
 *                int varargin_3
 * Return Type  : void
 */
void useConstantDim(emxArray_real_T *varargin_2, int varargin_3)
{
  double *varargin_2_data;
  int i;
  int k;
  varargin_2_data = varargin_2->data;
  if ((varargin_3 == 1) && (varargin_2->size[0] != 0) &&
      (varargin_2->size[0] != 1)) {
    i = varargin_2->size[0];
    for (k = 0; k <= i - 2; k++) {
      varargin_2_data[k + 1] += varargin_2_data[k];
    }
  }
}

/*
 * File trailer for useConstantDim.c
 *
 * [EOF]
 */
