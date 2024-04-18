/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: isequal.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 17-Mar-2024 11:09:30
 */

/* Include Files */
#include "isequal.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : const emxArray_real_T *varargin_1
 *                const emxArray_real_T *varargin_2
 * Return Type  : boolean_T
 */
boolean_T b_isequal(const emxArray_real_T *varargin_1,
                    const emxArray_real_T *varargin_2)
{
  const double *varargin_1_data;
  const double *varargin_2_data;
  int k;
  boolean_T exitg1;
  boolean_T p;
  varargin_2_data = varargin_2->data;
  varargin_1_data = varargin_1->data;
  p = (varargin_1->size[1] == varargin_2->size[1]);
  if (p && (varargin_1->size[1] != 0) && (varargin_2->size[1] != 0)) {
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k <= varargin_2->size[1] - 1)) {
      if (!(varargin_1_data[k] == varargin_2_data[k])) {
        p = false;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }
  return p;
}

/*
 * Arguments    : const emxArray_real_T *varargin_1
 *                const emxArray_real_T *varargin_2
 * Return Type  : boolean_T
 */
boolean_T isequal(const emxArray_real_T *varargin_1,
                  const emxArray_real_T *varargin_2)
{
  const double *varargin_1_data;
  const double *varargin_2_data;
  int k;
  boolean_T exitg1;
  boolean_T p;
  varargin_2_data = varargin_2->data;
  varargin_1_data = varargin_1->data;
  p = false;
  if ((varargin_1->size[0] == varargin_2->size[0]) &&
      (varargin_1->size[1] == 1)) {
    p = true;
  }
  if (p && ((varargin_1->size[0] != 0) && (varargin_1->size[1] != 0)) &&
      (varargin_2->size[0] != 0)) {
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k <= varargin_2->size[0] - 1)) {
      if (!(varargin_1_data[k] == varargin_2_data[k])) {
        p = false;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }
  return p;
}

/*
 * File trailer for isequal.c
 *
 * [EOF]
 */
