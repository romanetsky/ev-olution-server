/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: minOrMax.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 17-Mar-2024 11:09:30
 */

/* Include Files */
#include "minOrMax.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : const double x[2]
 * Return Type  : double
 */
double b_maximum(const double x[2])
{
  double ex;
  if ((x[0] < x[1]) || (rtIsNaN(x[0]) && (!rtIsNaN(x[1])))) {
    ex = x[1];
  } else {
    ex = x[0];
  }
  return ex;
}

/*
 * Arguments    : const emxArray_real32_T *x
 *                int *idx
 * Return Type  : float
 */
float b_minimum(const emxArray_real32_T *x, int *idx)
{
  const float *x_data;
  float ex;
  float f;
  int i;
  int k;
  int last;
  boolean_T exitg1;
  x_data = x->data;
  last = x->size[0];
  if (x->size[0] <= 2) {
    if (x->size[0] == 1) {
      ex = x_data[0];
      *idx = 1;
    } else {
      ex = x_data[x->size[0] - 1];
      if ((x_data[0] > ex) || (rtIsNaNF(x_data[0]) && (!rtIsNaNF(ex)))) {
        *idx = x->size[0];
      } else {
        ex = x_data[0];
        *idx = 1;
      }
    }
  } else {
    if (!rtIsNaNF(x_data[0])) {
      *idx = 1;
    } else {
      *idx = 0;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= last)) {
        if (!rtIsNaNF(x_data[k - 1])) {
          *idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }
    if (*idx == 0) {
      ex = x_data[0];
      *idx = 1;
    } else {
      ex = x_data[*idx - 1];
      i = *idx + 1;
      for (k = i; k <= last; k++) {
        f = x_data[k - 1];
        if (ex > f) {
          ex = f;
          *idx = k;
        }
      }
    }
  }
  return ex;
}

/*
 * Arguments    : const float x[16]
 * Return Type  : float
 */
float c_maximum(const float x[16])
{
  float ex;
  float f;
  int idx;
  int k;
  boolean_T exitg1;
  if (!rtIsNaNF(x[0])) {
    idx = 1;
  } else {
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k <= 16)) {
      if (!rtIsNaNF(x[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }
  if (idx == 0) {
    ex = x[0];
  } else {
    ex = x[idx - 1];
    idx++;
    for (k = idx; k < 17; k++) {
      f = x[k - 1];
      if (ex < f) {
        ex = f;
      }
    }
  }
  return ex;
}

/*
 * Arguments    : const float x[1024]
 *                int *idx
 * Return Type  : float
 */
float c_minimum(const float x[1024], int *idx)
{
  float ex;
  float f;
  int i;
  int k;
  boolean_T exitg1;
  if (!rtIsNaNF(x[0])) {
    *idx = 1;
  } else {
    *idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 1025)) {
      if (!rtIsNaNF(x[k - 1])) {
        *idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }
  if (*idx == 0) {
    ex = x[0];
    *idx = 1;
  } else {
    ex = x[*idx - 1];
    i = *idx + 1;
    for (k = i; k < 1025; k++) {
      f = x[k - 1];
      if (ex > f) {
        ex = f;
        *idx = k;
      }
    }
  }
  return ex;
}

/*
 * Arguments    : const double x[2]
 * Return Type  : double
 */
double d_minimum(const double x[2])
{
  double ex;
  if ((x[0] > x[1]) || (rtIsNaN(x[0]) && (!rtIsNaN(x[1])))) {
    ex = x[1];
  } else {
    ex = x[0];
  }
  return ex;
}

/*
 * Arguments    : const float x[32]
 * Return Type  : float
 */
float maximum(const float x[32])
{
  float ex;
  float f;
  int idx;
  int k;
  boolean_T exitg1;
  if (!rtIsNaNF(x[0])) {
    idx = 1;
  } else {
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k <= 32)) {
      if (!rtIsNaNF(x[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }
  if (idx == 0) {
    ex = x[0];
  } else {
    ex = x[idx - 1];
    idx++;
    for (k = idx; k < 33; k++) {
      f = x[k - 1];
      if (ex < f) {
        ex = f;
      }
    }
  }
  return ex;
}

/*
 * Arguments    : const float x[2]
 * Return Type  : float
 */
float minimum(const float x[2])
{
  float ex;
  if ((x[0] > x[1]) || (rtIsNaNF(x[0]) && (!rtIsNaNF(x[1])))) {
    ex = x[1];
  } else {
    ex = x[0];
  }
  return ex;
}

/*
 * File trailer for minOrMax.c
 *
 * [EOF]
 */
