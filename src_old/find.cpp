/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: find.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 31-Mar-2024 07:04:52
 */

/* Include Files */
#include "find.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : const boolean_T x[6]
 *                int i_data[]
 * Return Type  : int
 */
int b_eml_find(const boolean_T x[6], int i_data[])
{
  int i_size;
  int ii;
  boolean_T exitg1;
  i_size = 0;
  ii = 0;
  exitg1 = false;
  while ((!exitg1) && (ii < 6)) {
    if (x[ii]) {
      i_size++;
      i_data[i_size - 1] = ii + 1;
      if (i_size >= 6) {
        exitg1 = true;
      } else {
        ii++;
      }
    } else {
      ii++;
    }
  }
  if (i_size < 1) {
    i_size = 0;
  }
  return i_size;
}

/*
 * Arguments    : const boolean_T x_data[]
 *                int x_size
 *                int i_data[]
 * Return Type  : int
 */
int c_eml_find(const boolean_T x_data[], int x_size, int i_data[])
{
  int i_size;
  int idx;
  int ii;
  boolean_T exitg1;
  idx = 0;
  i_size = x_size;
  ii = 0;
  exitg1 = false;
  while ((!exitg1) && (ii <= x_size - 1)) {
    if (x_data[ii]) {
      idx++;
      i_data[idx - 1] = ii + 1;
      if (idx >= x_size) {
        exitg1 = true;
      } else {
        ii++;
      }
    } else {
      ii++;
    }
  }
  if (x_size == 1) {
    if (idx == 0) {
      i_size = 0;
    }
  } else if (idx < 1) {
    i_size = 0;
  } else {
    i_size = idx;
  }
  return i_size;
}

/*
 * Arguments    : const emxArray_boolean_T *x
 *                int i_data[]
 * Return Type  : int
 */
int d_eml_find(const emxArray_boolean_T *x, int i_data[])
{
  int i_size;
  int idx;
  int ii;
  const boolean_T *x_data;
  boolean_T exitg1;
  x_data = x->data;
  i_size = (x->size[0] >= 1);
  ii = x->size[0];
  idx = 0;
  exitg1 = false;
  while ((!exitg1) && (ii > 0)) {
    if (x_data[ii - 1]) {
      idx = 1;
      i_data[0] = ii;
      exitg1 = true;
    } else {
      ii--;
    }
  }
  if (i_size == 1) {
    if (idx == 0) {
      i_size = 0;
    }
  } else {
    i_size = (idx >= 1);
  }
  return i_size;
}

/*
 * Arguments    : const boolean_T x_data[]
 *                const int x_size[2]
 *                int i_data[]
 *                int i_size[2]
 * Return Type  : void
 */
void e_eml_find(const boolean_T x_data[], const int x_size[2], int i_data[],
                int i_size[2])
{
  int idx;
  int ii;
  int k;
  boolean_T exitg1;
  k = (x_size[1] >= 1);
  ii = x_size[1];
  idx = 0;
  i_size[0] = 1;
  i_size[1] = k;
  exitg1 = false;
  while ((!exitg1) && (ii > 0)) {
    if (x_data[ii - 1]) {
      idx = 1;
      i_data[0] = ii;
      exitg1 = true;
    } else {
      ii--;
    }
  }
  if (k == 1) {
    if (idx == 0) {
      i_size[0] = 1;
      i_size[1] = 0;
    }
  } else {
    i_size[1] = (idx >= 1);
  }
}

/*
 * Arguments    : const boolean_T x[257]
 *                int i_data[]
 * Return Type  : int
 */
int eml_find(const boolean_T x[257], int i_data[])
{
  int i_size;
  int ii;
  boolean_T exitg1;
  i_size = 0;
  ii = 0;
  exitg1 = false;
  while ((!exitg1) && (ii < 257)) {
    if (x[ii]) {
      i_size++;
      i_data[i_size - 1] = ii + 1;
      if (i_size >= 257) {
        exitg1 = true;
      } else {
        ii++;
      }
    } else {
      ii++;
    }
  }
  if (i_size < 1) {
    i_size = 0;
  }
  return i_size;
}

/*
 * Arguments    : const emxArray_boolean_T *x
 *                int i_data[]
 *                int i_size[2]
 * Return Type  : void
 */
void f_eml_find(const emxArray_boolean_T *x, int i_data[], int i_size[2])
{
  int idx;
  int ii;
  int k;
  const boolean_T *x_data;
  boolean_T exitg1;
  x_data = x->data;
  k = (x->size[1] >= 1);
  idx = 0;
  i_size[0] = 1;
  i_size[1] = k;
  ii = 0;
  exitg1 = false;
  while ((!exitg1) && (ii <= x->size[1] - 1)) {
    if (x_data[ii]) {
      idx = 1;
      i_data[0] = ii + 1;
      exitg1 = true;
    } else {
      ii++;
    }
  }
  if (k == 1) {
    if (idx == 0) {
      i_size[0] = 1;
      i_size[1] = 0;
    }
  } else {
    i_size[1] = (idx >= 1);
  }
}

/*
 * Arguments    : const boolean_T x_data[]
 *                int x_size
 *                int i_data[]
 * Return Type  : int
 */
int g_eml_find(const boolean_T x_data[], int x_size, int i_data[])
{
  int i_size;
  int idx;
  int ii;
  unsigned short b_i_data[2];
  boolean_T exitg1;
  i_size = (x_size >= 1);
  idx = 0;
  ii = 0;
  exitg1 = false;
  while ((!exitg1) && (ii <= x_size - 1)) {
    if (x_data[ii]) {
      idx++;
      b_i_data[idx - 1] = (unsigned short)(ii + 1);
      if (idx >= i_size) {
        exitg1 = true;
      } else {
        ii++;
      }
    } else {
      ii++;
    }
  }
  if (i_size == 1) {
    if (idx == 0) {
      i_size = 0;
    }
  } else {
    i_size = (idx >= 1);
  }
  if (i_size - 1 >= 0) {
    i_data[0] = b_i_data[0];
  }
  return i_size;
}

/*
 * Arguments    : const boolean_T x_data[]
 *                int x_size
 *                int i_data[]
 * Return Type  : int
 */
int h_eml_find(const boolean_T x_data[], int x_size, int i_data[])
{
  int i_size;
  int idx;
  int ii;
  boolean_T exitg1;
  i_size = (x_size >= 1);
  ii = x_size;
  idx = 0;
  exitg1 = false;
  while ((!exitg1) && (ii > 0)) {
    if (x_data[ii - 1]) {
      idx = 1;
      i_data[0] = ii;
      exitg1 = true;
    } else {
      ii--;
    }
  }
  if (i_size == 1) {
    if (idx == 0) {
      i_size = 0;
    }
  } else {
    i_size = (idx >= 1);
  }
  return i_size;
}

/*
 * File trailer for find.c
 *
 * [EOF]
 */
