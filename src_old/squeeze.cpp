/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: squeeze.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 31-Mar-2024 07:04:52
 */

/* Include Files */
#include "squeeze.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_emxutil.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : const emxArray_real_T *a
 *                emxArray_real_T *b
 * Return Type  : void
 */
void b_squeeze(const emxArray_real_T *a, emxArray_real_T *b)
{
  const double *a_data;
  double *b_data;
  int i;
  int j;
  unsigned char szb[2];
  boolean_T p;
  a_data = a->data;
  szb[0] = 1U;
  szb[1] = 1U;
  p = (a->size[2] == 1);
  if ((!p) || (a->size[3] != 1)) {
    p = false;
  }
  if (!p) {
    j = 0;
    if (a->size[2] != 1) {
      j = 1;
      szb[0] = (unsigned char)a->size[2];
    }
    if (a->size[3] != 1) {
      szb[j] = (unsigned char)a->size[3];
    }
  }
  i = b->size[0] * b->size[1];
  b->size[0] = szb[0];
  b->size[1] = szb[1];
  emxEnsureCapacity_real_T(b, i);
  b_data = b->data;
  j = szb[0] * szb[1];
  for (i = 0; i < j; i++) {
    b_data[i] = a_data[i];
  }
}

/*
 * Arguments    : const emxArray_uint8_T *a
 *                emxArray_uint8_T *b
 * Return Type  : void
 */
void c_squeeze(const emxArray_uint8_T *a, emxArray_uint8_T *b)
{
  int szb[2];
  int i;
  int j;
  const unsigned char *a_data;
  unsigned char *b_data;
  a_data = a->data;
  szb[0] = 1;
  szb[1] = a->size[1];
  if (a->size[2] != 1) {
    j = 0;
    if (a->size[1] != 1) {
      j = 1;
      szb[0] = a->size[1];
    }
    if (a->size[2] != 1) {
      szb[j] = a->size[2];
    }
  }
  i = b->size[0] * b->size[1];
  b->size[0] = szb[0];
  b->size[1] = szb[1];
  emxEnsureCapacity_uint8_T(b, i);
  b_data = b->data;
  j = szb[0] * szb[1];
  for (i = 0; i < j; i++) {
    b_data[i] = a_data[i];
  }
}

/*
 * Arguments    : const emxArray_real_T *a
 *                emxArray_real_T *b
 * Return Type  : void
 */
void squeeze(const emxArray_real_T *a, emxArray_real_T *b)
{
  const double *a_data;
  double *b_data;
  int szb[2];
  int i;
  int j;
  a_data = a->data;
  szb[0] = 1;
  szb[1] = a->size[1];
  if (a->size[2] != 1) {
    j = 0;
    if (a->size[1] != 1) {
      j = 1;
      szb[0] = a->size[1];
    }
    if (a->size[2] != 1) {
      szb[j] = a->size[2];
    }
  }
  i = b->size[0] * b->size[1];
  b->size[0] = szb[0];
  b->size[1] = szb[1];
  emxEnsureCapacity_real_T(b, i);
  b_data = b->data;
  j = szb[0] * szb[1];
  for (i = 0; i < j; i++) {
    b_data[i] = a_data[i];
  }
}

/*
 * File trailer for squeeze.c
 *
 * [EOF]
 */
