/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: squeeze.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 08-Apr-2024 21:02:32
 */

/* Include Files */
#include "squeeze.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_emxutil.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const double a_data[]
 *                const int a_size[4]
 *                emxArray_real_T *b
 * Return Type  : void
 */
void b_squeeze(const double a_data[], const int a_size[4], emxArray_real_T *b)
{
  double *b_data;
  int i;
  int j;
  unsigned char szb[2];
  boolean_T p;
  szb[0] = 1U;
  szb[1] = 1U;
  p = (a_size[2] == 1);
  if ((!p) || (a_size[3] != 1)) {
    p = false;
  }
  if (!p) {
    j = 0;
    if (a_size[2] != 1) {
      j = 1;
      szb[0] = (unsigned char)a_size[2];
    }
    if (a_size[3] != 1) {
      szb[j] = (unsigned char)a_size[3];
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
 * Arguments    : const unsigned char a_data[]
 *                const int a_size[3]
 *                unsigned char b_data[]
 *                int b_size[2]
 * Return Type  : void
 */
void c_squeeze(const unsigned char a_data[], const int a_size[3],
               unsigned char b_data[], int b_size[2])
{
  int j;
  signed char szb[2];
  szb[0] = 1;
  szb[1] = (signed char)a_size[1];
  if (a_size[2] != 1) {
    j = 0;
    if (a_size[1] != 1) {
      j = 1;
      szb[0] = (signed char)a_size[1];
    }
    if (a_size[2] != 1) {
      szb[j] = (signed char)a_size[2];
    }
  }
  b_size[0] = szb[0];
  b_size[1] = szb[1];
  j = szb[0] * szb[1];
  if (j - 1 >= 0) {
    memcpy(&b_data[0], &a_data[0], (unsigned int)j * sizeof(unsigned char));
  }
}

/*
 * Arguments    : const double a_data[]
 *                const int a_size[3]
 *                double b_data[]
 *                int b_size[2]
 * Return Type  : void
 */
void squeeze(const double a_data[], const int a_size[3], double b_data[],
             int b_size[2])
{
  int j;
  signed char szb[2];
  szb[0] = 1;
  szb[1] = (signed char)a_size[1];
  if (a_size[2] != 1) {
    j = 0;
    if (a_size[1] != 1) {
      j = 1;
      szb[0] = (signed char)a_size[1];
    }
    if (a_size[2] != 1) {
      szb[j] = (signed char)a_size[2];
    }
  }
  b_size[0] = szb[0];
  b_size[1] = szb[1];
  j = szb[0] * szb[1];
  if (j - 1 >= 0) {
    memcpy(&b_data[0], &a_data[0], (unsigned int)j * sizeof(double));
  }
}

/*
 * File trailer for squeeze.c
 *
 * [EOF]
 */
