/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: padArrUint8.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 17-Mar-2024 11:09:30
 */

/* Include Files */
#include "padArrUint8.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_emxutil.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h"
#include "find.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const emxArray_uint8_T *Ain
 *                unsigned char B[256]
 * Return Type  : void
 */
void b_padArrUint8(const emxArray_uint8_T *Ain, unsigned char B[256])
{
  emxArray_boolean_T c_A_data;
  emxArray_uint8_T *A;
  int b_A_size[2];
  int lastCol_size[2];
  int A_size;
  int i;
  int i1;
  int lastCol_data;
  int lastRow_data;
  const unsigned char *Ain_data;
  unsigned char *A_data;
  boolean_T b_A_data[32767];
  boolean_T d_A_data[32767];
  Ain_data = Ain->data;
  emxInit_uint8_T(&A, 2);
  if ((Ain->size[0] == 0) || (Ain->size[1] == 0)) {
    i = A->size[0] * A->size[1];
    A->size[0] = 16;
    A->size[1] = 16;
    emxEnsureCapacity_uint8_T(A, i);
    A_data = A->data;
    for (i = 0; i < 256; i++) {
      A_data[i] = 0U;
    }
  } else {
    i = A->size[0] * A->size[1];
    A->size[0] = Ain->size[0];
    A->size[1] = Ain->size[1];
    emxEnsureCapacity_uint8_T(A, i);
    A_data = A->data;
    lastCol_data = Ain->size[0] * Ain->size[1];
    for (i = 0; i < lastCol_data; i++) {
      A_data[i] = Ain_data[i];
    }
  }
  memset(&B[0], 0, 256U * sizeof(unsigned char));
  A_size = A->size[0];
  lastCol_data = A->size[0];
  for (i = 0; i < lastCol_data; i++) {
    b_A_data[i] = (A_data[i] != 0);
  }
  c_A_data.data = &b_A_data[0];
  c_A_data.size = &A_size;
  c_A_data.allocatedSize = 32767;
  c_A_data.numDimensions = 1;
  c_A_data.canFreeData = false;
  d_eml_find(&c_A_data, (int *)&lastRow_data);
  b_A_size[0] = 1;
  b_A_size[1] = A->size[1];
  lastCol_data = A->size[1];
  for (i = 0; i < lastCol_data; i++) {
    d_A_data[i] = (A_data[A->size[0] * i] != 0);
  }
  e_eml_find(d_A_data, b_A_size, (int *)&lastCol_data, lastCol_size);
  for (i = 0; i < lastCol_data; i++) {
    for (i1 = 0; i1 < lastRow_data; i1++) {
      A_data[i1 + lastRow_data * i] = A_data[i1 + A->size[0] * i];
    }
  }
  i = A->size[0] * A->size[1];
  A->size[0] = lastRow_data;
  A->size[1] = lastCol_data;
  emxEnsureCapacity_uint8_T(A, i);
  A_data = A->data;
  for (i = 0; i < lastCol_data; i++) {
    for (i1 = 0; i1 < lastRow_data; i1++) {
      B[i1 + (i << 4)] = A_data[i1 + lastRow_data * i];
    }
  }
  emxFree_uint8_T(&A);
}

/*
 * Arguments    : const unsigned char Ain_data[]
 *                int Ain_size
 *                short N1
 *                short N2
 *                emxArray_uint8_T *B
 * Return Type  : void
 */
void c_padArrUint8(const unsigned char Ain_data[], int Ain_size, short N1,
                   short N2, emxArray_uint8_T *B)
{
  emxArray_boolean_T c_A_data;
  emxArray_uint8_T *A;
  int b_A_size[2];
  int lastCol_size[2];
  int A_size;
  int i;
  int i1;
  int lastCol_data;
  int lastRow_data;
  unsigned char *A_data;
  unsigned char *B_data;
  boolean_T b_A_data[32767];
  boolean_T d_A_data[32767];
  emxInit_uint8_T(&A, 2);
  if (Ain_size == 0) {
    i = A->size[0] * A->size[1];
    A->size[0] = N1;
    A->size[1] = N2;
    emxEnsureCapacity_uint8_T(A, i);
    A_data = A->data;
    lastCol_data = N1 * N2;
    for (i = 0; i < lastCol_data; i++) {
      A_data[i] = 0U;
    }
  } else {
    i = A->size[0] * A->size[1];
    A->size[0] = Ain_size;
    A->size[1] = 1;
    emxEnsureCapacity_uint8_T(A, i);
    A_data = A->data;
    for (i = 0; i < Ain_size; i++) {
      A_data[i] = Ain_data[i];
    }
  }
  i = B->size[0] * B->size[1];
  B->size[0] = N1;
  B->size[1] = N2;
  emxEnsureCapacity_uint8_T(B, i);
  B_data = B->data;
  lastCol_data = N1 * N2;
  for (i = 0; i < lastCol_data; i++) {
    B_data[i] = 0U;
  }
  A_size = A->size[0];
  lastCol_data = A->size[0];
  for (i = 0; i < lastCol_data; i++) {
    b_A_data[i] = (A_data[i] != 0);
  }
  c_A_data.data = &b_A_data[0];
  c_A_data.size = &A_size;
  c_A_data.allocatedSize = 32767;
  c_A_data.numDimensions = 1;
  c_A_data.canFreeData = false;
  d_eml_find(&c_A_data, (int *)&lastRow_data);
  b_A_size[0] = 1;
  b_A_size[1] = A->size[1];
  lastCol_data = A->size[1];
  for (i = 0; i < lastCol_data; i++) {
    d_A_data[i] = (A_data[A->size[0] * i] != 0);
  }
  e_eml_find(d_A_data, b_A_size, (int *)&lastCol_data, lastCol_size);
  for (i = 0; i < lastCol_data; i++) {
    for (i1 = 0; i1 < lastRow_data; i1++) {
      A_data[i1 + lastRow_data * i] = A_data[i1 + A->size[0] * i];
    }
  }
  i = A->size[0] * A->size[1];
  A->size[0] = lastRow_data;
  A->size[1] = lastCol_data;
  emxEnsureCapacity_uint8_T(A, i);
  A_data = A->data;
  for (i = 0; i < lastCol_data; i++) {
    for (i1 = 0; i1 < lastRow_data; i1++) {
      B_data[i1 + B->size[0] * i] = A_data[i1 + lastRow_data * i];
    }
  }
  emxFree_uint8_T(&A);
}

/*
 * Arguments    : const unsigned char Ain[256]
 *                double N1
 *                double N2
 *                emxArray_uint8_T *B
 * Return Type  : void
 */
void d_padArrUint8(const unsigned char Ain[256], double N1, double N2,
                   emxArray_uint8_T *B)
{
  int i;
  int i1;
  int idx;
  int ii;
  unsigned char Ain_data[256];
  signed char b_ii_data;
  signed char ii_data;
  unsigned char *B_data;
  boolean_T x[16];
  boolean_T exitg1;
  i = B->size[0] * B->size[1];
  B->size[0] = (int)N1;
  B->size[1] = (int)N2;
  emxEnsureCapacity_uint8_T(B, i);
  B_data = B->data;
  idx = (int)N1 * (int)N2;
  for (i = 0; i < idx; i++) {
    B_data[i] = 0U;
  }
  for (i = 0; i < 16; i++) {
    x[i] = (Ain[i] != 0);
  }
  idx = 0;
  ii = 16;
  exitg1 = false;
  while ((!exitg1) && (ii > 0)) {
    if (x[ii - 1]) {
      idx = 1;
      ii_data = (signed char)ii;
      exitg1 = true;
    } else {
      ii--;
    }
  }
  if (idx == 0) {
  }
  for (i = 0; i < 16; i++) {
    x[i] = (Ain[i << 4] != 0);
  }
  idx = 0;
  ii = 16;
  exitg1 = false;
  while ((!exitg1) && (ii > 0)) {
    if (x[ii - 1]) {
      idx = 1;
      b_ii_data = (signed char)ii;
      exitg1 = true;
    } else {
      ii--;
    }
  }
  if (idx == 0) {
  }
  idx = ii_data;
  ii = b_ii_data;
  for (i = 0; i < ii; i++) {
    for (i1 = 0; i1 < idx; i1++) {
      Ain_data[i1 + ii_data * i] = Ain[i1 + (i << 4)];
    }
  }
  for (i = 0; i < ii; i++) {
    for (i1 = 0; i1 < idx; i1++) {
      B_data[i1 + B->size[0] * i] = Ain_data[i1 + ii_data * i];
    }
  }
}

/*
 * Arguments    : unsigned char Ain
 *                double N1
 *                double N2
 *                emxArray_uint8_T *B
 * Return Type  : void
 */
void e_padArrUint8(unsigned char Ain, double N1, double N2, emxArray_uint8_T *B)
{
  int i;
  int loop_ub;
  unsigned char *B_data;
  i = B->size[0] * B->size[1];
  B->size[0] = (int)N1;
  B->size[1] = (int)N2;
  emxEnsureCapacity_uint8_T(B, i);
  B_data = B->data;
  loop_ub = (int)N1 * (int)N2;
  for (i = 0; i < loop_ub; i++) {
    B_data[i] = 0U;
  }
  B_data[0] = Ain;
}

/*
 * Arguments    : unsigned char Ain
 *                short N1
 *                short N2
 *                emxArray_uint8_T *B
 * Return Type  : void
 */
void padArrUint8(unsigned char Ain, short N1, short N2, emxArray_uint8_T *B)
{
  int i;
  int loop_ub;
  unsigned char *B_data;
  i = B->size[0] * B->size[1];
  B->size[0] = N1;
  B->size[1] = N2;
  emxEnsureCapacity_uint8_T(B, i);
  B_data = B->data;
  loop_ub = N1 * N2;
  for (i = 0; i < loop_ub; i++) {
    B_data[i] = 0U;
  }
  B_data[0] = Ain;
}

/*
 * File trailer for padArrUint8.c
 *
 * [EOF]
 */
