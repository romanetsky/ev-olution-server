/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: qrsolve.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 17-Mar-2024 11:09:30
 */

/* Include Files */
#include "qrsolve.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_emxutil.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h"
#include "rt_nonfinite.h"
#include "xzgeqp3.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const emxArray_real_T *A
 *                const emxArray_real_T *B
 *                double Y[2]
 * Return Type  : int
 */
int qrsolve(const emxArray_real_T *A, const emxArray_real_T *B, double Y[2])
{
  emxArray_real_T *b_A;
  emxArray_real_T *b_B;
  double tau_data[2];
  const double *A_data;
  const double *B_data;
  double tol;
  double *b_A_data;
  double *b_B_data;
  int jpvt[2];
  int assumedRank;
  int b_i;
  int i;
  int j;
  int maxmn;
  int minmana;
  int rankA;
  B_data = B->data;
  A_data = A->data;
  emxInit_real_T(&b_A, 2);
  i = b_A->size[0] * b_A->size[1];
  b_A->size[0] = A->size[0];
  b_A->size[1] = 2;
  emxEnsureCapacity_real_T(b_A, i);
  b_A_data = b_A->data;
  minmana = A->size[0] << 1;
  for (i = 0; i < minmana; i++) {
    b_A_data[i] = A_data[i];
  }
  minmana = A->size[0];
  if (minmana > 2) {
    minmana = 2;
  }
  if (minmana - 1 >= 0) {
    memset(&tau_data[0], 0, (unsigned int)minmana * sizeof(double));
  }
  if (A->size[0] == 0) {
    jpvt[0] = 1;
    jpvt[1] = 2;
  } else {
    jpvt[0] = 1;
    jpvt[1] = 2;
    qrpf(b_A, A->size[0], tau_data, jpvt);
    b_A_data = b_A->data;
  }
  rankA = 0;
  if (b_A->size[0] < 2) {
    minmana = b_A->size[0];
    maxmn = 2;
  } else {
    minmana = 2;
    maxmn = b_A->size[0];
  }
  if (minmana > 0) {
    tol = 2.2204460492503131E-15 * (double)maxmn * fabs(b_A_data[0]);
    while ((rankA < minmana) &&
           (!(fabs(b_A_data[rankA + b_A->size[0] * rankA]) <= tol))) {
      rankA++;
    }
  }
  assumedRank = 0;
  maxmn = b_A->size[0];
  if (maxmn > 2) {
    maxmn = 2;
  }
  if (maxmn > 0) {
    for (minmana = 0; minmana < maxmn; minmana++) {
      if (b_A_data[minmana + b_A->size[0] * minmana] != 0.0) {
        assumedRank++;
      }
    }
  }
  emxInit_real_T(&b_B, 1);
  i = b_B->size[0];
  b_B->size[0] = B->size[0];
  emxEnsureCapacity_real_T(b_B, i);
  b_B_data = b_B->data;
  minmana = B->size[0];
  for (i = 0; i < minmana; i++) {
    b_B_data[i] = B_data[i];
  }
  Y[0] = 0.0;
  Y[1] = 0.0;
  minmana = b_A->size[0];
  for (j = 0; j < maxmn; j++) {
    if (tau_data[j] != 0.0) {
      tol = b_B_data[j];
      i = j + 2;
      for (b_i = i; b_i <= minmana; b_i++) {
        tol += b_A_data[(b_i + b_A->size[0] * j) - 1] * b_B_data[b_i - 1];
      }
      tol *= tau_data[j];
      if (tol != 0.0) {
        b_B_data[j] -= tol;
        for (b_i = i; b_i <= minmana; b_i++) {
          b_B_data[b_i - 1] -= b_A_data[(b_i + b_A->size[0] * j) - 1] * tol;
        }
      }
    }
  }
  for (b_i = 0; b_i < assumedRank; b_i++) {
    Y[jpvt[b_i] - 1] = b_B_data[b_i];
  }
  emxFree_real_T(&b_B);
  for (j = assumedRank; j >= 1; j--) {
    minmana = jpvt[j - 1] - 1;
    Y[minmana] /= b_A_data[(j + b_A->size[0] * (j - 1)) - 1];
    for (b_i = 0; b_i <= j - 2; b_i++) {
      Y[jpvt[0] - 1] -= Y[minmana] * b_A_data[b_A->size[0] * (j - 1)];
    }
  }
  emxFree_real_T(&b_A);
  return rankA;
}

/*
 * File trailer for qrsolve.c
 *
 * [EOF]
 */
