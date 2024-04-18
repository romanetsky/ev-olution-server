/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: padArr.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 17-Mar-2024 11:09:30
 */

/* Include Files */
#include "padArr.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_emxutil.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h"
#include "find.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : unsigned char A
 *                double N1
 *                double N2
 *                emxArray_real_T *B
 * Return Type  : void
 */
void b_padArr(unsigned char A, double N1, double N2, emxArray_real_T *B)
{
  double *B_data;
  int i;
  int loop_ub;
  i = B->size[0] * B->size[1];
  B->size[0] = (int)N1;
  B->size[1] = (int)N2;
  emxEnsureCapacity_real_T(B, i);
  B_data = B->data;
  loop_ub = (int)N1 * (int)N2;
  for (i = 0; i < loop_ub; i++) {
    B_data[i] = 0.0;
  }
  B_data[0] = A;
}

/*
 * Arguments    : const emxArray_real_T *A
 *                double N1
 *                double N2
 *                emxArray_real_T *B
 * Return Type  : void
 */
void padArr(const emxArray_real_T *A, double N1, double N2, emxArray_real_T *B)
{
  emxArray_boolean_T *b_A;
  const double *A_data;
  double *B_data;
  int i;
  int loop_ub;
  int tmp_data;
  boolean_T *b_A_data;
  A_data = A->data;
  i = B->size[0] * B->size[1];
  B->size[0] = (int)N1;
  B->size[1] = (int)N2;
  emxEnsureCapacity_real_T(B, i);
  B_data = B->data;
  loop_ub = (int)N1 * (int)N2;
  for (i = 0; i < loop_ub; i++) {
    B_data[i] = 0.0;
  }
  emxInit_boolean_T(&b_A, 1);
  i = b_A->size[0];
  b_A->size[0] = A->size[0];
  emxEnsureCapacity_boolean_T(b_A, i);
  b_A_data = b_A->data;
  loop_ub = A->size[0];
  for (i = 0; i < loop_ub; i++) {
    b_A_data[i] = (A_data[i] != 0.0);
  }
  d_eml_find(b_A, (int *)&tmp_data);
  emxFree_boolean_T(&b_A);
  for (i = 0; i < tmp_data; i++) {
    B_data[i] = A_data[i];
  }
}

/*
 * File trailer for padArr.c
 *
 * [EOF]
 */
