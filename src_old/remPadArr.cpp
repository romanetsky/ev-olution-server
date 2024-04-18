/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: remPadArr.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 31-Mar-2024 07:04:52
 */

/* Include Files */
#include "remPadArr.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_emxutil.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h"
#include "find.h"
#include "rt_nonfinite.h"
#include "squeeze.h"

/* Function Definitions */
/*
 * Arguments    : const emxArray_real_T *A
 *                emxArray_real_T *B
 * Return Type  : void
 */
void remPadArr(const emxArray_real_T *A, emxArray_real_T *B)
{
  emxArray_boolean_T d_A_data;
  emxArray_boolean_T e_A_data;
  emxArray_real_T *b_A;
  double *A_data;
  double *B_data;
  int b_A_size[2];
  int tmp_size[2];
  int A_size;
  int b_tmp_data;
  int c_tmp_data;
  int i;
  int loop_ub;
  int tmp_data;
  boolean_T b_A_data[32767];
  boolean_T c_A_data[32767];
  emxInit_real_T(&b_A, 2);
  squeeze(A, b_A);
  A_data = b_A->data;
  if ((b_A->size[0] == 0) || (b_A->size[1] == 0)) {
    B->size[0] = 0;
    B->size[1] = 0;
  } else {
    /* forCoder */
    A_size = b_A->size[0];
    loop_ub = b_A->size[0];
    for (i = 0; i < loop_ub; i++) {
      b_A_data[i] = (A_data[i] != 0.0);
    }
    b_A_size[0] = 1;
    b_A_size[1] = b_A->size[1];
    loop_ub = b_A->size[1];
    for (i = 0; i < loop_ub; i++) {
      c_A_data[i] = (A_data[b_A->size[0] * i] != 0.0);
    }
    d_A_data.data = &b_A_data[0];
    d_A_data.size = &A_size;
    d_A_data.allocatedSize = 32767;
    d_A_data.numDimensions = 1;
    d_A_data.canFreeData = false;
    d_eml_find(&d_A_data, (int *)&tmp_data);
    e_eml_find(c_A_data, b_A_size, (int *)&b_tmp_data, tmp_size);
    e_A_data.data = &b_A_data[0];
    e_A_data.size = &A_size;
    e_A_data.allocatedSize = 32767;
    e_A_data.numDimensions = 1;
    e_A_data.canFreeData = false;
    d_eml_find(&e_A_data, (int *)&c_tmp_data);
    i = B->size[0] * B->size[1];
    B->size[0] = c_tmp_data;
    B->size[1] = b_tmp_data;
    emxEnsureCapacity_real_T(B, i);
    B_data = B->data;
    for (i = 0; i < b_tmp_data; i++) {
      for (loop_ub = 0; loop_ub < tmp_data; loop_ub++) {
        B_data[loop_ub + B->size[0] * i] = A_data[loop_ub + b_A->size[0] * i];
      }
    }
  }
  emxFree_real_T(&b_A);
}

/*
 * File trailer for remPadArr.c
 *
 * [EOF]
 */
