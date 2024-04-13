/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: remPadArr.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 08-Apr-2024 21:02:32
 */

/* Include Files */
#include "remPadArr.h"
#include "find.h"
#include "rt_nonfinite.h"
#include "squeeze.h"

/* Function Definitions */
/*
 * Arguments    : const double A_data[]
 *                const int A_size[3]
 *                double B_data[]
 *                int B_size[2]
 * Return Type  : void
 */
void remPadArr(const double A_data[], const int A_size[3], double B_data[],
               int B_size[2])
{
  double b_A_data[256];
  int b_A_size[2];
  int c_A_size[2];
  int tmp_size[2];
  int b_tmp_data;
  int c_tmp_data;
  int i;
  int loop_ub;
  int tmp_data;
  boolean_T c_A_data[16];
  boolean_T d_A_data[16];
  squeeze(A_data, A_size, b_A_data, b_A_size);
  if ((b_A_size[0] == 0) || (b_A_size[1] == 0)) {
    B_size[0] = 0;
    B_size[1] = 0;
  } else {
    /* forCoder */
    loop_ub = b_A_size[0];
    for (i = 0; i < loop_ub; i++) {
      c_A_data[i] = (b_A_data[i] != 0.0);
    }
    c_A_size[0] = 1;
    c_A_size[1] = b_A_size[1];
    loop_ub = b_A_size[1];
    for (i = 0; i < loop_ub; i++) {
      d_A_data[i] = (b_A_data[b_A_size[0] * i] != 0.0);
    }
    d_eml_find(c_A_data, b_A_size[0], (int *)&tmp_data);
    e_eml_find(d_A_data, c_A_size, (int *)&b_tmp_data, tmp_size);
    d_eml_find(c_A_data, b_A_size[0], (int *)&c_tmp_data);
    B_size[0] = c_tmp_data;
    B_size[1] = b_tmp_data;
    for (i = 0; i < b_tmp_data; i++) {
      for (loop_ub = 0; loop_ub < tmp_data; loop_ub++) {
        B_data[loop_ub + c_tmp_data * i] = b_A_data[loop_ub + b_A_size[0] * i];
      }
    }
  }
}

/*
 * File trailer for remPadArr.c
 *
 * [EOF]
 */
