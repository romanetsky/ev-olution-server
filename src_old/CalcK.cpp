/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: CalcK.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 31-Mar-2024 07:04:52
 */

/* Include Files */
#include "CalcK.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_emxutil.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h"
#include "mpower.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const emxArray_real_T *VdebugN
 *                const double VdebugVec_data[]
 *                const int VdebugVec_size[2]
 *                const signed char VIpacId[16]
 *                const unsigned char Pac2Vid0[16]
 *                const double VmKp184Test_data[]
 *                int VmKp184Test_size
 *                emxArray_real_T *k
 * Return Type  : void
 */
void CalcK(const emxArray_real_T *VdebugN, const double VdebugVec_data[],
           const int VdebugVec_size[2], const signed char VIpacId[16],
           const unsigned char Pac2Vid0[16], const double VmKp184Test_data[],
           int VmKp184Test_size, emxArray_real_T *k)
{
  emxArray_real_T *A;
  emxArray_real_T *b_A;
  emxArray_real_T *c_k;
  emxArray_real_T *s;
  emxArray_real_T *y;
  double VdebugVec1_data[512];
  double Vdebug1N1_data[32];
  double VdebugN1_data[32];
  double VmKp184_data[32];
  double Vdebug3[16];
  double Vdebug4[16];
  const double *VdebugN_data;
  double bkj;
  double d;
  double *A_data;
  double *k_data;
  double *y_data;
  int aoffset;
  int b_i;
  int b_k;
  int coffset;
  int i;
  int i1;
  int i2;
  int j;
  int mc;
  int xj;
  VdebugN_data = VdebugN->data;
  coffset = VdebugVec_size[1];
  for (i = 0; i < coffset; i++) {
    for (i1 = 0; i1 < 16; i1++) {
      VdebugVec1_data[i1 + 16 * i] =
          VdebugVec_data[(VIpacId[i1] + VdebugVec_size[0] * i) - 1];
    }
  }
  /*  twos_comp converts input variable */
  /*  "data" to it's twos complement */
  /*  value. Input variable "data" is */
  /*  expected to be hexadecimal. */
  /*  twos_comp converts input variable */
  /*  "data" to it's twos complement */
  /*  value. Input variable "data" is */
  /*  expected to be hexadecimal. */
  for (xj = 0; xj < 16; xj++) {
    Vdebug4[xj] = (VdebugVec1_data[xj] * 256.0 + VdebugVec1_data[xj + 16]) /
                  32768.0 * 9.0;
  }
  coffset = VdebugN->size[1];
  /*  twos_comp converts input variable */
  /*  "data" to it's twos complement */
  /*  value. Input variable "data" is */
  /*  expected to be hexadecimal. */
  /*  twos_comp converts input variable */
  /*  "data" to it's twos complement */
  /*  value. Input variable "data" is */
  /*  expected to be hexadecimal. */
  for (i = 0; i < coffset; i++) {
    for (i1 = 0; i1 < 16; i1++) {
      bkj = VdebugN_data[(VIpacId[i1] + VdebugN->size[0] * i) - 1];
      xj = i1 + 16 * i;
      VdebugN1_data[xj] = bkj;
      Vdebug1N1_data[xj] = bkj;
    }
  }
  /*  twos_comp converts input variable */
  /*  "data" to it's twos complement */
  /*  value. Input variable "data" is */
  /*  expected to be hexadecimal. */
  /*  twos_comp converts input variable */
  /*  "data" to it's twos complement */
  /*  value. Input variable "data" is */
  /*  expected to be hexadecimal. */
  if (VmKp184Test_size == 0) {
  } else {
    memcpy(&VmKp184_data[0], &VmKp184Test_data[0],
           (unsigned int)VmKp184Test_size * sizeof(double));
  }
  for (i = 0; i < 16; i++) {
    Vdebug3[i] = Vdebug4[Pac2Vid0[i] - 1];
  }
  memcpy(&Vdebug4[0], &Vdebug3[0], 16U * sizeof(double));
  Vdebug3[0] = 0.0;
  for (b_k = 0; b_k < 15; b_k++) {
    Vdebug3[b_k + 1] =
        Vdebug3[b_k] + VmKp184_data[(signed char)Pac2Vid0[b_k] - 1];
  }
  emxInit_real_T(&A, 2);
  i = VdebugN->size[0] << 1;
  i1 = A->size[0] * A->size[1];
  A->size[0] = i;
  A->size[1] = i;
  emxEnsureCapacity_real_T(A, i1);
  A_data = A->data;
  coffset = i * i;
  for (i = 0; i < coffset; i++) {
    A_data[i] = 0.0;
  }
  i = VdebugN->size[0];
  for (aoffset = 0; aoffset < i; aoffset++) {
    coffset = aoffset << 1;
    bkj = VmKp184_data[Pac2Vid0[aoffset] - 1];
    d = Vdebug3[aoffset];
    A_data[aoffset + A->size[0] * coffset] = bkj + d;
    A_data[aoffset + A->size[0] * (coffset + 1)] = -d;
    A_data[(aoffset + VdebugN->size[0]) + A->size[0] * coffset] = bkj;
  }
  bkj = VmKp184_data[Pac2Vid0[(int)((double)VdebugN->size[0] / 2.0) - 1] - 1];
  A_data[0] = VmKp184_data[Pac2Vid0[0] - 1] + bkj;
  A_data[A->size[0]] = -bkj;
  for (i = 0; i < 16; i++) {
    coffset = Pac2Vid0[i] - 1;
    VmKp184_data[i] =
        (VdebugN1_data[coffset] * 256.0 + VdebugN1_data[coffset + 16]) /
        32768.0 * 9.0;
    VmKp184_data[i + 16] = Vdebug4[i];
  }
  VmKp184_data[0] = (Vdebug1N1_data[Pac2Vid0[0] - 1] * 256.0 +
                     Vdebug1N1_data[Pac2Vid0[0] + 15]) /
                    32768.0 * 9.0;
  mc = A->size[1];
  coffset = A->size[0];
  xj = A->size[1];
  emxInit_real_T(&y, 2);
  i = y->size[0] * y->size[1];
  y->size[0] = A->size[1];
  y->size[1] = A->size[1];
  emxEnsureCapacity_real_T(y, i);
  y_data = y->data;
  for (j = 0; j < xj; j++) {
    aoffset = j * mc;
    for (b_i = 0; b_i < mc; b_i++) {
      y_data[aoffset + b_i] = 0.0;
    }
    for (b_k = 0; b_k < coffset; b_k++) {
      bkj = A_data[aoffset + b_k];
      for (b_i = 0; b_i < mc; b_i++) {
        i = aoffset + b_i;
        y_data[i] += A_data[b_i * A->size[0] + b_k] * bkj;
      }
    }
  }
  emxInit_real_T(&b_A, 2);
  mpower(y, b_A);
  k_data = b_A->data;
  mc = b_A->size[0];
  i = y->size[0] * y->size[1];
  y->size[0] = b_A->size[0];
  y->size[1] = A->size[0];
  emxEnsureCapacity_real_T(y, i);
  y_data = y->data;
  i = A->size[0];
  for (j = 0; j < i; j++) {
    coffset = j * mc;
    i1 = (unsigned short)mc;
    for (b_i = 0; b_i < i1; b_i++) {
      y_data[coffset + b_i] = 0.0;
    }
    xj = (unsigned short)b_A->size[1];
    for (b_k = 0; b_k < xj; b_k++) {
      aoffset = b_k * b_A->size[0];
      bkj = A_data[b_k * A->size[0] + j];
      for (b_i = 0; b_i < i1; b_i++) {
        i2 = coffset + b_i;
        y_data[i2] += k_data[aoffset + b_i] * bkj;
      }
    }
  }
  emxFree_real_T(&b_A);
  emxFree_real_T(&A);
  emxInit_real_T(&s, 1);
  i = s->size[0];
  s->size[0] = y->size[0];
  emxEnsureCapacity_real_T(s, i);
  A_data = s->data;
  i = (unsigned short)y->size[0];
  for (b_i = 0; b_i < i; b_i++) {
    A_data[b_i] = 0.0;
  }
  i1 = y->size[1];
  for (b_k = 0; b_k < i1; b_k++) {
    aoffset = b_k * y->size[0];
    for (b_i = 0; b_i < i; b_i++) {
      A_data[b_i] += y_data[aoffset + b_i] * VmKp184_data[b_k];
    }
  }
  emxFree_real_T(&y);
  i = k->size[0] * k->size[1];
  k->size[0] = VdebugN->size[0];
  k->size[1] = 2;
  emxEnsureCapacity_real_T(k, i);
  k_data = k->data;
  coffset = VdebugN->size[0];
  for (i = 0; i < 2; i++) {
    for (i1 = 0; i1 < coffset; i1++) {
      k_data[i1 + k->size[0] * i] = A_data[i + 2 * i1];
    }
  }
  emxFree_real_T(&s);
  emxInit_real_T(&c_k, 2);
  i = c_k->size[0] * c_k->size[1];
  c_k->size[0] = k->size[0];
  c_k->size[1] = 2;
  emxEnsureCapacity_real_T(c_k, i);
  A_data = c_k->data;
  coffset = k->size[0];
  for (i = 0; i < 2; i++) {
    for (i1 = 0; i1 < coffset; i1++) {
      A_data[i1 + c_k->size[0] * i] = k_data[i1 + k->size[0] * (1 - i)];
    }
  }
  for (i = 0; i < 2; i++) {
    for (i1 = 0; i1 < 16; i1++) {
      k_data[(Pac2Vid0[i1] + k->size[0] * i) - 1] = A_data[i1 + (i << 4)];
    }
  }
  emxFree_real_T(&c_k);
}

/*
 * File trailer for CalcK.c
 *
 * [EOF]
 */
