/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: CalcK.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Apr-2024 13:06:24
 */

/* Include Files */
#include "CalcK.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const double VdebugN_data[]
 *                const int VdebugN_size[2]
 *                const double VdebugVec_data[]
 *                const int VdebugVec_size[2]
 *                const signed char VIpacId[16]
 *                const unsigned char Pac2Vid0[16]
 *                const double VmKp184Test_data[]
 *                int VmKp184Test_size
 *                double k_data[]
 *                int k_size[2]
 * Return Type  : void
 */
void CalcK(const double VdebugN_data[], const int VdebugN_size[2],
           const double VdebugVec_data[], const int VdebugVec_size[2],
           const signed char VIpacId[16], const unsigned char Pac2Vid0[16],
           const double VmKp184Test_data[], int VmKp184Test_size,
           double k_data[], int k_size[2])
{
  double A_data[1024];
  double a_data[1024];
  double x_data[1024];
  double VdebugVec1_data[256];
  double Vdebug1N1_data[32];
  double VdebugN1_data[32];
  double x[32];
  double Vdebug3[16];
  double Vdebug4[16];
  double VmKp184_data[16];
  double bkj;
  double s;
  int A_size_idx_0;
  int a_size_idx_0;
  int a_size_idx_1;
  int b_tmp;
  int i;
  int i1;
  int j;
  int jA;
  int jp1j;
  int k;
  int k_bat;
  int mmj_tmp;
  int u0;
  int yk;
  signed char ipiv_data[32];
  signed char p_data[32];
  signed char i2;
  yk = VdebugVec_size[1];
  for (i = 0; i < yk; i++) {
    for (i1 = 0; i1 < 16; i1++) {
      VdebugVec1_data[i1 + 16 * i] =
          VdebugVec_data[(VIpacId[i1] + VdebugVec_size[0] * i) - 1];
    }
  }
  /*  Vdebug2 = squeeze(twos_comp(VdebugVec( :,1,:)*256 + VdebugVec(
   * :,2,:),Nbits)/2^15*9); */
  /*  twos_comp converts input variable */
  /*  "data" to it's twos complement */
  /*  value. Input variable "data" is */
  /*  expected to be hexadecimal. */
  for (yk = 0; yk < 16; yk++) {
    Vdebug4[yk] = (VdebugVec1_data[yk] * 256.0 + VdebugVec1_data[yk + 16]) /
                  32768.0 * 9.0;
  }
  yk = VdebugN_size[1];
  /*  VdebugN2 = squeeze(twos_comp(VdebugN( :,1)*256 + VdebugN(
   * :,2),Nbits)/2^15*9); */
  /*  twos_comp converts input variable */
  /*  "data" to it's twos complement */
  /*  value. Input variable "data" is */
  /*  expected to be hexadecimal. */
  for (i = 0; i < yk; i++) {
    for (i1 = 0; i1 < 16; i1++) {
      bkj = VdebugN_data[(VIpacId[i1] + VdebugN_size[0] * i) - 1];
      jA = i1 + 16 * i;
      VdebugN1_data[jA] = bkj;
      Vdebug1N1_data[jA] = bkj;
    }
  }
  /*  Vdebug1N2 = squeeze(twos_comp(Vdebug1N( :,1)*256 + Vdebug1N(
   * :,2),Nbits)/2^15*9); */
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
  for (k = 0; k < 15; k++) {
    Vdebug3[k + 1] = Vdebug3[k] + VmKp184_data[(signed char)Pac2Vid0[k] - 1];
  }
  A_size_idx_0 = VdebugN_size[0] << 1;
  jp1j = A_size_idx_0 * A_size_idx_0;
  if (jp1j - 1 >= 0) {
    memset(&A_data[0], 0, (unsigned int)jp1j * sizeof(double));
  }
  i = VdebugN_size[0];
  for (k_bat = 0; k_bat < i; k_bat++) {
    yk = k_bat << 1;
    bkj = VmKp184_data[Pac2Vid0[k_bat] - 1];
    s = Vdebug3[k_bat];
    jA = A_size_idx_0 * yk;
    A_data[k_bat + jA] = bkj + s;
    A_data[k_bat + A_size_idx_0 * (yk + 1)] = -s;
    A_data[(k_bat + VdebugN_size[0]) + jA] = bkj;
  }
  bkj = VmKp184_data[Pac2Vid0[(int)((double)VdebugN_size[0] / 2.0) - 1] - 1];
  A_data[0] = VmKp184_data[Pac2Vid0[0] - 1] + bkj;
  A_data[A_size_idx_0] = -bkj;
  for (i = 0; i < 16; i++) {
    yk = Pac2Vid0[i] - 1;
    x[i] = (VdebugN1_data[yk] * 256.0 + VdebugN1_data[yk + 16]) / 32768.0 * 9.0;
    x[i + 16] = Vdebug4[i];
  }
  x[0] = (Vdebug1N1_data[Pac2Vid0[0] - 1] * 256.0 +
          Vdebug1N1_data[Pac2Vid0[0] + 15]) /
         32768.0 * 9.0;
  for (j = 0; j < A_size_idx_0; j++) {
    yk = j * A_size_idx_0;
    if (A_size_idx_0 - 1 >= 0) {
      memset(&x_data[yk], 0, (unsigned int)A_size_idx_0 * sizeof(double));
    }
    for (k = 0; k < A_size_idx_0; k++) {
      bkj = A_data[yk + k];
      for (k_bat = 0; k_bat < A_size_idx_0; k_bat++) {
        i = yk + k_bat;
        x_data[i] += A_data[k_bat * A_size_idx_0 + k] * bkj;
      }
    }
  }
  a_size_idx_0 = (signed char)A_size_idx_0;
  a_size_idx_1 = (signed char)A_size_idx_0;
  if (A_size_idx_0 == 1) {
    a_data[0] = 1.0 / x_data[0];
  } else if (A_size_idx_0 == 0) {
    a_size_idx_0 = 0;
    a_size_idx_1 = 0;
    if (jp1j - 1 >= 0) {
      memcpy(&a_data[0], &x_data[0], (unsigned int)jp1j * sizeof(double));
    }
  } else {
    a_size_idx_0 = A_size_idx_0;
    a_size_idx_1 = A_size_idx_0;
    memset(&a_data[0], 0, (unsigned int)jp1j * sizeof(double));
    ipiv_data[0] = 1;
    yk = 1;
    for (k = 2; k <= A_size_idx_0; k++) {
      yk++;
      ipiv_data[k - 1] = (signed char)yk;
    }
    u0 = A_size_idx_0 - 1;
    if (u0 > A_size_idx_0) {
      u0 = A_size_idx_0;
    }
    for (j = 0; j < u0; j++) {
      mmj_tmp = A_size_idx_0 - j;
      b_tmp = j * (A_size_idx_0 + 1);
      jp1j = b_tmp + 2;
      if (mmj_tmp < 1) {
        yk = -1;
      } else {
        yk = 0;
        if (mmj_tmp > 1) {
          bkj = fabs(x_data[b_tmp]);
          for (k = 2; k <= mmj_tmp; k++) {
            s = fabs(x_data[(b_tmp + k) - 1]);
            if (s > bkj) {
              yk = k - 1;
              bkj = s;
            }
          }
        }
      }
      if (x_data[b_tmp + yk] != 0.0) {
        if (yk != 0) {
          jA = j + yk;
          ipiv_data[j] = (signed char)(jA + 1);
          for (k = 0; k < A_size_idx_0; k++) {
            yk = k * A_size_idx_0;
            k_bat = j + yk;
            bkj = x_data[k_bat];
            i = jA + yk;
            x_data[k_bat] = x_data[i];
            x_data[i] = bkj;
          }
        }
        i = b_tmp + mmj_tmp;
        for (k_bat = jp1j; k_bat <= i; k_bat++) {
          x_data[k_bat - 1] /= x_data[b_tmp];
        }
      }
      yk = b_tmp + A_size_idx_0;
      jA = yk;
      for (k_bat = 0; k_bat <= mmj_tmp - 2; k_bat++) {
        bkj = x_data[yk + k_bat * A_size_idx_0];
        if (bkj != 0.0) {
          i = jA + 2;
          i1 = mmj_tmp + jA;
          for (jp1j = i; jp1j <= i1; jp1j++) {
            x_data[jp1j - 1] += x_data[((b_tmp + jp1j) - jA) - 1] * -bkj;
          }
        }
        jA += A_size_idx_0;
      }
    }
    p_data[0] = 1;
    yk = 1;
    for (k = 2; k <= A_size_idx_0; k++) {
      yk++;
      p_data[k - 1] = (signed char)yk;
    }
    for (k = 0; k < A_size_idx_0; k++) {
      i2 = ipiv_data[k];
      if (i2 > k + 1) {
        jA = p_data[i2 - 1];
        p_data[i2 - 1] = p_data[k];
        p_data[k] = (signed char)jA;
      }
    }
    for (k = 0; k < A_size_idx_0; k++) {
      i2 = p_data[k];
      yk = A_size_idx_0 * (i2 - 1);
      a_data[k + yk] = 1.0;
      for (j = k + 1; j <= A_size_idx_0; j++) {
        i = (j + A_size_idx_0 * (i2 - 1)) - 1;
        if (a_data[i] != 0.0) {
          i1 = j + 1;
          for (k_bat = i1; k_bat <= A_size_idx_0; k_bat++) {
            jA = (k_bat + yk) - 1;
            a_data[jA] -=
                a_data[i] * x_data[(k_bat + A_size_idx_0 * (j - 1)) - 1];
          }
        }
      }
    }
    for (j = 0; j < A_size_idx_0; j++) {
      yk = A_size_idx_0 * j - 1;
      for (k = A_size_idx_0; k >= 1; k--) {
        jA = A_size_idx_0 * (k - 1) - 1;
        i = k + yk;
        s = a_data[i];
        if (s != 0.0) {
          a_data[i] = s / x_data[k + jA];
          for (k_bat = 0; k_bat <= k - 2; k_bat++) {
            i1 = (k_bat + yk) + 1;
            a_data[i1] -= a_data[i] * x_data[(k_bat + jA) + 1];
          }
        }
      }
    }
  }
  for (j = 0; j < A_size_idx_0; j++) {
    yk = j * a_size_idx_0;
    if (a_size_idx_0 - 1 >= 0) {
      memset(&x_data[yk], 0, (unsigned int)a_size_idx_0 * sizeof(double));
    }
    for (k = 0; k < a_size_idx_1; k++) {
      jA = k * a_size_idx_0;
      bkj = A_data[k * A_size_idx_0 + j];
      for (k_bat = 0; k_bat < a_size_idx_0; k_bat++) {
        i = yk + k_bat;
        x_data[i] += a_data[jA + k_bat] * bkj;
      }
    }
  }
  yk = a_size_idx_0 - 1;
  if (yk >= 0) {
    memset(&VdebugN1_data[0], 0, (unsigned int)(yk + 1) * sizeof(double));
  }
  for (k = 0; k < A_size_idx_0; k++) {
    jA = k * a_size_idx_0;
    for (k_bat = 0; k_bat <= yk; k_bat++) {
      VdebugN1_data[k_bat] += x_data[jA + k_bat] * x[k];
    }
  }
  k_size[0] = VdebugN_size[0];
  k_size[1] = 2;
  yk = VdebugN_size[0];
  for (i = 0; i < 2; i++) {
    for (i1 = 0; i1 < yk; i1++) {
      k_data[i1 + k_size[0] * i] = VdebugN1_data[i + 2 * i1];
    }
  }
  for (i = 0; i < 2; i++) {
    for (i1 = 0; i1 < 16; i1++) {
      VdebugN1_data[i1 + (i << 4)] = k_data[i1 + k_size[0] * (1 - i)];
    }
  }
  for (i = 0; i < 2; i++) {
    for (i1 = 0; i1 < 16; i1++) {
      k_data[(Pac2Vid0[i1] + k_size[0] * i) - 1] = VdebugN1_data[i1 + (i << 4)];
    }
  }
}

/*
 * File trailer for CalcK.c
 *
 * [EOF]
 */
