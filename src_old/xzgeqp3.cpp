/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xzgeqp3.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 17-Mar-2024 11:09:30
 */

/* Include Files */
#include "xzgeqp3.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h"
#include "rt_nonfinite.h"
#include "xnrm2.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Declarations */
static double rt_hypotd_snf(double u0, double u1);

/* Function Definitions */
/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_hypotd_snf(double u0, double u1)
{
  double a;
  double b;
  double y;
  a = fabs(u0);
  b = fabs(u1);
  if (a < b) {
    a /= b;
    y = b * sqrt(a * a + 1.0);
  } else if (a > b) {
    b /= a;
    y = a * sqrt(b * b + 1.0);
  } else if (rtIsNaN(b)) {
    y = rtNaN;
  } else {
    y = a * 1.4142135623730951;
  }
  return y;
}

/*
 * Arguments    : emxArray_real_T *A
 *                int m
 *                double tau_data[]
 *                int jpvt[2]
 * Return Type  : void
 */
void qrpf(emxArray_real_T *A, int m, double tau_data[], int jpvt[2])
{
  double vn1[2];
  double vn2[2];
  double work[2];
  double atmp;
  double beta1;
  double temp;
  double *A_data;
  int b_i;
  int exitg1;
  int i;
  int i1;
  int ii;
  int ip1;
  int ix;
  int jA;
  int k;
  int knt;
  int lastv;
  int ma;
  int mmi;
  int pvt;
  A_data = A->data;
  ma = A->size[0];
  work[0] = 0.0;
  temp = xnrm2(m, A, 1);
  vn1[0] = temp;
  vn2[0] = temp;
  work[1] = 0.0;
  temp = xnrm2(m, A, ma + 1);
  vn1[1] = temp;
  vn2[1] = temp;
  if (m <= 2) {
    ix = m;
  } else {
    ix = 2;
  }
  i = (unsigned char)ix;
  for (b_i = 0; b_i < i; b_i++) {
    ip1 = b_i + 2;
    jA = b_i * ma;
    ii = jA + b_i;
    mmi = m - b_i;
    ix = 0;
    if ((2 - b_i > 1) && (fabs(vn1[b_i + 1]) > fabs(vn1[b_i]))) {
      ix = 1;
    }
    pvt = b_i + ix;
    if (pvt != b_i) {
      ix = pvt * ma;
      i1 = (unsigned short)m;
      for (k = 0; k < i1; k++) {
        knt = ix + k;
        temp = A_data[knt];
        lastv = jA + k;
        A_data[knt] = A_data[lastv];
        A_data[lastv] = temp;
      }
      ix = jpvt[pvt];
      jpvt[pvt] = jpvt[b_i];
      jpvt[b_i] = ix;
      vn1[pvt] = vn1[b_i];
      vn2[pvt] = vn2[b_i];
    }
    if (b_i + 1 < m) {
      atmp = A_data[ii];
      ix = ii + 2;
      tau_data[b_i] = 0.0;
      if (mmi > 0) {
        temp = xnrm2(mmi - 1, A, ii + 2);
        if (temp != 0.0) {
          beta1 = rt_hypotd_snf(A_data[ii], temp);
          if (A_data[ii] >= 0.0) {
            beta1 = -beta1;
          }
          if (fabs(beta1) < 1.0020841800044864E-292) {
            knt = 0;
            i1 = ii + mmi;
            do {
              knt++;
              for (k = ix; k <= i1; k++) {
                A_data[k - 1] *= 9.9792015476736E+291;
              }
              beta1 *= 9.9792015476736E+291;
              atmp *= 9.9792015476736E+291;
            } while ((fabs(beta1) < 1.0020841800044864E-292) && (knt < 20));
            beta1 = rt_hypotd_snf(atmp, xnrm2(mmi - 1, A, ii + 2));
            if (atmp >= 0.0) {
              beta1 = -beta1;
            }
            tau_data[b_i] = (beta1 - atmp) / beta1;
            temp = 1.0 / (atmp - beta1);
            for (k = ix; k <= i1; k++) {
              A_data[k - 1] *= temp;
            }
            for (k = 0; k < knt; k++) {
              beta1 *= 1.0020841800044864E-292;
            }
            atmp = beta1;
          } else {
            tau_data[b_i] = (beta1 - A_data[ii]) / beta1;
            temp = 1.0 / (A_data[ii] - beta1);
            i1 = ii + mmi;
            for (k = ix; k <= i1; k++) {
              A_data[k - 1] *= temp;
            }
            atmp = beta1;
          }
        }
      }
      A_data[ii] = atmp;
    } else {
      tau_data[b_i] = 0.0;
    }
    if (b_i + 1 < 2) {
      atmp = A_data[ii];
      A_data[ii] = 1.0;
      jA = (ii + ma) + 1;
      if (tau_data[0] != 0.0) {
        lastv = mmi - 1;
        ix = (ii + mmi) - 1;
        while ((lastv + 1 > 0) && (A_data[ix] == 0.0)) {
          lastv--;
          ix--;
        }
        ix = 1;
        pvt = jA;
        do {
          exitg1 = 0;
          if (pvt <= jA + lastv) {
            if (A_data[pvt - 1] != 0.0) {
              exitg1 = 1;
            } else {
              pvt++;
            }
          } else {
            ix = 0;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      } else {
        lastv = -1;
        ix = 0;
      }
      if (lastv + 1 > 0) {
        if (ix != 0) {
          work[0] = 0.0;
          knt = 0;
          for (k = jA; ma < 0 ? k >= jA : k <= jA; k += ma) {
            temp = 0.0;
            i1 = k + lastv;
            for (pvt = k; pvt <= i1; pvt++) {
              temp += A_data[pvt - 1] * A_data[(ii + pvt) - k];
            }
            work[knt] += temp;
            knt++;
          }
        }
        if (!(-tau_data[0] == 0.0)) {
          for (pvt = 0; pvt < ix; pvt++) {
            if (work[0] != 0.0) {
              temp = work[0] * -tau_data[0];
              i1 = lastv + jA;
              for (knt = jA; knt <= i1; knt++) {
                A_data[knt - 1] += A_data[(ii + knt) - jA] * temp;
              }
            }
            jA += ma;
          }
        }
      }
      A_data[ii] = atmp;
    }
    for (pvt = ip1; pvt < 3; pvt++) {
      ix = b_i + ma;
      if (vn1[1] != 0.0) {
        temp = fabs(A_data[ix]) / vn1[1];
        temp = 1.0 - temp * temp;
        if (temp < 0.0) {
          temp = 0.0;
        }
        beta1 = vn1[1] / vn2[1];
        beta1 = temp * (beta1 * beta1);
        if (beta1 <= 1.4901161193847656E-8) {
          if (b_i + 1 < m) {
            temp = xnrm2(mmi - 1, A, ix + 2);
            vn1[1] = temp;
            vn2[1] = temp;
          } else {
            vn1[1] = 0.0;
            vn2[1] = 0.0;
          }
        } else {
          vn1[1] *= sqrt(temp);
        }
      }
    }
  }
}

/*
 * File trailer for xzgeqp3.c
 *
 * [EOF]
 */
