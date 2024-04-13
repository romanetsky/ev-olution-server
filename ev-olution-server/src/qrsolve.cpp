/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: qrsolve.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 09-Apr-2024 22:23:00
 */

/* Include Files */
#include "qrsolve.h"
#include "rt_nonfinite.h"
#include "xnrm2.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

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
 * Arguments    : const double A_data[]
 *                const int A_size[2]
 *                const double B_data[]
 *                int B_size
 *                double Y[2]
 * Return Type  : int
 */
int qrsolve(const double A_data[], const int A_size[2], const double B_data[],
            int B_size, double Y[2])
{
  double b_A_data[4096];
  double b_B_data[2048];
  double tau_data[2];
  double vn1[2];
  double vn2[2];
  double work[2];
  double atmp;
  double beta1;
  double temp;
  int A_size_idx_0;
  int b_i;
  int c_i;
  int exitg1;
  int i;
  int ii;
  int ip1;
  int ix;
  int jA;
  int knt;
  int lastv;
  int m;
  int ma;
  int mmi;
  int pvt;
  int rankA;
  int u0;
  signed char jpvt[2];
  A_size_idx_0 = A_size[0];
  i = A_size[0] << 1;
  if (i - 1 >= 0) {
    memcpy(&b_A_data[0], &A_data[0], (unsigned int)i * sizeof(double));
  }
  m = A_size[0];
  u0 = A_size[0];
  if (u0 > 2) {
    u0 = 2;
  }
  if (u0 - 1 >= 0) {
    memset(&tau_data[0], 0, (unsigned int)u0 * sizeof(double));
  }
  if (A_size[0] == 0) {
    jpvt[0] = 1;
    jpvt[1] = 2;
  } else {
    ma = A_size[0];
    jpvt[0] = 1;
    work[0] = 0.0;
    temp = xnrm2(A_size[0], A_data, 1);
    vn1[0] = temp;
    vn2[0] = temp;
    jpvt[1] = 2;
    work[1] = 0.0;
    temp = xnrm2(A_size[0], A_data, A_size[0] + 1);
    vn1[1] = temp;
    vn2[1] = temp;
    for (b_i = 0; b_i < u0; b_i++) {
      ip1 = b_i + 2;
      knt = b_i * ma;
      ii = knt + b_i;
      mmi = m - b_i;
      ix = 0;
      if ((2 - b_i > 1) && (fabs(vn1[b_i + 1]) > fabs(vn1[b_i]))) {
        ix = 1;
      }
      pvt = b_i + ix;
      if (pvt != b_i) {
        ix = pvt * ma;
        for (lastv = 0; lastv < m; lastv++) {
          i = ix + lastv;
          temp = b_A_data[i];
          c_i = knt + lastv;
          b_A_data[i] = b_A_data[c_i];
          b_A_data[c_i] = temp;
        }
        ix = jpvt[pvt];
        jpvt[pvt] = jpvt[b_i];
        jpvt[b_i] = (signed char)ix;
        vn1[pvt] = vn1[b_i];
        vn2[pvt] = vn2[b_i];
      }
      if (b_i + 1 < m) {
        atmp = b_A_data[ii];
        ix = ii + 2;
        tau_data[b_i] = 0.0;
        if (mmi > 0) {
          temp = xnrm2(mmi - 1, b_A_data, ii + 2);
          if (temp != 0.0) {
            beta1 = rt_hypotd_snf(b_A_data[ii], temp);
            if (b_A_data[ii] >= 0.0) {
              beta1 = -beta1;
            }
            if (fabs(beta1) < 1.0020841800044864E-292) {
              knt = 0;
              c_i = ii + mmi;
              do {
                knt++;
                for (lastv = ix; lastv <= c_i; lastv++) {
                  b_A_data[lastv - 1] *= 9.9792015476736E+291;
                }
                beta1 *= 9.9792015476736E+291;
                atmp *= 9.9792015476736E+291;
              } while ((fabs(beta1) < 1.0020841800044864E-292) && (knt < 20));
              beta1 = rt_hypotd_snf(atmp, xnrm2(mmi - 1, b_A_data, ii + 2));
              if (atmp >= 0.0) {
                beta1 = -beta1;
              }
              tau_data[b_i] = (beta1 - atmp) / beta1;
              temp = 1.0 / (atmp - beta1);
              for (lastv = ix; lastv <= c_i; lastv++) {
                b_A_data[lastv - 1] *= temp;
              }
              for (lastv = 0; lastv < knt; lastv++) {
                beta1 *= 1.0020841800044864E-292;
              }
              atmp = beta1;
            } else {
              tau_data[b_i] = (beta1 - b_A_data[ii]) / beta1;
              temp = 1.0 / (b_A_data[ii] - beta1);
              c_i = ii + mmi;
              for (lastv = ix; lastv <= c_i; lastv++) {
                b_A_data[lastv - 1] *= temp;
              }
              atmp = beta1;
            }
          }
        }
        b_A_data[ii] = atmp;
      } else {
        tau_data[b_i] = 0.0;
      }
      if (b_i + 1 < 2) {
        atmp = b_A_data[ii];
        b_A_data[ii] = 1.0;
        jA = (ii + ma) + 1;
        if (tau_data[0] != 0.0) {
          lastv = mmi - 1;
          i = (ii + mmi) - 1;
          while ((lastv + 1 > 0) && (b_A_data[i] == 0.0)) {
            lastv--;
            i--;
          }
          knt = 1;
          ix = jA;
          do {
            exitg1 = 0;
            if (ix <= jA + lastv) {
              if (b_A_data[ix - 1] != 0.0) {
                exitg1 = 1;
              } else {
                ix++;
              }
            } else {
              knt = 0;
              exitg1 = 1;
            }
          } while (exitg1 == 0);
        } else {
          lastv = -1;
          knt = 0;
        }
        if (lastv + 1 > 0) {
          if (knt != 0) {
            work[0] = 0.0;
            i = 0;
            for (pvt = jA; ma < 0 ? pvt >= jA : pvt <= jA; pvt += ma) {
              temp = 0.0;
              c_i = pvt + lastv;
              for (ix = pvt; ix <= c_i; ix++) {
                temp += b_A_data[ix - 1] * b_A_data[(ii + ix) - pvt];
              }
              work[i] += temp;
              i++;
            }
          }
          if (!(-tau_data[0] == 0.0)) {
            for (pvt = 0; pvt < knt; pvt++) {
              if (work[0] != 0.0) {
                temp = work[0] * -tau_data[0];
                c_i = lastv + jA;
                for (i = jA; i <= c_i; i++) {
                  b_A_data[i - 1] += b_A_data[(ii + i) - jA] * temp;
                }
              }
              jA += ma;
            }
          }
        }
        b_A_data[ii] = atmp;
      }
      for (pvt = ip1; pvt < 3; pvt++) {
        i = b_i + ma;
        if (vn1[1] != 0.0) {
          temp = fabs(b_A_data[i]) / vn1[1];
          temp = 1.0 - temp * temp;
          if (temp < 0.0) {
            temp = 0.0;
          }
          beta1 = vn1[1] / vn2[1];
          beta1 = temp * (beta1 * beta1);
          if (beta1 <= 1.4901161193847656E-8) {
            if (b_i + 1 < m) {
              temp = xnrm2(mmi - 1, b_A_data, i + 2);
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
  rankA = 0;
  if (A_size[0] < 2) {
    i = A_size[0];
    ix = 2;
  } else {
    i = 2;
    ix = A_size[0];
  }
  if (i > 0) {
    temp = 2.2204460492503131E-15 * (double)ix * fabs(b_A_data[0]);
    while ((rankA < i) &&
           (!(fabs(b_A_data[rankA + A_size_idx_0 * rankA]) <= temp))) {
      rankA++;
    }
  }
  knt = 0;
  if (u0 > 0) {
    for (lastv = 0; lastv < u0; lastv++) {
      if (b_A_data[lastv + A_size_idx_0 * lastv] != 0.0) {
        knt++;
      }
    }
  }
  if (B_size - 1 >= 0) {
    memcpy(&b_B_data[0], &B_data[0], (unsigned int)B_size * sizeof(double));
  }
  Y[0] = 0.0;
  Y[1] = 0.0;
  for (pvt = 0; pvt < u0; pvt++) {
    if (tau_data[pvt] != 0.0) {
      temp = b_B_data[pvt];
      c_i = pvt + 2;
      for (b_i = c_i; b_i <= A_size_idx_0; b_i++) {
        temp += b_A_data[(b_i + A_size_idx_0 * pvt) - 1] * b_B_data[b_i - 1];
      }
      temp *= tau_data[pvt];
      if (temp != 0.0) {
        b_B_data[pvt] -= temp;
        for (b_i = c_i; b_i <= A_size_idx_0; b_i++) {
          b_B_data[b_i - 1] -= b_A_data[(b_i + A_size_idx_0 * pvt) - 1] * temp;
        }
      }
    }
  }
  for (b_i = 0; b_i < knt; b_i++) {
    Y[jpvt[b_i] - 1] = b_B_data[b_i];
  }
  for (pvt = knt; pvt >= 1; pvt--) {
    i = jpvt[pvt - 1] - 1;
    ix = A_size_idx_0 * (pvt - 1);
    Y[i] /= b_A_data[(pvt + ix) - 1];
    for (b_i = 0; b_i <= pvt - 2; b_i++) {
      Y[jpvt[0] - 1] -= Y[i] * b_A_data[ix];
    }
  }
  return rankA;
}

/*
 * File trailer for qrsolve.c
 *
 * [EOF]
 */
