/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: mpower.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 17-Mar-2024 11:09:30
 */

/* Include Files */
#include "mpower.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_emxutil.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : const emxArray_real_T *a
 *                emxArray_real_T *c
 * Return Type  : void
 */
void mpower(const emxArray_real_T *a, emxArray_real_T *c)
{
  emxArray_int32_T *ipiv;
  emxArray_int32_T *p;
  emxArray_real_T *x;
  const double *a_data;
  double s;
  double smax;
  double *c_data;
  double *x_data;
  int b_i;
  int b_tmp;
  int i;
  int i1;
  int i2;
  int ipiv_tmp;
  int j;
  int jA;
  int jp1j;
  int k;
  int mmj_tmp;
  int n;
  int n_tmp;
  int yk;
  int *ipiv_data;
  int *p_data;
  a_data = a->data;
  i = c->size[0] * c->size[1];
  c->size[0] = a->size[0];
  c->size[1] = a->size[1];
  emxEnsureCapacity_real_T(c, i);
  c_data = c->data;
  if ((a->size[0] == 1) && (a->size[1] == 1)) {
    c_data[0] = 1.0 / a_data[0];
  } else if ((a->size[0] == 0) || (a->size[1] == 0)) {
    i = c->size[0] * c->size[1];
    c->size[0] = a->size[0];
    c->size[1] = a->size[1];
    emxEnsureCapacity_real_T(c, i);
    c_data = c->data;
    yk = a->size[0] * a->size[1];
    for (i = 0; i < yk; i++) {
      c_data[i] = a_data[i];
    }
  } else {
    n = a->size[0];
    i = c->size[0] * c->size[1];
    c->size[0] = a->size[0];
    c->size[1] = a->size[1];
    emxEnsureCapacity_real_T(c, i);
    c_data = c->data;
    yk = a->size[0] * a->size[1];
    for (i = 0; i < yk; i++) {
      c_data[i] = 0.0;
    }
    emxInit_real_T(&x, 2);
    i = x->size[0] * x->size[1];
    x->size[0] = a->size[0];
    x->size[1] = a->size[1];
    emxEnsureCapacity_real_T(x, i);
    x_data = x->data;
    for (i = 0; i < yk; i++) {
      x_data[i] = a_data[i];
    }
    n_tmp = (unsigned short)(a->size[0] - 1) + 1;
    emxInit_int32_T(&ipiv, 2);
    i = ipiv->size[0] * ipiv->size[1];
    ipiv->size[0] = 1;
    ipiv->size[1] = (unsigned short)(a->size[0] - 1) + 1;
    emxEnsureCapacity_int32_T(ipiv, i);
    ipiv_data = ipiv->data;
    ipiv_data[0] = 1;
    yk = 1;
    for (k = 2; k <= n_tmp; k++) {
      yk++;
      ipiv_data[k - 1] = yk;
    }
    yk = a->size[0] - 1;
    jA = a->size[0];
    if (yk <= jA) {
      jA = yk;
    }
    i = (unsigned short)jA;
    for (j = 0; j < i; j++) {
      mmj_tmp = n - j;
      b_tmp = j * (n + 1);
      jp1j = b_tmp + 2;
      if (mmj_tmp < 1) {
        yk = -1;
      } else {
        yk = 0;
        if (mmj_tmp > 1) {
          smax = fabs(x_data[b_tmp]);
          for (k = 2; k <= mmj_tmp; k++) {
            s = fabs(x_data[(b_tmp + k) - 1]);
            if (s > smax) {
              yk = k - 1;
              smax = s;
            }
          }
        }
      }
      if (x_data[b_tmp + yk] != 0.0) {
        if (yk != 0) {
          ipiv_tmp = j + yk;
          ipiv_data[j] = ipiv_tmp + 1;
          i1 = (unsigned short)n;
          for (k = 0; k < i1; k++) {
            yk = k * n;
            jA = j + yk;
            smax = x_data[jA];
            i2 = ipiv_tmp + yk;
            x_data[jA] = x_data[i2];
            x_data[i2] = smax;
          }
        }
        i1 = b_tmp + mmj_tmp;
        for (b_i = jp1j; b_i <= i1; b_i++) {
          x_data[b_i - 1] /= x_data[b_tmp];
        }
      }
      yk = b_tmp + n;
      jA = yk;
      for (ipiv_tmp = 0; ipiv_tmp <= mmj_tmp - 2; ipiv_tmp++) {
        smax = x_data[yk + ipiv_tmp * n];
        if (smax != 0.0) {
          i1 = jA + 2;
          i2 = mmj_tmp + jA;
          for (jp1j = i1; jp1j <= i2; jp1j++) {
            x_data[jp1j - 1] += x_data[((b_tmp + jp1j) - jA) - 1] * -smax;
          }
        }
        jA += n;
      }
    }
    emxInit_int32_T(&p, 2);
    i = p->size[0] * p->size[1];
    p->size[0] = 1;
    p->size[1] = (unsigned short)(a->size[0] - 1) + 1;
    emxEnsureCapacity_int32_T(p, i);
    p_data = p->data;
    p_data[0] = 1;
    yk = 1;
    for (k = 2; k <= n_tmp; k++) {
      yk++;
      p_data[k - 1] = yk;
    }
    i = ipiv->size[1];
    for (k = 0; k < i; k++) {
      i1 = ipiv_data[k];
      if (i1 > k + 1) {
        yk = p_data[i1 - 1];
        p_data[i1 - 1] = p_data[k];
        p_data[k] = yk;
      }
    }
    emxFree_int32_T(&ipiv);
    i = (unsigned short)a->size[0];
    for (k = 0; k < i; k++) {
      i1 = p_data[k];
      c_data[k + c->size[0] * (i1 - 1)] = 1.0;
      for (j = k + 1; j <= n; j++) {
        if (c_data[(j + c->size[0] * (i1 - 1)) - 1] != 0.0) {
          i2 = j + 1;
          for (b_i = i2; b_i <= n; b_i++) {
            c_data[(b_i + c->size[0] * (i1 - 1)) - 1] -=
                c_data[(j + c->size[0] * (i1 - 1)) - 1] *
                x_data[(b_i + x->size[0] * (j - 1)) - 1];
          }
        }
      }
    }
    emxFree_int32_T(&p);
    for (j = 0; j < i; j++) {
      yk = n * j - 1;
      for (k = n; k >= 1; k--) {
        jA = n * (k - 1) - 1;
        i1 = k + yk;
        smax = c_data[i1];
        if (smax != 0.0) {
          c_data[i1] = smax / x_data[k + jA];
          i2 = (unsigned short)(k - 1);
          for (b_i = 0; b_i < i2; b_i++) {
            ipiv_tmp = (b_i + yk) + 1;
            c_data[ipiv_tmp] -= c_data[i1] * x_data[(b_i + jA) + 1];
          }
        }
      }
    }
    emxFree_real_T(&x);
  }
}

/*
 * File trailer for mpower.c
 *
 * [EOF]
 */
