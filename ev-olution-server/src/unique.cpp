/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: unique.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 17-Mar-2024 11:09:30
 */

/* Include Files */
#include "unique.h"
#include "rt_nonfinite.h"
#include "sortLE.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const double a_data[]
 *                const int a_size[2]
 *                double b_data[]
 *                int b_size[2]
 * Return Type  : void
 */
void unique_rows(const double a_data[], const int a_size[2], double b_data[],
                 int b_size[2])
{
  double ycol_data[16];
  int col_data[16];
  int idx_data[16];
  int iwork_data[16];
  int col_size[2];
  int b_i;
  int exitg1;
  int i;
  int i1;
  int i2;
  int j;
  int k;
  int kEnd;
  int n;
  int p;
  int pEnd;
  int q;
  int qEnd;
  boolean_T b_p;
  boolean_T exitg2;
  b_size[0] = a_size[0];
  b_size[1] = a_size[1];
  pEnd = a_size[0] * a_size[1];
  if (pEnd - 1 >= 0) {
    memcpy(&b_data[0], &a_data[0], (unsigned int)pEnd * sizeof(double));
  }
  n = a_size[1];
  col_size[0] = 1;
  col_size[1] = a_size[1];
  for (k = 0; k < n; k++) {
    col_data[k] = k + 1;
  }
  n = a_size[0] + 1;
  pEnd = a_size[0];
  if (pEnd - 1 >= 0) {
    memset(&idx_data[0], 0, (unsigned int)pEnd * sizeof(int));
  }
  if (a_size[1] == 0) {
    for (k = 0; k <= n - 2; k++) {
      idx_data[k] = k + 1;
    }
  } else {
    i = a_size[0] - 1;
    for (k = 1; k <= i; k += 2) {
      if (sortLE(a_data, a_size, col_data, col_size, k, k + 1)) {
        idx_data[k - 1] = k;
        idx_data[k] = k + 1;
      } else {
        idx_data[k - 1] = k + 1;
        idx_data[k] = k;
      }
    }
    if ((a_size[0] & 1) != 0) {
      idx_data[a_size[0] - 1] = a_size[0];
    }
    b_i = 2;
    while (b_i < n - 1) {
      i2 = b_i << 1;
      j = 1;
      for (pEnd = b_i + 1; pEnd < n; pEnd = qEnd + b_i) {
        p = j;
        q = pEnd;
        qEnd = j + i2;
        if (qEnd > n) {
          qEnd = n;
        }
        k = 0;
        kEnd = qEnd - j;
        while (k + 1 <= kEnd) {
          i = idx_data[p - 1];
          i1 = idx_data[q - 1];
          if (sortLE(a_data, a_size, col_data, col_size, i, i1)) {
            iwork_data[k] = i;
            p++;
            if (p == pEnd) {
              while (q < qEnd) {
                k++;
                iwork_data[k] = idx_data[q - 1];
                q++;
              }
            }
          } else {
            iwork_data[k] = i1;
            q++;
            if (q == qEnd) {
              while (p < pEnd) {
                k++;
                iwork_data[k] = idx_data[p - 1];
                p++;
              }
            }
          }
          k++;
        }
        for (k = 0; k < kEnd; k++) {
          idx_data[(j + k) - 1] = iwork_data[k];
        }
        j = qEnd;
      }
      b_i = i2;
    }
  }
  i2 = a_size[0] - 1;
  i = a_size[1];
  for (j = 0; j < i; j++) {
    for (b_i = 0; b_i <= i2; b_i++) {
      ycol_data[b_i] = b_data[(idx_data[b_i] + b_size[0] * j) - 1];
    }
    for (b_i = 0; b_i <= i2; b_i++) {
      b_data[b_i + b_size[0] * j] = ycol_data[b_i];
    }
  }
  i2 = -1;
  pEnd = a_size[0];
  k = 0;
  while (k + 1 <= pEnd) {
    p = k;
    do {
      exitg1 = 0;
      k++;
      if (k + 1 > pEnd) {
        exitg1 = 1;
      } else {
        b_p = false;
        j = 0;
        exitg2 = false;
        while ((!exitg2) && (j <= b_size[1] - 1)) {
          i = b_size[0] * j;
          if (b_data[p + i] != b_data[k + i]) {
            b_p = true;
            exitg2 = true;
          } else {
            j++;
          }
        }
        if (b_p) {
          exitg1 = 1;
        }
      }
    } while (exitg1 == 0);
    i2++;
    i = b_size[1];
    for (j = 0; j < i; j++) {
      q = b_size[0] * j;
      b_data[i2 + q] = b_data[p + q];
    }
  }
  if (i2 + 1 < 1) {
    pEnd = 0;
  } else {
    pEnd = i2 + 1;
  }
  i2 = a_size[1];
  for (i = 0; i < i2; i++) {
    for (i1 = 0; i1 < pEnd; i1++) {
      b_data[i1 + pEnd * i] = b_data[i1 + b_size[0] * i];
    }
  }
  b_size[0] = pEnd;
}

/*
 * Arguments    : const short a_data[]
 *                int a_size
 *                short b_data[]
 *                int ndx_data[]
 *                int *ndx_size
 *                int pos_data[]
 *                int *pos_size
 * Return Type  : int
 */
int unique_vector(const short a_data[], int a_size, short b_data[],
                  int ndx_data[], int *ndx_size, int pos_data[], int *pos_size)
{
  int idx_data[2];
  int b_size;
  int i;
  int j;
  int k;
  int k0;
  short x;
  *pos_size = a_size;
  if (a_size - 1 >= 0) {
    memset(&idx_data[0], 0, (unsigned int)a_size * sizeof(int));
  }
  if (a_size != 0) {
    if (a_size - 1 >= 1) {
      if (a_data[0] <= a_data[1]) {
        idx_data[0] = 1;
        idx_data[1] = 2;
      } else {
        idx_data[0] = 2;
        idx_data[1] = 1;
      }
    }
    if ((a_size & 1) != 0) {
      idx_data[a_size - 1] = a_size;
    }
  }
  for (k = 0; k < a_size; k++) {
    b_data[k] = a_data[idx_data[k] - 1];
  }
  *ndx_size = 0;
  k = 1;
  while (k <= a_size) {
    x = b_data[k - 1];
    k0 = k;
    do {
      k++;
    } while (!((k > a_size) || (b_data[1] != x)));
    (*ndx_size)++;
    b_data[*ndx_size - 1] = x;
    i = k - 1;
    for (j = k0; j <= i; j++) {
      pos_data[idx_data[j - 1] - 1] = *ndx_size;
    }
    idx_data[*ndx_size - 1] = idx_data[k0 - 1];
  }
  if (*ndx_size < 1) {
    b_size = 0;
  } else {
    b_size = *ndx_size;
  }
  i = (unsigned char)*ndx_size;
  if (i - 1 >= 0) {
    memcpy(&ndx_data[0], &idx_data[0], (unsigned int)i * sizeof(int));
  }
  return b_size;
}

/*
 * File trailer for unique.c
 *
 * [EOF]
 */
