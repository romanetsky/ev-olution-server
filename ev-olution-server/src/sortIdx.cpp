/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: sortIdx.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Apr-2024 17:01:48
 */

/* Include Files */
#include "sortIdx.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Declarations */
static void c_merge(int idx_data[], double x_data[], int offset, int np, int nq,
                    int iwork_data[], double xwork_data[]);

static void merge(emxArray_int32_T *idx, short x_data[], int offset, int np,
                  int nq, emxArray_int32_T *iwork, short xwork_data[]);

/* Function Definitions */
/*
 * Arguments    : int idx_data[]
 *                double x_data[]
 *                int offset
 *                int np
 *                int nq
 *                int iwork_data[]
 *                double xwork_data[]
 * Return Type  : void
 */
static void c_merge(int idx_data[], double x_data[], int offset, int np, int nq,
                    int iwork_data[], double xwork_data[])
{
  int exitg1;
  int iout;
  int j;
  int n_tmp;
  int p;
  int q;
  if (nq != 0) {
    n_tmp = np + nq;
    for (j = 0; j < n_tmp; j++) {
      iout = offset + j;
      iwork_data[j] = idx_data[iout];
      xwork_data[j] = x_data[iout];
    }
    p = 0;
    q = np;
    iout = offset - 1;
    do {
      exitg1 = 0;
      iout++;
      if (xwork_data[p] <= xwork_data[q]) {
        idx_data[iout] = iwork_data[p];
        x_data[iout] = xwork_data[p];
        if (p + 1 < np) {
          p++;
        } else {
          exitg1 = 1;
        }
      } else {
        idx_data[iout] = iwork_data[q];
        x_data[iout] = xwork_data[q];
        if (q + 1 < n_tmp) {
          q++;
        } else {
          q = iout - p;
          for (j = p + 1; j <= np; j++) {
            iout = q + j;
            idx_data[iout] = iwork_data[j - 1];
            x_data[iout] = xwork_data[j - 1];
          }
          exitg1 = 1;
        }
      }
    } while (exitg1 == 0);
  }
}

/*
 * Arguments    : emxArray_int32_T *idx
 *                short x_data[]
 *                int offset
 *                int np
 *                int nq
 *                emxArray_int32_T *iwork
 *                short xwork_data[]
 * Return Type  : void
 */
static void merge(emxArray_int32_T *idx, short x_data[], int offset, int np,
                  int nq, emxArray_int32_T *iwork, short xwork_data[])
{
  int exitg1;
  int iout;
  int j;
  int n_tmp;
  int p;
  int q;
  int *idx_data;
  int *iwork_data;
  iwork_data = iwork->data;
  idx_data = idx->data;
  if (nq != 0) {
    n_tmp = np + nq;
    for (j = 0; j < n_tmp; j++) {
      iout = offset + j;
      iwork_data[j] = idx_data[iout];
      xwork_data[j] = x_data[iout];
    }
    p = 0;
    q = np;
    iout = offset - 1;
    do {
      exitg1 = 0;
      iout++;
      if (xwork_data[p] <= xwork_data[q]) {
        idx_data[iout] = iwork_data[p];
        x_data[iout] = xwork_data[p];
        if (p + 1 < np) {
          p++;
        } else {
          exitg1 = 1;
        }
      } else {
        idx_data[iout] = iwork_data[q];
        x_data[iout] = xwork_data[q];
        if (q + 1 < n_tmp) {
          q++;
        } else {
          q = iout - p;
          for (j = p + 1; j <= np; j++) {
            iout = q + j;
            idx_data[iout] = iwork_data[j - 1];
            x_data[iout] = xwork_data[j - 1];
          }
          exitg1 = 1;
        }
      }
    } while (exitg1 == 0);
  }
}

/*
 * Arguments    : int idx_data[]
 *                double x_data[]
 *                int offset
 *                int np
 *                int nq
 *                int iwork_data[]
 *                double xwork_data[]
 * Return Type  : void
 */
void b_merge(int idx_data[], double x_data[], int offset, int np, int nq,
             int iwork_data[], double xwork_data[])
{
  int exitg1;
  int iout;
  int j;
  int n_tmp;
  int p;
  int q;
  if (nq != 0) {
    n_tmp = np + nq;
    for (j = 0; j < n_tmp; j++) {
      iout = offset + j;
      iwork_data[j] = idx_data[iout];
      xwork_data[j] = x_data[iout];
    }
    p = 0;
    q = np;
    iout = offset - 1;
    do {
      exitg1 = 0;
      iout++;
      if (xwork_data[p] >= xwork_data[q]) {
        idx_data[iout] = iwork_data[p];
        x_data[iout] = xwork_data[p];
        if (p + 1 < np) {
          p++;
        } else {
          exitg1 = 1;
        }
      } else {
        idx_data[iout] = iwork_data[q];
        x_data[iout] = xwork_data[q];
        if (q + 1 < n_tmp) {
          q++;
        } else {
          q = iout - p;
          for (j = p + 1; j <= np; j++) {
            iout = q + j;
            idx_data[iout] = iwork_data[j - 1];
            x_data[iout] = xwork_data[j - 1];
          }
          exitg1 = 1;
        }
      }
    } while (exitg1 == 0);
  }
}

/*
 * Arguments    : emxArray_int32_T *idx
 *                short x_data[]
 *                int offset
 *                int n
 *                int preSortLevel
 *                emxArray_int32_T *iwork
 *                short xwork_data[]
 * Return Type  : void
 */
void merge_block(emxArray_int32_T *idx, short x_data[], int offset, int n,
                 int preSortLevel, emxArray_int32_T *iwork, short xwork_data[])
{
  int bLen;
  int k;
  int nPairs;
  int nTail;
  int tailOffset;
  nPairs = n >> preSortLevel;
  bLen = 1 << preSortLevel;
  while (nPairs > 1) {
    if ((nPairs & 1) != 0) {
      nPairs--;
      tailOffset = bLen * nPairs;
      nTail = n - tailOffset;
      if (nTail > bLen) {
        merge(idx, x_data, offset + tailOffset, bLen, nTail - bLen, iwork,
              xwork_data);
      }
    }
    tailOffset = bLen << 1;
    nPairs >>= 1;
    nTail = (unsigned short)nPairs;
    for (k = 0; k < nTail; k++) {
      merge(idx, x_data, offset + k * tailOffset, bLen, bLen, iwork,
            xwork_data);
    }
    bLen = tailOffset;
  }
  if (n > bLen) {
    merge(idx, x_data, offset, bLen, n - bLen, iwork, xwork_data);
  }
}

/*
 * Arguments    : double x_data[]
 *                const int *x_size
 *                int idx_data[]
 * Return Type  : int
 */
int sortIdx(double x_data[], const int *x_size, int idx_data[])
{
  double xwork_data[34];
  double x4[4];
  double d;
  double d1;
  int iwork_data[34];
  int i;
  int i1;
  int i2;
  int i3;
  int i4;
  int ib;
  int idx_size;
  int k;
  int n;
  int nNaNs;
  int quartetOffset;
  int wOffset_tmp;
  signed char idx4[4];
  signed char perm[4];
  idx_size = *x_size;
  ib = *x_size;
  if (ib - 1 >= 0) {
    memset(&idx_data[0], 0, (unsigned int)ib * sizeof(int));
  }
  if (*x_size != 0) {
    n = *x_size;
    x4[0] = 0.0;
    idx4[0] = 0;
    x4[1] = 0.0;
    idx4[1] = 0;
    x4[2] = 0.0;
    idx4[2] = 0;
    x4[3] = 0.0;
    idx4[3] = 0;
    ib = *x_size;
    if (ib - 1 >= 0) {
      memset(&iwork_data[0], 0, (unsigned int)ib * sizeof(int));
      memset(&xwork_data[0], 0, (unsigned int)ib * sizeof(double));
    }
    nNaNs = 0;
    ib = 0;
    for (k = 0; k < n; k++) {
      if (rtIsNaN(x_data[k])) {
        i4 = (n - nNaNs) - 1;
        idx_data[i4] = k + 1;
        xwork_data[i4] = x_data[k];
        nNaNs++;
      } else {
        ib++;
        idx4[ib - 1] = (signed char)(k + 1);
        x4[ib - 1] = x_data[k];
        if (ib == 4) {
          quartetOffset = k - nNaNs;
          if (x4[0] <= x4[1]) {
            ib = 1;
            i2 = 2;
          } else {
            ib = 2;
            i2 = 1;
          }
          if (x4[2] <= x4[3]) {
            i3 = 3;
            i4 = 4;
          } else {
            i3 = 4;
            i4 = 3;
          }
          d = x4[i3 - 1];
          d1 = x4[ib - 1];
          if (d1 <= d) {
            d1 = x4[i2 - 1];
            if (d1 <= d) {
              i = ib;
              i1 = i2;
              ib = i3;
              i2 = i4;
            } else if (d1 <= x4[i4 - 1]) {
              i = ib;
              i1 = i3;
              ib = i2;
              i2 = i4;
            } else {
              i = ib;
              i1 = i3;
              ib = i4;
            }
          } else {
            d = x4[i4 - 1];
            if (d1 <= d) {
              if (x4[i2 - 1] <= d) {
                i = i3;
                i1 = ib;
                ib = i2;
                i2 = i4;
              } else {
                i = i3;
                i1 = ib;
                ib = i4;
              }
            } else {
              i = i3;
              i1 = i4;
            }
          }
          idx_data[quartetOffset - 3] = idx4[i - 1];
          idx_data[quartetOffset - 2] = idx4[i1 - 1];
          idx_data[quartetOffset - 1] = idx4[ib - 1];
          idx_data[quartetOffset] = idx4[i2 - 1];
          x_data[quartetOffset - 3] = x4[i - 1];
          x_data[quartetOffset - 2] = x4[i1 - 1];
          x_data[quartetOffset - 1] = x4[ib - 1];
          x_data[quartetOffset] = x4[i2 - 1];
          ib = 0;
        }
      }
    }
    wOffset_tmp = *x_size - nNaNs;
    if (ib > 0) {
      perm[1] = 0;
      perm[2] = 0;
      perm[3] = 0;
      if (ib == 1) {
        perm[0] = 1;
      } else if (ib == 2) {
        if (x4[0] <= x4[1]) {
          perm[0] = 1;
          perm[1] = 2;
        } else {
          perm[0] = 2;
          perm[1] = 1;
        }
      } else if (x4[0] <= x4[1]) {
        if (x4[1] <= x4[2]) {
          perm[0] = 1;
          perm[1] = 2;
          perm[2] = 3;
        } else if (x4[0] <= x4[2]) {
          perm[0] = 1;
          perm[1] = 3;
          perm[2] = 2;
        } else {
          perm[0] = 3;
          perm[1] = 1;
          perm[2] = 2;
        }
      } else if (x4[0] <= x4[2]) {
        perm[0] = 2;
        perm[1] = 1;
        perm[2] = 3;
      } else if (x4[1] <= x4[2]) {
        perm[0] = 2;
        perm[1] = 3;
        perm[2] = 1;
      } else {
        perm[0] = 3;
        perm[1] = 2;
        perm[2] = 1;
      }
      i = (unsigned char)ib;
      for (k = 0; k < i; k++) {
        i4 = (wOffset_tmp - ib) + k;
        i1 = perm[k];
        idx_data[i4] = idx4[i1 - 1];
        x_data[i4] = x4[i1 - 1];
      }
    }
    i2 = nNaNs >> 1;
    for (k = 0; k < i2; k++) {
      quartetOffset = wOffset_tmp + k;
      i3 = idx_data[quartetOffset];
      i4 = (n - k) - 1;
      idx_data[quartetOffset] = idx_data[i4];
      idx_data[i4] = i3;
      x_data[quartetOffset] = xwork_data[i4];
      x_data[i4] = xwork_data[quartetOffset];
    }
    if ((nNaNs & 1) != 0) {
      i = wOffset_tmp + i2;
      x_data[i] = xwork_data[i];
    }
    if (wOffset_tmp > 1) {
      i3 = wOffset_tmp >> 2;
      quartetOffset = 4;
      while (i3 > 1) {
        if ((i3 & 1) != 0) {
          i3--;
          ib = quartetOffset * i3;
          i2 = wOffset_tmp - ib;
          if (i2 > quartetOffset) {
            c_merge(idx_data, x_data, ib, quartetOffset, i2 - quartetOffset,
                    iwork_data, xwork_data);
          }
        }
        ib = quartetOffset << 1;
        i3 >>= 1;
        for (k = 0; k < i3; k++) {
          c_merge(idx_data, x_data, k * ib, quartetOffset, quartetOffset,
                  iwork_data, xwork_data);
        }
        quartetOffset = ib;
      }
      if (wOffset_tmp > quartetOffset) {
        c_merge(idx_data, x_data, 0, quartetOffset, wOffset_tmp - quartetOffset,
                iwork_data, xwork_data);
      }
    }
  }
  return idx_size;
}

/*
 * File trailer for sortIdx.c
 *
 * [EOF]
 */
