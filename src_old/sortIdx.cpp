/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: sortIdx.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 17-Mar-2024 11:09:30
 */

/* Include Files */
#include "sortIdx.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_emxutil.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static void b_merge(emxArray_int32_T *idx, short x_data[], int offset, int np,
                    int nq, emxArray_int32_T *iwork, short xwork_data[]);

static void b_merge_block(emxArray_int32_T *idx, short x_data[], int offset,
                          int n, int preSortLevel, emxArray_int32_T *iwork,
                          short xwork_data[]);

static void c_merge(emxArray_int32_T *idx, emxArray_real_T *x, int offset,
                    int np, int nq, emxArray_int32_T *iwork,
                    emxArray_real_T *xwork);

static void c_merge_block(emxArray_int32_T *idx, emxArray_real_T *x, int offset,
                          int n, int preSortLevel, emxArray_int32_T *iwork,
                          emxArray_real_T *xwork);

static void merge(emxArray_int32_T *idx, emxArray_real_T *x, int offset, int np,
                  int nq, emxArray_int32_T *iwork, emxArray_real_T *xwork);

static void merge_block(emxArray_int32_T *idx, emxArray_real_T *x, int offset,
                        int n, int preSortLevel, emxArray_int32_T *iwork,
                        emxArray_real_T *xwork);

static void merge_pow2_block(emxArray_int32_T *idx, emxArray_real_T *x,
                             int offset);

/* Function Definitions */
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
static void b_merge(emxArray_int32_T *idx, short x_data[], int offset, int np,
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
 * Arguments    : emxArray_int32_T *idx
 *                short x_data[]
 *                int offset
 *                int n
 *                int preSortLevel
 *                emxArray_int32_T *iwork
 *                short xwork_data[]
 * Return Type  : void
 */
static void b_merge_block(emxArray_int32_T *idx, short x_data[], int offset,
                          int n, int preSortLevel, emxArray_int32_T *iwork,
                          short xwork_data[])
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
        b_merge(idx, x_data, offset + tailOffset, bLen, nTail - bLen, iwork,
                xwork_data);
      }
    }
    tailOffset = bLen << 1;
    nPairs >>= 1;
    nTail = (unsigned short)nPairs;
    for (k = 0; k < nTail; k++) {
      b_merge(idx, x_data, offset + k * tailOffset, bLen, bLen, iwork,
              xwork_data);
    }
    bLen = tailOffset;
  }
  if (n > bLen) {
    b_merge(idx, x_data, offset, bLen, n - bLen, iwork, xwork_data);
  }
}

/*
 * Arguments    : emxArray_int32_T *idx
 *                emxArray_real_T *x
 *                int offset
 *                int np
 *                int nq
 *                emxArray_int32_T *iwork
 *                emxArray_real_T *xwork
 * Return Type  : void
 */
static void c_merge(emxArray_int32_T *idx, emxArray_real_T *x, int offset,
                    int np, int nq, emxArray_int32_T *iwork,
                    emxArray_real_T *xwork)
{
  double *x_data;
  double *xwork_data;
  int exitg1;
  int iout;
  int j;
  int n_tmp;
  int p;
  int q;
  int *idx_data;
  int *iwork_data;
  xwork_data = xwork->data;
  iwork_data = iwork->data;
  x_data = x->data;
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
 *                emxArray_real_T *x
 *                int offset
 *                int n
 *                int preSortLevel
 *                emxArray_int32_T *iwork
 *                emxArray_real_T *xwork
 * Return Type  : void
 */
static void c_merge_block(emxArray_int32_T *idx, emxArray_real_T *x, int offset,
                          int n, int preSortLevel, emxArray_int32_T *iwork,
                          emxArray_real_T *xwork)
{
  int bLen;
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
        c_merge(idx, x, offset + tailOffset, bLen, nTail - bLen, iwork, xwork);
      }
    }
    tailOffset = bLen << 1;
    nPairs >>= 1;
    for (nTail = 0; nTail < nPairs; nTail++) {
      c_merge(idx, x, offset + nTail * tailOffset, bLen, bLen, iwork, xwork);
    }
    bLen = tailOffset;
  }
  if (n > bLen) {
    c_merge(idx, x, offset, bLen, n - bLen, iwork, xwork);
  }
}

/*
 * Arguments    : emxArray_int32_T *idx
 *                emxArray_real_T *x
 *                int offset
 *                int np
 *                int nq
 *                emxArray_int32_T *iwork
 *                emxArray_real_T *xwork
 * Return Type  : void
 */
static void merge(emxArray_int32_T *idx, emxArray_real_T *x, int offset, int np,
                  int nq, emxArray_int32_T *iwork, emxArray_real_T *xwork)
{
  double *x_data;
  double *xwork_data;
  int exitg1;
  int iout;
  int j;
  int n_tmp;
  int p;
  int q;
  int *idx_data;
  int *iwork_data;
  xwork_data = xwork->data;
  iwork_data = iwork->data;
  x_data = x->data;
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
 * Arguments    : emxArray_int32_T *idx
 *                emxArray_real_T *x
 *                int offset
 *                int n
 *                int preSortLevel
 *                emxArray_int32_T *iwork
 *                emxArray_real_T *xwork
 * Return Type  : void
 */
static void merge_block(emxArray_int32_T *idx, emxArray_real_T *x, int offset,
                        int n, int preSortLevel, emxArray_int32_T *iwork,
                        emxArray_real_T *xwork)
{
  int bLen;
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
        merge(idx, x, offset + tailOffset, bLen, nTail - bLen, iwork, xwork);
      }
    }
    tailOffset = bLen << 1;
    nPairs >>= 1;
    for (nTail = 0; nTail < nPairs; nTail++) {
      merge(idx, x, offset + nTail * tailOffset, bLen, bLen, iwork, xwork);
    }
    bLen = tailOffset;
  }
  if (n > bLen) {
    merge(idx, x, offset, bLen, n - bLen, iwork, xwork);
  }
}

/*
 * Arguments    : emxArray_int32_T *idx
 *                emxArray_real_T *x
 *                int offset
 * Return Type  : void
 */
static void merge_pow2_block(emxArray_int32_T *idx, emxArray_real_T *x,
                             int offset)
{
  double xwork[256];
  double *x_data;
  int iwork[256];
  int b;
  int bLen;
  int bLen2;
  int blockOffset;
  int exitg1;
  int i;
  int iout;
  int j;
  int k;
  int p;
  int q;
  int *idx_data;
  x_data = x->data;
  idx_data = idx->data;
  for (b = 0; b < 6; b++) {
    bLen = 1 << (b + 2);
    bLen2 = bLen << 1;
    i = 256 >> (b + 3);
    for (k = 0; k < i; k++) {
      blockOffset = offset + k * bLen2;
      for (j = 0; j < bLen2; j++) {
        iout = blockOffset + j;
        iwork[j] = idx_data[iout];
        xwork[j] = x_data[iout];
      }
      p = 0;
      q = bLen;
      iout = blockOffset - 1;
      do {
        exitg1 = 0;
        iout++;
        if (xwork[p] <= xwork[q]) {
          idx_data[iout] = iwork[p];
          x_data[iout] = xwork[p];
          if (p + 1 < bLen) {
            p++;
          } else {
            exitg1 = 1;
          }
        } else {
          idx_data[iout] = iwork[q];
          x_data[iout] = xwork[q];
          if (q + 1 < bLen2) {
            q++;
          } else {
            iout -= p;
            for (j = p + 1; j <= bLen; j++) {
              q = iout + j;
              idx_data[q] = iwork[j - 1];
              x_data[q] = xwork[j - 1];
            }
            exitg1 = 1;
          }
        }
      } while (exitg1 == 0);
    }
  }
}

/*
 * Arguments    : short x_data[]
 *                const int *x_size
 *                emxArray_int32_T *idx
 * Return Type  : void
 */
void b_sortIdx(short x_data[], const int *x_size, emxArray_int32_T *idx)
{
  emxArray_int32_T *iwork;
  int b;
  int bLen;
  int bLen2;
  int b_b;
  int b_i;
  int b_i2;
  int b_i3;
  int exitg1;
  int i;
  int i1;
  int i4;
  int j;
  int k;
  int nQuartets;
  int offset;
  int *idx_data;
  int *iwork_data;
  short xwork_data[32767];
  unsigned short b_iwork[256];
  short xwork[256];
  unsigned short idx4[4];
  short x4[4];
  short b_i1;
  short b_i4;
  short i2;
  short i3;
  signed char perm[4];
  i = idx->size[0];
  idx->size[0] = *x_size;
  emxEnsureCapacity_int32_T(idx, i);
  idx_data = idx->data;
  for (i = 0; i < *x_size; i++) {
    idx_data[i] = 0;
  }
  if (*x_size != 0) {
    x4[0] = 0;
    idx4[0] = 0U;
    x4[1] = 0;
    idx4[1] = 0U;
    x4[2] = 0;
    idx4[2] = 0U;
    x4[3] = 0;
    idx4[3] = 0U;
    emxInit_int32_T(&iwork, 1);
    i = iwork->size[0];
    iwork->size[0] = *x_size;
    emxEnsureCapacity_int32_T(iwork, i);
    iwork_data = iwork->data;
    for (i = 0; i < *x_size; i++) {
      iwork_data[i] = 0;
      xwork_data[i] = 0;
    }
    nQuartets = *x_size >> 2;
    for (j = 0; j < nQuartets; j++) {
      b_i = j << 2;
      idx4[0] = (unsigned short)(b_i + 1);
      idx4[1] = (unsigned short)(b_i + 2);
      idx4[2] = (unsigned short)(b_i + 3);
      idx4[3] = (unsigned short)(b_i + 4);
      b_i1 = x_data[b_i];
      x4[0] = b_i1;
      i2 = x_data[b_i + 1];
      x4[1] = i2;
      i3 = x_data[b_i + 2];
      x4[2] = i3;
      b_i4 = x_data[b_i + 3];
      x4[3] = b_i4;
      if (b_i1 <= i2) {
        i1 = 1;
        b_i2 = 2;
      } else {
        i1 = 2;
        b_i2 = 1;
      }
      if (i3 <= b_i4) {
        b_i3 = 3;
        i4 = 4;
      } else {
        b_i3 = 4;
        i4 = 3;
      }
      i = x4[b_i3 - 1];
      offset = x4[i1 - 1];
      if (offset <= i) {
        offset = x4[b_i2 - 1];
        if (offset <= i) {
          i = i1;
          offset = b_i2;
          i1 = b_i3;
          b_i2 = i4;
        } else if (offset <= x4[i4 - 1]) {
          i = i1;
          offset = b_i3;
          i1 = b_i2;
          b_i2 = i4;
        } else {
          i = i1;
          offset = b_i3;
          i1 = i4;
        }
      } else {
        i = x4[i4 - 1];
        if (offset <= i) {
          if (x4[b_i2 - 1] <= i) {
            i = b_i3;
            offset = i1;
            i1 = b_i2;
            b_i2 = i4;
          } else {
            i = b_i3;
            offset = i1;
            i1 = i4;
          }
        } else {
          i = b_i3;
          offset = i4;
        }
      }
      idx_data[b_i] = idx4[i - 1];
      idx_data[b_i + 1] = idx4[offset - 1];
      idx_data[b_i + 2] = idx4[i1 - 1];
      idx_data[b_i + 3] = idx4[b_i2 - 1];
      x_data[b_i] = x4[i - 1];
      x_data[b_i + 1] = x4[offset - 1];
      x_data[b_i + 2] = x4[i1 - 1];
      x_data[b_i + 3] = x4[b_i2 - 1];
    }
    b_i = nQuartets << 2;
    i1 = *x_size - b_i;
    if (i1 > 0) {
      for (k = 0; k < i1; k++) {
        nQuartets = b_i + k;
        idx4[k] = (unsigned short)(nQuartets + 1);
        x4[k] = x_data[nQuartets];
      }
      perm[1] = 0;
      perm[2] = 0;
      perm[3] = 0;
      if (i1 == 1) {
        perm[0] = 1;
      } else if (i1 == 2) {
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
      for (k = 0; k < i1; k++) {
        b_i2 = b_i + k;
        i = perm[k];
        idx_data[b_i2] = idx4[i - 1];
        x_data[b_i2] = x4[i - 1];
      }
    }
    nQuartets = 2;
    if (*x_size > 1) {
      if (*x_size >= 256) {
        i4 = *x_size >> 8;
        for (b = 0; b < i4; b++) {
          offset = (b << 8) - 1;
          for (b_b = 0; b_b < 6; b_b++) {
            bLen = 1 << (b_b + 2);
            bLen2 = bLen << 1;
            i = 256 >> (b_b + 3);
            for (k = 0; k < i; k++) {
              i1 = (offset + k * bLen2) + 1;
              for (j = 0; j < bLen2; j++) {
                nQuartets = i1 + j;
                b_iwork[j] = (unsigned short)idx_data[nQuartets];
                xwork[j] = x_data[nQuartets];
              }
              b_i3 = 0;
              b_i = bLen;
              nQuartets = i1 - 1;
              do {
                exitg1 = 0;
                nQuartets++;
                if (xwork[b_i3] <= xwork[b_i]) {
                  idx_data[nQuartets] = b_iwork[b_i3];
                  x_data[nQuartets] = xwork[b_i3];
                  if (b_i3 + 1 < bLen) {
                    b_i3++;
                  } else {
                    exitg1 = 1;
                  }
                } else {
                  idx_data[nQuartets] = b_iwork[b_i];
                  x_data[nQuartets] = xwork[b_i];
                  if (b_i + 1 < bLen2) {
                    b_i++;
                  } else {
                    nQuartets -= b_i3;
                    for (j = b_i3 + 1; j <= bLen; j++) {
                      b_i2 = nQuartets + j;
                      idx_data[b_i2] = b_iwork[j - 1];
                      x_data[b_i2] = xwork[j - 1];
                    }
                    exitg1 = 1;
                  }
                }
              } while (exitg1 == 0);
            }
          }
        }
        nQuartets = i4 << 8;
        b_i = *x_size - nQuartets;
        if (b_i > 0) {
          b_merge_block(idx, x_data, nQuartets, b_i, 2, iwork, xwork_data);
        }
        nQuartets = 8;
      }
      b_merge_block(idx, x_data, 0, *x_size, nQuartets, iwork, xwork_data);
    }
    emxFree_int32_T(&iwork);
  }
}

/*
 * Arguments    : emxArray_real_T *x
 *                emxArray_int32_T *idx
 * Return Type  : void
 */
void c_sortIdx(emxArray_real_T *x, emxArray_int32_T *idx)
{
  emxArray_int32_T *iwork;
  emxArray_real_T *xwork;
  double b_xwork[256];
  double x4[4];
  double d;
  double d1;
  double *x_data;
  double *xwork_data;
  int b;
  int bLen;
  int b_b;
  int exitg1;
  int i;
  int i1;
  int i2;
  int i3;
  int i4;
  int ib;
  int idx_tmp;
  int k;
  int n;
  int nBlocks;
  int nNaNs;
  int wOffset_tmp;
  int *idx_data;
  int *iwork_data;
  unsigned short b_iwork[256];
  unsigned short idx4[4];
  signed char perm[4];
  x_data = x->data;
  i = idx->size[0];
  idx->size[0] = x->size[0];
  emxEnsureCapacity_int32_T(idx, i);
  idx_data = idx->data;
  ib = x->size[0];
  for (i = 0; i < ib; i++) {
    idx_data[i] = 0;
  }
  if (x->size[0] != 0) {
    n = x->size[0];
    x4[0] = 0.0;
    idx4[0] = 0U;
    x4[1] = 0.0;
    idx4[1] = 0U;
    x4[2] = 0.0;
    idx4[2] = 0U;
    x4[3] = 0.0;
    idx4[3] = 0U;
    emxInit_int32_T(&iwork, 1);
    i = iwork->size[0];
    iwork->size[0] = x->size[0];
    emxEnsureCapacity_int32_T(iwork, i);
    iwork_data = iwork->data;
    ib = x->size[0];
    emxInit_real_T(&xwork, 1);
    i = xwork->size[0];
    xwork->size[0] = x->size[0];
    emxEnsureCapacity_real_T(xwork, i);
    xwork_data = xwork->data;
    for (i = 0; i < ib; i++) {
      iwork_data[i] = 0;
      xwork_data[i] = 0.0;
    }
    nNaNs = 0;
    ib = 0;
    for (k = 0; k < n; k++) {
      if (rtIsNaN(x_data[k])) {
        idx_tmp = (n - nNaNs) - 1;
        idx_data[idx_tmp] = k + 1;
        xwork_data[idx_tmp] = x_data[k];
        nNaNs++;
      } else {
        ib++;
        idx4[ib - 1] = (unsigned short)(k + 1);
        x4[ib - 1] = x_data[k];
        if (ib == 4) {
          ib = k - nNaNs;
          if (x4[0] >= x4[1]) {
            i1 = 1;
            i2 = 2;
          } else {
            i1 = 2;
            i2 = 1;
          }
          if (x4[2] >= x4[3]) {
            i3 = 3;
            i4 = 4;
          } else {
            i3 = 4;
            i4 = 3;
          }
          d = x4[i3 - 1];
          d1 = x4[i1 - 1];
          if (d1 >= d) {
            d1 = x4[i2 - 1];
            if (d1 >= d) {
              i = i1;
              bLen = i2;
              i1 = i3;
              i2 = i4;
            } else if (d1 >= x4[i4 - 1]) {
              i = i1;
              bLen = i3;
              i1 = i2;
              i2 = i4;
            } else {
              i = i1;
              bLen = i3;
              i1 = i4;
            }
          } else {
            d = x4[i4 - 1];
            if (d1 >= d) {
              if (x4[i2 - 1] >= d) {
                i = i3;
                bLen = i1;
                i1 = i2;
                i2 = i4;
              } else {
                i = i3;
                bLen = i1;
                i1 = i4;
              }
            } else {
              i = i3;
              bLen = i4;
            }
          }
          idx_data[ib - 3] = idx4[i - 1];
          idx_data[ib - 2] = idx4[bLen - 1];
          idx_data[ib - 1] = idx4[i1 - 1];
          idx_data[ib] = idx4[i2 - 1];
          x_data[ib - 3] = x4[i - 1];
          x_data[ib - 2] = x4[bLen - 1];
          x_data[ib - 1] = x4[i1 - 1];
          x_data[ib] = x4[i2 - 1];
          ib = 0;
        }
      }
    }
    wOffset_tmp = x->size[0] - nNaNs;
    if (ib > 0) {
      perm[1] = 0;
      perm[2] = 0;
      perm[3] = 0;
      if (ib == 1) {
        perm[0] = 1;
      } else if (ib == 2) {
        if (x4[0] >= x4[1]) {
          perm[0] = 1;
          perm[1] = 2;
        } else {
          perm[0] = 2;
          perm[1] = 1;
        }
      } else if (x4[0] >= x4[1]) {
        if (x4[1] >= x4[2]) {
          perm[0] = 1;
          perm[1] = 2;
          perm[2] = 3;
        } else if (x4[0] >= x4[2]) {
          perm[0] = 1;
          perm[1] = 3;
          perm[2] = 2;
        } else {
          perm[0] = 3;
          perm[1] = 1;
          perm[2] = 2;
        }
      } else if (x4[0] >= x4[2]) {
        perm[0] = 2;
        perm[1] = 1;
        perm[2] = 3;
      } else if (x4[1] >= x4[2]) {
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
        idx_tmp = (wOffset_tmp - ib) + k;
        bLen = perm[k];
        idx_data[idx_tmp] = idx4[bLen - 1];
        x_data[idx_tmp] = x4[bLen - 1];
      }
    }
    i1 = nNaNs >> 1;
    for (k = 0; k < i1; k++) {
      ib = wOffset_tmp + k;
      i2 = idx_data[ib];
      idx_tmp = (n - k) - 1;
      idx_data[ib] = idx_data[idx_tmp];
      idx_data[idx_tmp] = i2;
      x_data[ib] = xwork_data[idx_tmp];
      x_data[idx_tmp] = xwork_data[ib];
    }
    if ((nNaNs & 1) != 0) {
      i = wOffset_tmp + i1;
      x_data[i] = xwork_data[i];
    }
    ib = 2;
    if (wOffset_tmp > 1) {
      if (x->size[0] >= 256) {
        nBlocks = wOffset_tmp >> 8;
        if (nBlocks > 0) {
          for (b = 0; b < nBlocks; b++) {
            i4 = (b << 8) - 1;
            for (b_b = 0; b_b < 6; b_b++) {
              bLen = 1 << (b_b + 2);
              n = bLen << 1;
              i = 256 >> (b_b + 3);
              for (k = 0; k < i; k++) {
                i2 = (i4 + k * n) + 1;
                for (i1 = 0; i1 < n; i1++) {
                  ib = i2 + i1;
                  b_iwork[i1] = (unsigned short)idx_data[ib];
                  b_xwork[i1] = x_data[ib];
                }
                i3 = 0;
                i1 = bLen;
                ib = i2 - 1;
                do {
                  exitg1 = 0;
                  ib++;
                  if (b_xwork[i3] >= b_xwork[i1]) {
                    idx_data[ib] = b_iwork[i3];
                    x_data[ib] = b_xwork[i3];
                    if (i3 + 1 < bLen) {
                      i3++;
                    } else {
                      exitg1 = 1;
                    }
                  } else {
                    idx_data[ib] = b_iwork[i1];
                    x_data[ib] = b_xwork[i1];
                    if (i1 + 1 < n) {
                      i1++;
                    } else {
                      ib -= i3;
                      for (i1 = i3 + 1; i1 <= bLen; i1++) {
                        idx_tmp = ib + i1;
                        idx_data[idx_tmp] = b_iwork[i1 - 1];
                        x_data[idx_tmp] = b_xwork[i1 - 1];
                      }
                      exitg1 = 1;
                    }
                  }
                } while (exitg1 == 0);
              }
            }
          }
          ib = nBlocks << 8;
          i1 = wOffset_tmp - ib;
          if (i1 > 0) {
            c_merge_block(idx, x, ib, i1, 2, iwork, xwork);
          }
          ib = 8;
        }
      }
      c_merge_block(idx, x, 0, wOffset_tmp, ib, iwork, xwork);
      xwork_data = xwork->data;
      iwork_data = iwork->data;
      x_data = x->data;
      idx_data = idx->data;
    }
    if ((nNaNs > 0) && (wOffset_tmp > 0)) {
      for (k = 0; k < nNaNs; k++) {
        ib = wOffset_tmp + k;
        xwork_data[k] = x_data[ib];
        iwork_data[k] = idx_data[ib];
      }
      for (k = wOffset_tmp; k >= 1; k--) {
        i = (nNaNs + k) - 1;
        x_data[i] = x_data[k - 1];
        idx_data[i] = idx_data[k - 1];
      }
      for (k = 0; k < nNaNs; k++) {
        x_data[k] = xwork_data[k];
        idx_data[k] = iwork_data[k];
      }
    }
    emxFree_real_T(&xwork);
    emxFree_int32_T(&iwork);
  }
}

/*
 * Arguments    : emxArray_real_T *x
 *                emxArray_int32_T *idx
 * Return Type  : void
 */
void sortIdx(emxArray_real_T *x, emxArray_int32_T *idx)
{
  emxArray_int32_T *iwork;
  emxArray_real_T *xwork;
  double x4[4];
  double d;
  double d1;
  double *x_data;
  double *xwork_data;
  int idx4[4];
  int b_i1;
  int i;
  int i1;
  int i2;
  int i3;
  int i4;
  int ib;
  int k;
  int n;
  int nNaNs;
  int *idx_data;
  int *iwork_data;
  signed char perm[4];
  x_data = x->data;
  i = idx->size[0];
  idx->size[0] = x->size[0];
  emxEnsureCapacity_int32_T(idx, i);
  idx_data = idx->data;
  ib = x->size[0];
  for (i = 0; i < ib; i++) {
    idx_data[i] = 0;
  }
  if (x->size[0] != 0) {
    n = x->size[0];
    x4[0] = 0.0;
    idx4[0] = 0;
    x4[1] = 0.0;
    idx4[1] = 0;
    x4[2] = 0.0;
    idx4[2] = 0;
    x4[3] = 0.0;
    idx4[3] = 0;
    emxInit_int32_T(&iwork, 1);
    i = iwork->size[0];
    iwork->size[0] = x->size[0];
    emxEnsureCapacity_int32_T(iwork, i);
    iwork_data = iwork->data;
    ib = x->size[0];
    emxInit_real_T(&xwork, 1);
    i = xwork->size[0];
    xwork->size[0] = x->size[0];
    emxEnsureCapacity_real_T(xwork, i);
    xwork_data = xwork->data;
    for (i = 0; i < ib; i++) {
      iwork_data[i] = 0;
      xwork_data[i] = 0.0;
    }
    nNaNs = 0;
    ib = 0;
    for (k = 0; k < n; k++) {
      if (rtIsNaN(x_data[k])) {
        i3 = (n - nNaNs) - 1;
        idx_data[i3] = k + 1;
        xwork_data[i3] = x_data[k];
        nNaNs++;
      } else {
        ib++;
        idx4[ib - 1] = k + 1;
        x4[ib - 1] = x_data[k];
        if (ib == 4) {
          ib = k - nNaNs;
          if (x4[0] <= x4[1]) {
            i1 = 1;
            i2 = 2;
          } else {
            i1 = 2;
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
          d1 = x4[i1 - 1];
          if (d1 <= d) {
            d1 = x4[i2 - 1];
            if (d1 <= d) {
              i = i1;
              b_i1 = i2;
              i1 = i3;
              i2 = i4;
            } else if (d1 <= x4[i4 - 1]) {
              i = i1;
              b_i1 = i3;
              i1 = i2;
              i2 = i4;
            } else {
              i = i1;
              b_i1 = i3;
              i1 = i4;
            }
          } else {
            d = x4[i4 - 1];
            if (d1 <= d) {
              if (x4[i2 - 1] <= d) {
                i = i3;
                b_i1 = i1;
                i1 = i2;
                i2 = i4;
              } else {
                i = i3;
                b_i1 = i1;
                i1 = i4;
              }
            } else {
              i = i3;
              b_i1 = i4;
            }
          }
          idx_data[ib - 3] = idx4[i - 1];
          idx_data[ib - 2] = idx4[b_i1 - 1];
          idx_data[ib - 1] = idx4[i1 - 1];
          idx_data[ib] = idx4[i2 - 1];
          x_data[ib - 3] = x4[i - 1];
          x_data[ib - 2] = x4[b_i1 - 1];
          x_data[ib - 1] = x4[i1 - 1];
          x_data[ib] = x4[i2 - 1];
          ib = 0;
        }
      }
    }
    i4 = x->size[0] - nNaNs;
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
        i3 = (i4 - ib) + k;
        b_i1 = perm[k];
        idx_data[i3] = idx4[b_i1 - 1];
        x_data[i3] = x4[b_i1 - 1];
      }
    }
    i1 = nNaNs >> 1;
    for (k = 0; k < i1; k++) {
      ib = i4 + k;
      i2 = idx_data[ib];
      i3 = (n - k) - 1;
      idx_data[ib] = idx_data[i3];
      idx_data[i3] = i2;
      x_data[ib] = xwork_data[i3];
      x_data[i3] = xwork_data[ib];
    }
    if ((nNaNs & 1) != 0) {
      i = i4 + i1;
      x_data[i] = xwork_data[i];
    }
    ib = 2;
    if (i4 > 1) {
      if (x->size[0] >= 256) {
        i1 = i4 >> 8;
        if (i1 > 0) {
          for (ib = 0; ib < i1; ib++) {
            merge_pow2_block(idx, x, ib << 8);
          }
          ib = i1 << 8;
          i1 = i4 - ib;
          if (i1 > 0) {
            merge_block(idx, x, ib, i1, 2, iwork, xwork);
          }
          ib = 8;
        }
      }
      merge_block(idx, x, 0, i4, ib, iwork, xwork);
    }
    emxFree_real_T(&xwork);
    emxFree_int32_T(&iwork);
  }
}

/*
 * File trailer for sortIdx.c
 *
 * [EOF]
 */
