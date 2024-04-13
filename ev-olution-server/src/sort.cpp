/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: sort.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Apr-2024 17:40:22
 */

/* Include Files */
#include "sort.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_emxutil.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h"
#include "rt_nonfinite.h"
#include "sortIdx.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Type Definitions */
#ifndef struct_emxArray_int32_T_34
#define struct_emxArray_int32_T_34
struct emxArray_int32_T_34 {
  int data[34];
};
#endif /* struct_emxArray_int32_T_34 */
#ifndef typedef_emxArray_int32_T_34
#define typedef_emxArray_int32_T_34
typedef struct emxArray_int32_T_34 emxArray_int32_T_34;
#endif /* typedef_emxArray_int32_T_34 */

/* Function Definitions */
/*
 * Arguments    : short x_data[]
 *                const int *x_size
 * Return Type  : void
 */
void b_sort(short x_data[], const int *x_size)
{
  /*static */short b_vwork_data[32767];
  /*static */short vwork_data[32767];
  emxArray_int32_T *iidx;
  emxArray_int32_T *iwork;
  int b;
  int bLen;
  int bLen2;
  int b_b;
  int b_i;
  int b_j;
  int c_i;
  int dim;
  int exitg1;
  int i;
  int i1;
  int i2;
  int i3;
  int i4;
  int j;
  int k;
  int offset;
  int vlen;
  int vstride;
  int vwork_size;
  int *iidx_data;
  int *iwork_data;
  unsigned short b_iwork[256];
  short xwork[256];
  unsigned short idx4[4];
  short x4[4];
  short b_i1;
  short b_x4_tmp;
  short c_x4_tmp;
  short x4_tmp;
  signed char perm[4];
  dim = 0;
  if (*x_size != 1) {
    dim = -1;
  }
  if (dim + 2 <= 1) {
    vwork_size = *x_size;
  } else {
    vwork_size = 1;
  }
  vlen = vwork_size - 1;
  vstride = 1;
  for (k = 0; k <= dim; k++) {
    vstride *= *x_size;
  }
  emxInit_int32_T(&iidx);
  emxInit_int32_T(&iwork);
  for (i = 0; i < 1; i++) {
    for (j = 0; j < vstride; j++) {
      for (k = 0; k <= vlen; k++) {
        vwork_data[k] = x_data[j + k * vstride];
      }
      b_i = iidx->size[0];
      iidx->size[0] = vwork_size;
      emxEnsureCapacity_int32_T(iidx, b_i);
      iidx_data = iidx->data;
      for (b_i = 0; b_i < vwork_size; b_i++) {
        b_vwork_data[b_i] = vwork_data[b_i];
        iidx_data[b_i] = 0;
      }
      if (vwork_size != 0) {
        x4[0] = 0;
        idx4[0] = 0U;
        x4[1] = 0;
        idx4[1] = 0U;
        x4[2] = 0;
        idx4[2] = 0U;
        x4[3] = 0;
        idx4[3] = 0U;
        b_i = iwork->size[0];
        iwork->size[0] = vwork_size;
        emxEnsureCapacity_int32_T(iwork, b_i);
        iwork_data = iwork->data;
        for (b_i = 0; b_i < vwork_size; b_i++) {
          iwork_data[b_i] = 0;
          vwork_data[b_i] = 0;
        }
        dim = vwork_size >> 2;
        for (b_j = 0; b_j < dim; b_j++) {
          c_i = b_j << 2;
          idx4[0] = (unsigned short)(c_i + 1);
          idx4[1] = (unsigned short)(c_i + 2);
          idx4[2] = (unsigned short)(c_i + 3);
          idx4[3] = (unsigned short)(c_i + 4);
          b_i1 = b_vwork_data[c_i];
          x4[0] = b_i1;
          x4_tmp = b_vwork_data[c_i + 1];
          x4[1] = x4_tmp;
          b_x4_tmp = b_vwork_data[c_i + 2];
          x4[2] = b_x4_tmp;
          c_x4_tmp = b_vwork_data[c_i + 3];
          x4[3] = c_x4_tmp;
          if (b_i1 <= x4_tmp) {
            i1 = 1;
            i2 = 2;
          } else {
            i1 = 2;
            i2 = 1;
          }
          if (b_x4_tmp <= c_x4_tmp) {
            i3 = 3;
            i4 = 4;
          } else {
            i3 = 4;
            i4 = 3;
          }
          b_i = x4[i3 - 1];
          offset = x4[i1 - 1];
          if (offset <= b_i) {
            offset = x4[i2 - 1];
            if (offset <= b_i) {
              b_i = i1;
              offset = i2;
              i1 = i3;
              i2 = i4;
            } else if (offset <= x4[i4 - 1]) {
              b_i = i1;
              offset = i3;
              i1 = i2;
              i2 = i4;
            } else {
              b_i = i1;
              offset = i3;
              i1 = i4;
            }
          } else {
            b_i = x4[i4 - 1];
            if (offset <= b_i) {
              if (x4[i2 - 1] <= b_i) {
                b_i = i3;
                offset = i1;
                i1 = i2;
                i2 = i4;
              } else {
                b_i = i3;
                offset = i1;
                i1 = i4;
              }
            } else {
              b_i = i3;
              offset = i4;
            }
          }
          iidx_data[c_i] = idx4[b_i - 1];
          iidx_data[c_i + 1] = idx4[offset - 1];
          iidx_data[c_i + 2] = idx4[i1 - 1];
          iidx_data[c_i + 3] = idx4[i2 - 1];
          b_vwork_data[c_i] = x4[b_i - 1];
          b_vwork_data[c_i + 1] = x4[offset - 1];
          b_vwork_data[c_i + 2] = x4[i1 - 1];
          b_vwork_data[c_i + 3] = x4[i2 - 1];
        }
        c_i = dim << 2;
        i1 = vwork_size - c_i;
        if (i1 > 0) {
          for (k = 0; k < i1; k++) {
            dim = c_i + k;
            idx4[k] = (unsigned short)(dim + 1);
            x4[k] = b_vwork_data[dim];
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
            i2 = c_i + k;
            b_i = perm[k];
            iidx_data[i2] = idx4[b_i - 1];
            b_vwork_data[i2] = x4[b_i - 1];
          }
        }
        dim = 2;
        if (vwork_size > 1) {
          if (vwork_size >= 256) {
            i4 = vwork_size >> 8;
            for (b = 0; b < i4; b++) {
              offset = (b << 8) - 1;
              for (b_b = 0; b_b < 6; b_b++) {
                bLen = 1 << (b_b + 2);
                bLen2 = bLen << 1;
                b_i = 256 >> (b_b + 3);
                for (k = 0; k < b_i; k++) {
                  i1 = (offset + k * bLen2) + 1;
                  for (b_j = 0; b_j < bLen2; b_j++) {
                    dim = i1 + b_j;
                    b_iwork[b_j] = (unsigned short)iidx_data[dim];
                    xwork[b_j] = b_vwork_data[dim];
                  }
                  i3 = 0;
                  c_i = bLen;
                  dim = i1 - 1;
                  do {
                    exitg1 = 0;
                    dim++;
                    if (xwork[i3] <= xwork[c_i]) {
                      iidx_data[dim] = b_iwork[i3];
                      b_vwork_data[dim] = xwork[i3];
                      if (i3 + 1 < bLen) {
                        i3++;
                      } else {
                        exitg1 = 1;
                      }
                    } else {
                      iidx_data[dim] = b_iwork[c_i];
                      b_vwork_data[dim] = xwork[c_i];
                      if (c_i + 1 < bLen2) {
                        c_i++;
                      } else {
                        dim -= i3;
                        for (b_j = i3 + 1; b_j <= bLen; b_j++) {
                          i2 = dim + b_j;
                          iidx_data[i2] = b_iwork[b_j - 1];
                          b_vwork_data[i2] = xwork[b_j - 1];
                        }
                        exitg1 = 1;
                      }
                    }
                  } while (exitg1 == 0);
                }
              }
            }
            dim = i4 << 8;
            c_i = vwork_size - dim;
            if (c_i > 0) {
              merge_block(iidx, b_vwork_data, dim, c_i, 2, iwork, vwork_data);
            }
            dim = 8;
          }
          merge_block(iidx, b_vwork_data, 0, vwork_size, dim, iwork,
                      vwork_data);
        }
      }
      if (vwork_size - 1 >= 0) {
        memcpy(&vwork_data[0], &b_vwork_data[0],
               (unsigned int)vwork_size * sizeof(short));
      }
      for (k = 0; k <= vlen; k++) {
        x_data[j + k * vstride] = b_vwork_data[k];
      }
    }
  }
  emxFree_int32_T(&iwork);
  emxFree_int32_T(&iidx);
}

/*
 * Arguments    : double x_data[]
 *                const int *x_size
 *                int idx_data[]
 * Return Type  : int
 */
int c_sort(double x_data[], const int *x_size, int idx_data[])
{
  double vwork_data[32];
  double xwork_data[32];
  double x4[4];
  double d;
  double d1;
  int iidx_data[32];
  int iwork_data[32];
  int b_i;
  int b_i1;
  int dim;
  int i;
  int i1;
  int i2;
  int i3;
  int i4;
  int idx_size;
  int j;
  int k;
  int nNaNs;
  int vlen;
  int vstride;
  int vwork_size;
  signed char idx4[4];
  signed char perm[4];
  dim = 0;
  if (*x_size != 1) {
    dim = -1;
  }
  if (dim + 2 <= 1) {
    vwork_size = *x_size;
  } else {
    vwork_size = 1;
  }
  vlen = vwork_size - 1;
  idx_size = *x_size;
  vstride = 1;
  for (k = 0; k <= dim; k++) {
    vstride *= *x_size;
  }
  for (i = 0; i < 1; i++) {
    for (j = 0; j < vstride; j++) {
      for (k = 0; k <= vlen; k++) {
        vwork_data[k] = x_data[j + k * vstride];
      }
      if (vwork_size - 1 >= 0) {
        memset(&iidx_data[0], 0, (unsigned int)vwork_size * sizeof(int));
      }
      if (vwork_size != 0) {
        x4[0] = 0.0;
        idx4[0] = 0;
        x4[1] = 0.0;
        idx4[1] = 0;
        x4[2] = 0.0;
        idx4[2] = 0;
        x4[3] = 0.0;
        idx4[3] = 0;
        memset(&iwork_data[0], 0, (unsigned int)vwork_size * sizeof(int));
        memset(&xwork_data[0], 0, (unsigned int)vwork_size * sizeof(double));
        nNaNs = 0;
        dim = 0;
        for (k = 0; k < vwork_size; k++) {
          if (rtIsNaN(vwork_data[k])) {
            i3 = (vwork_size - nNaNs) - 1;
            iidx_data[i3] = k + 1;
            xwork_data[i3] = vwork_data[k];
            nNaNs++;
          } else {
            dim++;
            idx4[dim - 1] = (signed char)(k + 1);
            x4[dim - 1] = vwork_data[k];
            if (dim == 4) {
              dim = k - nNaNs;
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
                  b_i = i1;
                  b_i1 = i2;
                  i1 = i3;
                  i2 = i4;
                } else if (d1 >= x4[i4 - 1]) {
                  b_i = i1;
                  b_i1 = i3;
                  i1 = i2;
                  i2 = i4;
                } else {
                  b_i = i1;
                  b_i1 = i3;
                  i1 = i4;
                }
              } else {
                d = x4[i4 - 1];
                if (d1 >= d) {
                  if (x4[i2 - 1] >= d) {
                    b_i = i3;
                    b_i1 = i1;
                    i1 = i2;
                    i2 = i4;
                  } else {
                    b_i = i3;
                    b_i1 = i1;
                    i1 = i4;
                  }
                } else {
                  b_i = i3;
                  b_i1 = i4;
                }
              }
              iidx_data[dim - 3] = idx4[b_i - 1];
              iidx_data[dim - 2] = idx4[b_i1 - 1];
              iidx_data[dim - 1] = idx4[i1 - 1];
              iidx_data[dim] = idx4[i2 - 1];
              vwork_data[dim - 3] = x4[b_i - 1];
              vwork_data[dim - 2] = x4[b_i1 - 1];
              vwork_data[dim - 1] = x4[i1 - 1];
              vwork_data[dim] = x4[i2 - 1];
              dim = 0;
            }
          }
        }
        i4 = vwork_size - nNaNs;
        if (dim > 0) {
          perm[1] = 0;
          perm[2] = 0;
          perm[3] = 0;
          if (dim == 1) {
            perm[0] = 1;
          } else if (dim == 2) {
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
          b_i = (unsigned char)dim;
          for (k = 0; k < b_i; k++) {
            i3 = (i4 - dim) + k;
            b_i1 = perm[k];
            iidx_data[i3] = idx4[b_i1 - 1];
            vwork_data[i3] = x4[b_i1 - 1];
          }
        }
        dim = nNaNs >> 1;
        for (k = 0; k < dim; k++) {
          i1 = i4 + k;
          i2 = iidx_data[i1];
          i3 = (vwork_size - k) - 1;
          iidx_data[i1] = iidx_data[i3];
          iidx_data[i3] = i2;
          vwork_data[i1] = xwork_data[i3];
          vwork_data[i3] = xwork_data[i1];
        }
        if ((nNaNs & 1) != 0) {
          dim += i4;
          vwork_data[dim] = xwork_data[dim];
        }
        if (i4 > 1) {
          i3 = i4 >> 2;
          i2 = 4;
          while (i3 > 1) {
            if ((i3 & 1) != 0) {
              i3--;
              dim = i2 * i3;
              i1 = i4 - dim;
              if (i1 > i2) {
                b_merge(iidx_data, vwork_data, dim, i2, i1 - i2, iwork_data,
                        xwork_data);
              }
            }
            dim = i2 << 1;
            i3 >>= 1;
            for (k = 0; k < i3; k++) {
              b_merge(iidx_data, vwork_data, k * dim, i2, i2, iwork_data,
                      xwork_data);
            }
            i2 = dim;
          }
          if (i4 > i2) {
            b_merge(iidx_data, vwork_data, 0, i2, i4 - i2, iwork_data,
                    xwork_data);
          }
        }
        if ((nNaNs > 0) && (i4 > 0)) {
          for (k = 0; k < nNaNs; k++) {
            dim = i4 + k;
            xwork_data[k] = vwork_data[dim];
            iwork_data[k] = iidx_data[dim];
          }
          for (k = i4; k >= 1; k--) {
            dim = (nNaNs + k) - 1;
            vwork_data[dim] = vwork_data[k - 1];
            iidx_data[dim] = iidx_data[k - 1];
          }
          memcpy(&vwork_data[0], &xwork_data[0],
                 (unsigned int)nNaNs * sizeof(double));
          memcpy(&iidx_data[0], &iwork_data[0],
                 (unsigned int)nNaNs * sizeof(int));
        }
      }
      for (k = 0; k <= vlen; k++) {
        b_i = j + k * vstride;
        x_data[b_i] = vwork_data[k];
        idx_data[b_i] = iidx_data[k];
      }
    }
  }
  return idx_size;
}

/*
 * Arguments    : double x_data[]
 *                const int *x_size
 *                int idx_data[]
 * Return Type  : int
 */
int d_sort(double x_data[], const int *x_size, int idx_data[])
{
  double vwork_data[34];
  int iidx_data[34];
  int dim;
  int idx_size;
  int j;
  int k;
  int vlen;
  int vstride;
  int vwork_size;
  dim = 0;
  if (*x_size != 1) {
    dim = -1;
  }
  if (dim + 2 <= 1) {
    vwork_size = *x_size;
  } else {
    vwork_size = 1;
  }
  vlen = vwork_size - 1;
  idx_size = *x_size;
  vstride = 1;
  for (k = 0; k <= dim; k++) {
    vstride *= *x_size;
  }
  for (j = 0; j < vstride; j++) {
    for (k = 0; k <= vlen; k++) {
      vwork_data[k] = x_data[j + k * vstride];
    }
    sortIdx(vwork_data, &vwork_size, iidx_data);
    for (k = 0; k <= vlen; k++) {
      dim = j + k * vstride;
      x_data[dim] = vwork_data[k];
      idx_data[dim] = iidx_data[k];
    }
  }
  return idx_size;
}

/*
 * Arguments    : double x_data[]
 *                const int *x_size
 * Return Type  : void
 */
void e_sort(double x_data[], const int *x_size)
{
  emxArray_int32_T_34 b_vwork_data;
  double vwork_data[34];
  int dim;
  int k;
  int vlen;
  int vstride;
  int vwork_size;
  dim = 0;
  if (*x_size != 1) {
    dim = -1;
  }
  if (dim + 2 <= 1) {
    vwork_size = *x_size;
  } else {
    vwork_size = 1;
  }
  vlen = vwork_size - 1;
  vstride = 1;
  for (k = 0; k <= dim; k++) {
    vstride *= *x_size;
  }
  for (dim = 0; dim < vstride; dim++) {
    for (k = 0; k <= vlen; k++) {
      vwork_data[k] = x_data[dim + k * vstride];
    }
    sortIdx(vwork_data, &vwork_size, b_vwork_data.data);
    for (k = 0; k <= vlen; k++) {
      x_data[dim + k * vstride] = vwork_data[k];
    }
  }
}

/*
 * Arguments    : short x[2]
 * Return Type  : void
 */
void sort(short x[2])
{
  short tmp;
  if (x[0] > x[1]) {
    tmp = x[0];
    x[0] = x[1];
    x[1] = tmp;
  }
}

/*
 * File trailer for sort.c
 *
 * [EOF]
 */
