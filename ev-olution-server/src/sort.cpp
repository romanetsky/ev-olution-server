/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: sort.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 17-Apr-2024 18:20:59
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
  /*static*/ short vwork_data[32767];
  emxArray_int32_T *b_vwork_data;
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
  emxInit_int32_T(&b_vwork_data);
  for (dim = 0; dim < vstride; dim++) {
    for (k = 0; k <= vlen; k++) {
      vwork_data[k] = x_data[dim + k * vstride];
    }
    sortIdx(vwork_data, &vwork_size, b_vwork_data);
    for (k = 0; k <= vlen; k++) {
      x_data[dim + k * vstride] = vwork_data[k];
    }
  }
  emxFree_int32_T(&b_vwork_data);
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
    b_sortIdx(vwork_data, &vwork_size, iidx_data);
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
    b_sortIdx(vwork_data, &vwork_size, b_vwork_data.data);
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
