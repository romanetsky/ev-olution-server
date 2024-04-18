/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: findSwitchMatEsp.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 31-Mar-2024 07:04:52
 */

/* Include Files */
#include "findSwitchMatEsp.h"
#include "rt_nonfinite.h"
#include "unique.h"
#include <string.h>

/* Function Definitions */
/*
 * Switch = uint8(Switchin);
 *
 * Arguments    : const unsigned char SwitchMat_esp[4112]
 *                const unsigned char SwitchMat_esp2[24]
 *                const unsigned char Switch[256]
 *                double SwitchId[2]
 *                double switchId2_data[]
 *                int switchId2_size[2]
 * Return Type  : void
 */
void findSwitchMatEsp(const unsigned char SwitchMat_esp[4112],
                      const unsigned char SwitchMat_esp2[24],
                      const unsigned char Switch[256], double SwitchId[2],
                      double switchId2_data[], int switchId2_size[2])
{
  double tmp_data[256];
  double uindNbatRem_data[256];
  double b_uindNbatRem_data[4];
  double indNbat2[4];
  int i_data[256];
  int tmp_size[2];
  int uindNbatRem_size[2];
  int i;
  int idx;
  int ii;
  int jj;
  int k_bat2;
  int last;
  int minCol;
  int minRow;
  short ii_data[257];
  signed char indNbat[256];
  signed char j_data[256];
  unsigned char Switch0[128];
  signed char d_ii_data[6];
  signed char b_ii_data;
  signed char c_ii_data;
  unsigned char u;
  boolean_T IsEqualFlag[257];
  boolean_T bv[256];
  boolean_T bv1[256];
  boolean_T x[16];
  boolean_T b_IsEqualFlag[6];
  boolean_T exitg1;
  boolean_T guard1;
  boolean_T p;
  memset(&indNbat[0], 0, 256U * sizeof(signed char));
  memset(&Switch0[0], 0, 128U * sizeof(unsigned char));
  /*  switchId0 = zeros(1,1); */
  for (k_bat2 = 0; k_bat2 < 2; k_bat2++) {
    idx = (k_bat2 + 1) << 3;
    ii = k_bat2 << 3;
    for (i = 0; i < 256; i++) {
      u = Switch[i];
      bv[i] = (idx >= u);
      bv1[i] = (u >= ii + 1);
    }
    idx = 0;
    ii = 0;
    jj = 0;
    exitg1 = false;
    while ((!exitg1) && (jj + 1 <= 16)) {
      i = ii + (jj << 4);
      guard1 = false;
      if (bv[i] && bv1[i]) {
        idx++;
        i_data[idx - 1] = ii + 1;
        j_data[idx - 1] = (signed char)(jj + 1);
        if (idx >= 256) {
          exitg1 = true;
        } else {
          guard1 = true;
        }
      } else {
        guard1 = true;
      }
      if (guard1) {
        ii++;
        if (ii + 1 > 16) {
          ii = 0;
          jj++;
        }
      }
    }
    if (idx < 1) {
      last = 0;
    } else {
      last = idx;
    }
    if (last > 0) {
      if (last <= 2) {
        if (last == 1) {
          minRow = i_data[0];
        } else if (i_data[0] > i_data[1]) {
          minRow = i_data[1];
        } else {
          minRow = i_data[0];
        }
        if (last == 1) {
          minCol = j_data[0];
        } else if (j_data[0] > j_data[1]) {
          minCol = j_data[1];
        } else {
          minCol = j_data[0];
        }
      } else {
        minRow = i_data[0];
        for (ii = 2; ii <= last; ii++) {
          i = i_data[ii - 1];
          if (minRow > i) {
            minRow = i;
          }
        }
        minCol = j_data[0];
        for (ii = 2; ii <= last; ii++) {
          i = j_data[ii - 1];
          if (minCol > i) {
            minCol = i;
          }
        }
      }
    } else {
      minRow = 0;
      minCol = 0;
    }
    for (idx = 0; idx < last; idx++) {
      for (ii = 0; ii < last; ii++) {
        i = i_data[idx];
        b_ii_data = j_data[ii];
        jj = (i + ((b_ii_data - 1) << 4)) - 1;
        Switch0[(((int)((double)i - ((double)minRow - 1.0)) +
                  ((b_ii_data - minCol) << 3)) +
                 (k_bat2 << 6)) -
                1] = Switch[jj];
        indNbat[jj] = (signed char)(k_bat2 + 1);
      }
    }
    /*  switchId0(1,1) =
     * findSwitchMat0(reshape(SwitchMat_esp(:,k_bat2,:,:),Nswitch1,Nbat1,Nbat1Col),reshape(Switch0(:,:,k_bat2),Nbat1,Nbat1));
     */
    /*  % if ~isempty(switchId0)|switchId0==0 */
    /*  %     SwitchId(k_bat2) = switchId0; */
    /*  % else */
    /*  %     SwitchId(k_bat2) = 0 ; */
    /*  % end */
    /*  SwitchId(k_bat2) = switchId0; */
    for (idx = 0; idx < 257; idx++) {
      /* squeeze(SwitchMat(k_switch,:,:)); */
      IsEqualFlag[idx] = false;
      p = true;
      ii = 0;
      exitg1 = false;
      while ((!exitg1) && (ii < 8)) {
        if (SwitchMat_esp[(idx + 257 * k_bat2) + 514 * ii] !=
            Switch0[ii + (k_bat2 << 6)]) {
          p = false;
          exitg1 = true;
        } else {
          ii++;
        }
      }
      if (p) {
        IsEqualFlag[idx] = true;
      }
    }
    idx = 0;
    ii = 0;
    exitg1 = false;
    while ((!exitg1) && (ii < 257)) {
      if (IsEqualFlag[ii]) {
        idx++;
        ii_data[idx - 1] = (short)(ii + 1);
        if (idx >= 257) {
          exitg1 = true;
        } else {
          ii++;
        }
      } else {
        ii++;
      }
    }
    if (idx < 1) {
      idx = 0;
    }
    if (idx == 0) {
      ii_data[0] = 0;
    }
    SwitchId[k_bat2] = ii_data[0];
  }
  /*  if coder.target('MATLAB') */
  /*      for k_row = Nbat1*Nbat2-1:-1:1 */
  /*          if all(indNbat(k_row,:)==indNbat(k_row+1,:)) */
  /*              Nrow = size(indNbat,1); */
  /*              indNbat = indNbat([1:Nrow]~=k_row+1,:);%indNbat(k_row+1,:) =
   * 0; */
  /*          end */
  /*      end */
  /*      for k_col = Nbat1*Nbat2-1:-1:1 */
  /*          if all(indNbat(:,k_col)==indNbat(:,k_col+1)) */
  /*              Ncol = size(indNbat,2); */
  /*              indNbat = indNbat(:,[1:Ncol]~=k_col+1);%indNbat(:,k_col+1) =
   * 0; */
  /*          end */
  /*      end */
  /*      indNbat1 = indNbat; */
  /*  else */
  /* forCoder */
  for (i = 0; i < 16; i++) {
    x[i] = (indNbat[i] != 0);
  }
  idx = 0;
  ii = 16;
  exitg1 = false;
  while ((!exitg1) && (ii > 0)) {
    if (x[ii - 1]) {
      idx = 1;
      b_ii_data = (signed char)ii;
      exitg1 = true;
    } else {
      ii--;
    }
  }
  if (idx == 0) {
  }
  for (i = 0; i < 16; i++) {
    x[i] = (indNbat[i << 4] != 0);
  }
  idx = 0;
  ii = 16;
  exitg1 = false;
  while ((!exitg1) && (ii > 0)) {
    if (x[ii - 1]) {
      idx = 1;
      c_ii_data = (signed char)ii;
      exitg1 = true;
    } else {
      ii--;
    }
  }
  if (idx == 0) {
  }
  /* uindNbatRem = unique(uRowIndNbatRem); */
  idx = b_ii_data;
  ii = c_ii_data;
  uindNbatRem_size[0] = b_ii_data;
  uindNbatRem_size[1] = c_ii_data;
  for (i = 0; i < ii; i++) {
    for (jj = 0; jj < idx; jj++) {
      uindNbatRem_data[jj + b_ii_data * i] = indNbat[jj + (i << 4)];
    }
  }
  unique_rows(uindNbatRem_data, uindNbatRem_size, tmp_data, tmp_size);
  uindNbatRem_size[0] = tmp_size[1];
  uindNbatRem_size[1] = tmp_size[0];
  idx = tmp_size[0];
  for (i = 0; i < idx; i++) {
    ii = tmp_size[1];
    for (jj = 0; jj < ii; jj++) {
      uindNbatRem_data[jj + uindNbatRem_size[0] * i] =
          tmp_data[i + tmp_size[0] * jj];
    }
  }
  unique_rows(uindNbatRem_data, uindNbatRem_size, tmp_data, tmp_size);
  uindNbatRem_size[0] = tmp_size[1];
  idx = tmp_size[0];
  for (i = 0; i < idx; i++) {
    ii = tmp_size[1];
    for (jj = 0; jj < ii; jj++) {
      uindNbatRem_data[jj + uindNbatRem_size[0] * i] =
          tmp_data[i + tmp_size[0] * jj];
    }
  }
  /*  end */
  indNbat2[0] = 0.0;
  indNbat2[1] = 0.0;
  indNbat2[2] = 0.0;
  indNbat2[3] = 0.0;
  idx = tmp_size[1];
  if (idx > 2) {
    idx = 2;
  }
  ii = tmp_size[0];
  if (ii > 2) {
    ii = 2;
  }
  for (i = 0; i < ii; i++) {
    for (jj = 0; jj < idx; jj++) {
      b_uindNbatRem_data[jj + idx * i] =
          uindNbatRem_data[jj + uindNbatRem_size[0] * i];
    }
  }
  for (i = 0; i < ii; i++) {
    for (jj = 0; jj < idx; jj++) {
      indNbat2[jj + (i << 1)] = b_uindNbatRem_data[jj + idx * i];
    }
  }
  for (idx = 0; idx < 6; idx++) {
    /* squeeze(SwitchMat(k_switch,:,:)); */
    b_IsEqualFlag[idx] = false;
    p = true;
    ii = 0;
    exitg1 = false;
    while ((!exitg1) && (ii < 4)) {
      if (!(SwitchMat_esp2[(idx + 6 * (ii % 2)) + 12 * (ii / 2)] ==
            indNbat2[ii])) {
        p = false;
        exitg1 = true;
      } else {
        ii++;
      }
    }
    if (p) {
      b_IsEqualFlag[idx] = true;
    }
  }
  idx = 0;
  ii = 0;
  exitg1 = false;
  while ((!exitg1) && (ii < 6)) {
    if (b_IsEqualFlag[ii]) {
      idx++;
      d_ii_data[idx - 1] = (signed char)(ii + 1);
      if (idx >= 6) {
        exitg1 = true;
      } else {
        ii++;
      }
    } else {
      ii++;
    }
  }
  if (idx < 1) {
    idx = 0;
  }
  switchId2_size[0] = 1;
  switchId2_size[1] = idx;
  for (i = 0; i < idx; i++) {
    switchId2_data[i] = d_ii_data[i];
  }
  if (idx == 0) {
    switchId2_size[0] = 1;
    switchId2_size[1] = 1;
    switchId2_data[0] = 0.0;
  }
}

/*
 * File trailer for findSwitchMatEsp.c
 *
 * [EOF]
 */
