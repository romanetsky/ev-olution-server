/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: CalcPortSpiPrm.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 03-Apr-2024 21:30:55
 */

/* Include Files */
#include "CalcPortSpiPrm.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_emxutil.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_rtwutil.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h"
#include "find.h"
#include "findSwitchMatEsp.h"
#include "padArrUint8.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : unsigned char prmBrdSpi_disconnect
 *                unsigned char prmBrdSpi_bypass
 *                const unsigned char prmBrdSpi_PortSpiRow_esp[12336]
 *                const unsigned char prmBrdSpi_SwitchMat_esp[4112]
 *                const unsigned char prmBrdSpi_PortSpiRow_esp2[288]
 *                const unsigned char prmBrdSpi_SwitchMat_esp2[24]
 *                const unsigned char prmBrdSpi_Pac2Vid[4112]
 *                const unsigned char prmBrdSpi_Pac2Vid2[24]
 *                const emxArray_uint8_T *SwitchIn
 *                unsigned char SpiPortRow[24]
 *                double SwitchIdOut_data[]
 *                int SwitchIdOut_size[2]
 *                unsigned char Pac2Vid0[256]
 * Return Type  : void
 */
void CalcPortSpiPrm(unsigned char prmBrdSpi_disconnect,
                    unsigned char prmBrdSpi_bypass,
                    const unsigned char prmBrdSpi_PortSpiRow_esp[12336],
                    const unsigned char prmBrdSpi_SwitchMat_esp[4112],
                    const unsigned char prmBrdSpi_PortSpiRow_esp2[288],
                    const unsigned char prmBrdSpi_SwitchMat_esp2[24],
                    const unsigned char prmBrdSpi_Pac2Vid[4112],
                    const unsigned char prmBrdSpi_Pac2Vid2[24],
                    const emxArray_uint8_T *SwitchIn,
                    unsigned char SpiPortRow[24], double SwitchIdOut_data[],
                    int SwitchIdOut_size[2], unsigned char Pac2Vid0[256])
{
  double SwitchId2_data[36];
  double b_SwitchIdOut_data[8];
  double SwitchId[2];
  int tmp_data[257];
  int b_SwitchId2_data[6];
  int SwitchId2_size[2];
  int SpiPortRow_tmp;
  int SwitchId2;
  int i;
  int k_bat2;
  int k_bat2_col;
  int k_bat2_row;
  int tmp_size;
  unsigned char b_tmp_data[10368];
  unsigned char prmBrdSpi_PortSpiRow_esp2_data[864];
  unsigned char Switch1[256];
  unsigned char k_bat2_0_data[36];
  signed char input_sizes_idx_0;
  unsigned char u;
  boolean_T b_prmBrdSpi_bypass[257];
  boolean_T x_data[36];
  boolean_T c_prmBrdSpi_bypass[6];
  boolean_T exitg1;
  boolean_T y;
  b_padArrUint8(SwitchIn, Switch1);
  /* forCoder */
  /*  if isempty(Switch1) */
  /*      Switch = prmBrdSpi.disconnect; */
  /*  elseif numel(Switch1)==(Nbat2*Nbat1)^2%forCoder */
  /*      %Switch  = reshape(Switch(1,:,:),Nbat2*Nbat1,Nbat2*Nbat1); */
  /*      lastRow = find(Switch1(:,1) ~= 0,1,"last"); */
  /*      lastCol = find(Switch1(1,:) ~= 0,1,"last"); */
  /*      Switch = Switch1(1:lastRow(1),1:lastCol(1)); */
  /*  end */
  if (Switch1[0] == prmBrdSpi_disconnect) {
    for (k_bat2 = 0; k_bat2 < 2; k_bat2++) {
      for (i = 0; i < 257; i++) {
        b_prmBrdSpi_bypass[i] =
            (prmBrdSpi_disconnect == prmBrdSpi_SwitchMat_esp[i + 257 * k_bat2]);
      }
      eml_find(b_prmBrdSpi_bypass, tmp_data);
      SwitchId[k_bat2] = tmp_data[0];
    }
    for (i = 0; i < 6; i++) {
      c_prmBrdSpi_bypass[i] =
          (prmBrdSpi_disconnect == prmBrdSpi_SwitchMat_esp2[i]);
    }
    tmp_size = b_eml_find(c_prmBrdSpi_bypass, b_SwitchId2_data);
    SwitchId2_size[0] = tmp_size;
    SwitchId2_size[1] = 1;
    for (i = 0; i < tmp_size; i++) {
      SwitchId2_data[i] = b_SwitchId2_data[i];
    }
    /*  disConAll = squeeze(PortSpiRow_esp(1,1,:,:)); */
    /*  disConAll(:,2:2:end) = 0; */
    /*  SpiPortRow = disConAll; */
  } else if (Switch1[0] == prmBrdSpi_bypass) {
    for (k_bat2 = 0; k_bat2 < 2; k_bat2++) {
      for (i = 0; i < 257; i++) {
        b_prmBrdSpi_bypass[i] =
            (prmBrdSpi_bypass == prmBrdSpi_SwitchMat_esp[i + 257 * k_bat2]);
      }
      eml_find(b_prmBrdSpi_bypass, tmp_data);
      SwitchId[k_bat2] = tmp_data[0];
    }
    for (i = 0; i < 6; i++) {
      c_prmBrdSpi_bypass[i] = (prmBrdSpi_bypass == prmBrdSpi_SwitchMat_esp2[i]);
    }
    tmp_size = b_eml_find(c_prmBrdSpi_bypass, b_SwitchId2_data);
    SwitchId2_size[0] = tmp_size;
    SwitchId2_size[1] = 1;
    for (i = 0; i < tmp_size; i++) {
      SwitchId2_data[i] = b_SwitchId2_data[i];
    }
  } else {
    findSwitchMatEsp(prmBrdSpi_SwitchMat_esp, prmBrdSpi_SwitchMat_esp2, Switch1,
                     SwitchId, SwitchId2_data, SwitchId2_size);
  }
  /* UNTITLED5 Summary of this function goes here */
  /*    Detailed explanation goes here */
  for (i = 0; i < 24; i++) {
    SpiPortRow[i] = 0U;
  }
  for (k_bat2 = 0; k_bat2 < 2; k_bat2++) {
    if (SwitchId[k_bat2] > 0.0) {
      /*          SpiPortRow =
       * bitor(squeeze(PortSpiRow_esp(SwitchId(k_bat2),k_bat2,:,:)),SpiPortRow);
       */
      for (i = 0; i < 6; i++) {
        tmp_size = i << 2;
        SpiPortRow_tmp = ((int)SwitchId[k_bat2] + 257 * k_bat2) + 2056 * i;
        SpiPortRow[tmp_size] |= prmBrdSpi_PortSpiRow_esp[SpiPortRow_tmp - 1];
        SpiPortRow[tmp_size + 1] |=
            prmBrdSpi_PortSpiRow_esp[SpiPortRow_tmp + 513];
        SpiPortRow[tmp_size + 2] |=
            prmBrdSpi_PortSpiRow_esp[SpiPortRow_tmp + 1027];
        SpiPortRow[tmp_size + 3] |=
            prmBrdSpi_PortSpiRow_esp[SpiPortRow_tmp + 1541];
      }
    }
  }
  tmp_size = SwitchId2_size[0] * SwitchId2_size[1];
  for (i = 0; i < tmp_size; i++) {
    x_data[i] = (SwitchId2_data[i] > 0.0);
  }
  y = ((SwitchId2_size[0] != 0) && (SwitchId2_size[1] != 0));
  if (y) {
    SpiPortRow_tmp = 0;
    exitg1 = false;
    while ((!exitg1) && (SpiPortRow_tmp <= tmp_size - 1)) {
      if (!x_data[SpiPortRow_tmp]) {
        y = false;
        exitg1 = true;
      } else {
        SpiPortRow_tmp++;
      }
    }
  }
  if (y) {
    /*      SpiPortRow =
     * bitor(squeeze(PortSpiRow_esp2(SwitchId2,1,:,:)),SpiPortRow); */
    for (i = 0; i < 6; i++) {
      for (SpiPortRow_tmp = 0; SpiPortRow_tmp < 4; SpiPortRow_tmp++) {
        for (k_bat2 = 0; k_bat2 < tmp_size; k_bat2++) {
          prmBrdSpi_PortSpiRow_esp2_data[(k_bat2 + tmp_size * SpiPortRow_tmp) +
                                         tmp_size * 4 * i] =
              prmBrdSpi_PortSpiRow_esp2[(((int)SwitchId2_data[k_bat2] +
                                          12 * SpiPortRow_tmp) +
                                         48 * i) -
                                        1];
        }
      }
    }
    for (i = 0; i < 24; i++) {
      SpiPortRow[i] |= prmBrdSpi_PortSpiRow_esp2_data[i];
    }
  } else if ((SwitchId2_size[0] == 0) || (SwitchId2_size[1] == 0)) {
    SwitchId2_size[0] = 1;
    SwitchId2_size[1] = 1;
    SwitchId2_data[0] = 0.0;
    /* uint8(0); */
  }
  if ((SwitchId2_size[0] != 0) && (SwitchId2_size[1] != 0)) {
    input_sizes_idx_0 = (signed char)SwitchId2_size[0];
  } else {
    input_sizes_idx_0 = 0;
  }
  tmp_size = input_sizes_idx_0 + 2;
  b_SwitchIdOut_data[0] = SwitchId[0];
  b_SwitchIdOut_data[1] = SwitchId[1];
  SpiPortRow_tmp = input_sizes_idx_0;
  if (SpiPortRow_tmp - 1 >= 0) {
    memcpy(&b_SwitchIdOut_data[2], &SwitchId2_data[0],
           (unsigned int)SpiPortRow_tmp * sizeof(double));
  }
  SwitchIdOut_size[0] = input_sizes_idx_0 + 2;
  SwitchIdOut_size[1] = 1;
  memcpy(&SwitchIdOut_data[0], &b_SwitchIdOut_data[0],
         (unsigned int)tmp_size * sizeof(double));
  /* Pac ground at 16 */
  memset(&Pac2Vid0[0], 0, 256U * sizeof(unsigned char));
  SwitchId2 = SwitchId2_size[0] * SwitchId2_size[1];
  for (k_bat2_col = 0; k_bat2_col < 2; k_bat2_col++) {
    for (k_bat2_row = 0; k_bat2_row < 2; k_bat2_row++) {
      for (i = 0; i < SwitchId2; i++) {
        u = prmBrdSpi_Pac2Vid2[(((int)SwitchId2_data[i] + 6 * k_bat2_row) +
                                12 * k_bat2_col) -
                               1];
        k_bat2_0_data[i] = u;
        x_data[i] = (u > 0);
      }
      y = (SwitchId2 != 0);
      if (y) {
        SpiPortRow_tmp = 0;
        exitg1 = false;
        while ((!exitg1) && (SpiPortRow_tmp <= SwitchId2 - 1)) {
          if (!x_data[SpiPortRow_tmp]) {
            y = false;
            exitg1 = true;
          } else {
            SpiPortRow_tmp++;
          }
        }
      }
      if (y) {
        for (i = 0; i < SwitchId2; i++) {
          x_data[i] = (SwitchId[k_bat2_0_data[i] - 1] > 0.0);
        }
        y = (SwitchId2 != 0);
        if (y) {
          SpiPortRow_tmp = 0;
          exitg1 = false;
          while ((!exitg1) && (SpiPortRow_tmp <= SwitchId2 - 1)) {
            if (!x_data[SpiPortRow_tmp]) {
              y = false;
              exitg1 = true;
            } else {
              SpiPortRow_tmp++;
            }
          }
        }
        if (y) {
          for (i = 0; i < 8; i++) {
            for (SpiPortRow_tmp = 0; SpiPortRow_tmp < SwitchId2;
                 SpiPortRow_tmp++) {
              for (k_bat2 = 0; k_bat2 < SwitchId2; k_bat2++) {
                b_tmp_data[(k_bat2 + SwitchId2 * SpiPortRow_tmp) +
                           SwitchId2 * SwitchId2 * i] = prmBrdSpi_Pac2Vid
                    [(((int)SwitchId[k_bat2_0_data[k_bat2] - 1] +
                       257 * (k_bat2_0_data[SpiPortRow_tmp] - 1)) +
                      514 * i) -
                     1];
              }
            }
          }
          tmp_size = k_bat2_row << 3;
          SpiPortRow_tmp = k_bat2_col << 3;
          for (i = 0; i < 8; i++) {
            Pac2Vid0[(i + tmp_size) + (SpiPortRow_tmp << 4)] = b_tmp_data[i];
          }
          /* squeeze(Pac2Vid(SwitchId(k_bat2_0),k_bat2_0,:,:)); */
        }
      }
    }
  }
}

/*
 * Arguments    : unsigned char prmBrdSpi_disconnect
 *                unsigned char prmBrdSpi_bypass
 *                const unsigned char prmBrdSpi_PortSpiRow_esp[12336]
 *                const unsigned char prmBrdSpi_SwitchMat_esp[4112]
 *                const unsigned char prmBrdSpi_PortSpiRow_esp2[288]
 *                const unsigned char prmBrdSpi_SwitchMat_esp2[24]
 *                const unsigned char prmBrdSpi_Pac2Vid[4112]
 *                const unsigned char prmBrdSpi_Pac2Vid2[24]
 *                const unsigned char SwitchIn[256]
 *                unsigned char SpiPortRow[24]
 *                double SwitchIdOut_data[]
 *                int SwitchIdOut_size[2]
 *                unsigned char Pac2Vid0[256]
 * Return Type  : void
 */
void b_CalcPortSpiPrm(unsigned char prmBrdSpi_disconnect,
                      unsigned char prmBrdSpi_bypass,
                      const unsigned char prmBrdSpi_PortSpiRow_esp[12336],
                      const unsigned char prmBrdSpi_SwitchMat_esp[4112],
                      const unsigned char prmBrdSpi_PortSpiRow_esp2[288],
                      const unsigned char prmBrdSpi_SwitchMat_esp2[24],
                      const unsigned char prmBrdSpi_Pac2Vid[4112],
                      const unsigned char prmBrdSpi_Pac2Vid2[24],
                      const unsigned char SwitchIn[256],
                      unsigned char SpiPortRow[24], double SwitchIdOut_data[],
                      int SwitchIdOut_size[2], unsigned char Pac2Vid0[256])
{
  double SwitchId2_data[36];
  double b_SwitchIdOut_data[8];
  double SwitchId[2];
  int tmp_data[257];
  int b_SwitchId2_data[6];
  int SwitchId2_size[2];
  int i;
  int idx;
  int ii;
  int k_bat2;
  int k_bat2_row;
  int loop_ub_tmp;
  unsigned char b_tmp_data[10368];
  unsigned char prmBrdSpi_PortSpiRow_esp2_data[864];
  unsigned char Switch1[256];
  unsigned char SwitchIn_data[256];
  unsigned char k_bat2_0_data[36];
  signed char b_ii_data;
  signed char ii_data;
  unsigned char u;
  boolean_T b_prmBrdSpi_bypass[257];
  boolean_T x_data[36];
  boolean_T x[16];
  boolean_T c_prmBrdSpi_bypass[6];
  boolean_T exitg1;
  boolean_T y;
  memset(&Switch1[0], 0, 256U * sizeof(unsigned char));
  for (i = 0; i < 16; i++) {
    x[i] = (SwitchIn[i] != 0);
  }
  idx = 0;
  ii = 16;
  exitg1 = false;
  while ((!exitg1) && (ii > 0)) {
    if (x[ii - 1]) {
      idx = 1;
      ii_data = (signed char)ii;
      exitg1 = true;
    } else {
      ii--;
    }
  }
  if (idx == 0) {
  }
  for (i = 0; i < 16; i++) {
    x[i] = (SwitchIn[i << 4] != 0);
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
  loop_ub_tmp = ii_data;
  idx = b_ii_data;
  for (i = 0; i < idx; i++) {
    for (k_bat2 = 0; k_bat2 < loop_ub_tmp; k_bat2++) {
      SwitchIn_data[k_bat2 + ii_data * i] = SwitchIn[k_bat2 + (i << 4)];
    }
  }
  for (i = 0; i < idx; i++) {
    for (k_bat2 = 0; k_bat2 < loop_ub_tmp; k_bat2++) {
      Switch1[k_bat2 + (i << 4)] = SwitchIn_data[k_bat2 + ii_data * i];
    }
  }
  /* forCoder */
  /*  if isempty(Switch1) */
  /*      Switch = prmBrdSpi.disconnect; */
  /*  elseif numel(Switch1)==(Nbat2*Nbat1)^2%forCoder */
  /*      %Switch  = reshape(Switch(1,:,:),Nbat2*Nbat1,Nbat2*Nbat1); */
  /*      lastRow = find(Switch1(:,1) ~= 0,1,"last"); */
  /*      lastCol = find(Switch1(1,:) ~= 0,1,"last"); */
  /*      Switch = Switch1(1:lastRow(1),1:lastCol(1)); */
  /*  end */
  if (Switch1[0] == prmBrdSpi_disconnect) {
    for (k_bat2 = 0; k_bat2 < 2; k_bat2++) {
      for (i = 0; i < 257; i++) {
        b_prmBrdSpi_bypass[i] =
            (prmBrdSpi_disconnect == prmBrdSpi_SwitchMat_esp[i + 257 * k_bat2]);
      }
      eml_find(b_prmBrdSpi_bypass, tmp_data);
      SwitchId[k_bat2] = tmp_data[0];
    }
    for (i = 0; i < 6; i++) {
      c_prmBrdSpi_bypass[i] =
          (prmBrdSpi_disconnect == prmBrdSpi_SwitchMat_esp2[i]);
    }
    idx = b_eml_find(c_prmBrdSpi_bypass, b_SwitchId2_data);
    SwitchId2_size[0] = idx;
    SwitchId2_size[1] = 1;
    for (i = 0; i < idx; i++) {
      SwitchId2_data[i] = b_SwitchId2_data[i];
    }
    /*  disConAll = squeeze(PortSpiRow_esp(1,1,:,:)); */
    /*  disConAll(:,2:2:end) = 0; */
    /*  SpiPortRow = disConAll; */
  } else if (Switch1[0] == prmBrdSpi_bypass) {
    for (k_bat2 = 0; k_bat2 < 2; k_bat2++) {
      for (i = 0; i < 257; i++) {
        b_prmBrdSpi_bypass[i] =
            (prmBrdSpi_bypass == prmBrdSpi_SwitchMat_esp[i + 257 * k_bat2]);
      }
      eml_find(b_prmBrdSpi_bypass, tmp_data);
      SwitchId[k_bat2] = tmp_data[0];
    }
    for (i = 0; i < 6; i++) {
      c_prmBrdSpi_bypass[i] = (prmBrdSpi_bypass == prmBrdSpi_SwitchMat_esp2[i]);
    }
    idx = b_eml_find(c_prmBrdSpi_bypass, b_SwitchId2_data);
    SwitchId2_size[0] = idx;
    SwitchId2_size[1] = 1;
    for (i = 0; i < idx; i++) {
      SwitchId2_data[i] = b_SwitchId2_data[i];
    }
  } else {
    findSwitchMatEsp(prmBrdSpi_SwitchMat_esp, prmBrdSpi_SwitchMat_esp2, Switch1,
                     SwitchId, SwitchId2_data, SwitchId2_size);
  }
  /* UNTITLED5 Summary of this function goes here */
  /*    Detailed explanation goes here */
  for (i = 0; i < 24; i++) {
    SpiPortRow[i] = 0U;
  }
  for (k_bat2 = 0; k_bat2 < 2; k_bat2++) {
    if (SwitchId[k_bat2] > 0.0) {
      /*          SpiPortRow =
       * bitor(squeeze(PortSpiRow_esp(SwitchId(k_bat2),k_bat2,:,:)),SpiPortRow);
       */
      for (i = 0; i < 6; i++) {
        idx = i << 2;
        loop_ub_tmp = ((int)SwitchId[k_bat2] + 257 * k_bat2) + 2056 * i;
        SpiPortRow[idx] |= prmBrdSpi_PortSpiRow_esp[loop_ub_tmp - 1];
        SpiPortRow[idx + 1] |= prmBrdSpi_PortSpiRow_esp[loop_ub_tmp + 513];
        SpiPortRow[idx + 2] |= prmBrdSpi_PortSpiRow_esp[loop_ub_tmp + 1027];
        SpiPortRow[idx + 3] |= prmBrdSpi_PortSpiRow_esp[loop_ub_tmp + 1541];
      }
    }
  }
  loop_ub_tmp = SwitchId2_size[0] * SwitchId2_size[1];
  for (i = 0; i < loop_ub_tmp; i++) {
    x_data[i] = (SwitchId2_data[i] > 0.0);
  }
  y = ((SwitchId2_size[0] != 0) && (SwitchId2_size[1] != 0));
  if (y) {
    idx = 0;
    exitg1 = false;
    while ((!exitg1) && (idx <= loop_ub_tmp - 1)) {
      if (!x_data[idx]) {
        y = false;
        exitg1 = true;
      } else {
        idx++;
      }
    }
  }
  if (y) {
    /*      SpiPortRow =
     * bitor(squeeze(PortSpiRow_esp2(SwitchId2,1,:,:)),SpiPortRow); */
    for (i = 0; i < 6; i++) {
      for (k_bat2 = 0; k_bat2 < 4; k_bat2++) {
        for (idx = 0; idx < loop_ub_tmp; idx++) {
          prmBrdSpi_PortSpiRow_esp2_data[(idx + loop_ub_tmp * k_bat2) +
                                         loop_ub_tmp * 4 * i] =
              prmBrdSpi_PortSpiRow_esp2
                  [(((int)SwitchId2_data[idx] + 12 * k_bat2) + 48 * i) - 1];
        }
      }
    }
    for (i = 0; i < 24; i++) {
      SpiPortRow[i] |= prmBrdSpi_PortSpiRow_esp2_data[i];
    }
  } else if ((SwitchId2_size[0] == 0) || (SwitchId2_size[1] == 0)) {
    SwitchId2_size[0] = 1;
    SwitchId2_size[1] = 1;
    SwitchId2_data[0] = 0.0;
    /* uint8(0); */
  }
  if ((SwitchId2_size[0] != 0) && (SwitchId2_size[1] != 0)) {
    ii_data = (signed char)SwitchId2_size[0];
  } else {
    ii_data = 0;
  }
  idx = ii_data + 2;
  b_SwitchIdOut_data[0] = SwitchId[0];
  b_SwitchIdOut_data[1] = SwitchId[1];
  loop_ub_tmp = ii_data;
  if (loop_ub_tmp - 1 >= 0) {
    memcpy(&b_SwitchIdOut_data[2], &SwitchId2_data[0],
           (unsigned int)loop_ub_tmp * sizeof(double));
  }
  SwitchIdOut_size[0] = ii_data + 2;
  SwitchIdOut_size[1] = 1;
  memcpy(&SwitchIdOut_data[0], &b_SwitchIdOut_data[0],
         (unsigned int)idx * sizeof(double));
  /* Pac ground at 16 */
  memset(&Pac2Vid0[0], 0, 256U * sizeof(unsigned char));
  ii = SwitchId2_size[0] * SwitchId2_size[1];
  for (loop_ub_tmp = 0; loop_ub_tmp < 2; loop_ub_tmp++) {
    for (k_bat2_row = 0; k_bat2_row < 2; k_bat2_row++) {
      for (i = 0; i < ii; i++) {
        u = prmBrdSpi_Pac2Vid2[(((int)SwitchId2_data[i] + 6 * k_bat2_row) +
                                12 * loop_ub_tmp) -
                               1];
        k_bat2_0_data[i] = u;
        x_data[i] = (u > 0);
      }
      y = (ii != 0);
      if (y) {
        idx = 0;
        exitg1 = false;
        while ((!exitg1) && (idx <= ii - 1)) {
          if (!x_data[idx]) {
            y = false;
            exitg1 = true;
          } else {
            idx++;
          }
        }
      }
      if (y) {
        for (i = 0; i < ii; i++) {
          x_data[i] = (SwitchId[k_bat2_0_data[i] - 1] > 0.0);
        }
        y = (ii != 0);
        if (y) {
          idx = 0;
          exitg1 = false;
          while ((!exitg1) && (idx <= ii - 1)) {
            if (!x_data[idx]) {
              y = false;
              exitg1 = true;
            } else {
              idx++;
            }
          }
        }
        if (y) {
          for (i = 0; i < 8; i++) {
            for (k_bat2 = 0; k_bat2 < ii; k_bat2++) {
              for (idx = 0; idx < ii; idx++) {
                b_tmp_data[(idx + ii * k_bat2) + ii * ii * i] =
                    prmBrdSpi_Pac2Vid[(((int)SwitchId[k_bat2_0_data[idx] - 1] +
                                        257 * (k_bat2_0_data[k_bat2] - 1)) +
                                       514 * i) -
                                      1];
              }
            }
          }
          idx = k_bat2_row << 3;
          k_bat2 = loop_ub_tmp << 3;
          for (i = 0; i < 8; i++) {
            Pac2Vid0[(i + idx) + (k_bat2 << 4)] = b_tmp_data[i];
          }
          /* squeeze(Pac2Vid(SwitchId(k_bat2_0),k_bat2_0,:,:)); */
        }
      }
    }
  }
}

/*
 * Arguments    : unsigned char prmBrdSpi_disconnect
 *                unsigned char prmBrdSpi_bypass
 *                const unsigned char prmBrdSpi_PortSpiRow_esp[12336]
 *                const unsigned char prmBrdSpi_SwitchMat_esp[4112]
 *                const unsigned char prmBrdSpi_PortSpiRow_esp2[288]
 *                const unsigned char prmBrdSpi_SwitchMat_esp2[24]
 *                const unsigned char prmBrdSpi_Pac2Vid[4112]
 *                const unsigned char prmBrdSpi_Pac2Vid2[24]
 *                const emxArray_real_T *SwitchIn
 *                unsigned char SpiPortRow[24]
 *                double SwitchIdOut_data[]
 *                int SwitchIdOut_size[2]
 *                unsigned char Pac2Vid0[256]
 * Return Type  : void
 */
void c_CalcPortSpiPrm(unsigned char prmBrdSpi_disconnect,
                      unsigned char prmBrdSpi_bypass,
                      const unsigned char prmBrdSpi_PortSpiRow_esp[12336],
                      const unsigned char prmBrdSpi_SwitchMat_esp[4112],
                      const unsigned char prmBrdSpi_PortSpiRow_esp2[288],
                      const unsigned char prmBrdSpi_SwitchMat_esp2[24],
                      const unsigned char prmBrdSpi_Pac2Vid[4112],
                      const unsigned char prmBrdSpi_Pac2Vid2[24],
                      const emxArray_real_T *SwitchIn,
                      unsigned char SpiPortRow[24], double SwitchIdOut_data[],
                      int SwitchIdOut_size[2], unsigned char Pac2Vid0[256])
{
  emxArray_uint8_T *b_SwitchIn;
  double SwitchId2_data[36];
  double b_SwitchIdOut_data[8];
  double SwitchId[2];
  const double *SwitchIn_data;
  double d;
  int tmp_data[257];
  int b_SwitchId2_data[6];
  int SwitchId2_size[2];
  int i;
  int i1;
  int k_bat2;
  int k_bat2_col;
  int k_bat2_row;
  int loop_ub;
  int tmp_size;
  unsigned char b_tmp_data[10368];
  unsigned char prmBrdSpi_PortSpiRow_esp2_data[864];
  unsigned char Switch1[256];
  unsigned char k_bat2_0_data[36];
  signed char input_sizes_idx_0;
  unsigned char u;
  unsigned char *b_SwitchIn_data;
  boolean_T b_prmBrdSpi_bypass[257];
  boolean_T x_data[36];
  boolean_T c_prmBrdSpi_bypass[6];
  boolean_T exitg1;
  boolean_T y;
  SwitchIn_data = SwitchIn->data;
  emxInit_uint8_T(&b_SwitchIn, 2);
  i = b_SwitchIn->size[0] * b_SwitchIn->size[1];
  b_SwitchIn->size[0] = SwitchIn->size[0];
  b_SwitchIn->size[1] = SwitchIn->size[1];
  emxEnsureCapacity_uint8_T(b_SwitchIn, i);
  b_SwitchIn_data = b_SwitchIn->data;
  loop_ub = SwitchIn->size[0] * SwitchIn->size[1];
  for (i = 0; i < loop_ub; i++) {
    d = rt_roundd_snf(SwitchIn_data[i]);
    if (d < 256.0) {
      if (d >= 0.0) {
        u = (unsigned char)d;
      } else {
        u = 0U;
      }
    } else if (d >= 256.0) {
      u = MAX_uint8_T;
    } else {
      u = 0U;
    }
    b_SwitchIn_data[i] = u;
  }
  b_padArrUint8(b_SwitchIn, Switch1);
  emxFree_uint8_T(&b_SwitchIn);
  /* forCoder */
  /*  if isempty(Switch1) */
  /*      Switch = prmBrdSpi.disconnect; */
  /*  elseif numel(Switch1)==(Nbat2*Nbat1)^2%forCoder */
  /*      %Switch  = reshape(Switch(1,:,:),Nbat2*Nbat1,Nbat2*Nbat1); */
  /*      lastRow = find(Switch1(:,1) ~= 0,1,"last"); */
  /*      lastCol = find(Switch1(1,:) ~= 0,1,"last"); */
  /*      Switch = Switch1(1:lastRow(1),1:lastCol(1)); */
  /*  end */
  if (Switch1[0] == prmBrdSpi_disconnect) {
    for (k_bat2 = 0; k_bat2 < 2; k_bat2++) {
      for (i = 0; i < 257; i++) {
        b_prmBrdSpi_bypass[i] =
            (prmBrdSpi_disconnect == prmBrdSpi_SwitchMat_esp[i + 257 * k_bat2]);
      }
      eml_find(b_prmBrdSpi_bypass, tmp_data);
      SwitchId[k_bat2] = tmp_data[0];
    }
    for (i = 0; i < 6; i++) {
      c_prmBrdSpi_bypass[i] =
          (prmBrdSpi_disconnect == prmBrdSpi_SwitchMat_esp2[i]);
    }
    tmp_size = b_eml_find(c_prmBrdSpi_bypass, b_SwitchId2_data);
    SwitchId2_size[0] = tmp_size;
    SwitchId2_size[1] = 1;
    for (i = 0; i < tmp_size; i++) {
      SwitchId2_data[i] = b_SwitchId2_data[i];
    }
    /*  disConAll = squeeze(PortSpiRow_esp(1,1,:,:)); */
    /*  disConAll(:,2:2:end) = 0; */
    /*  SpiPortRow = disConAll; */
  } else if (Switch1[0] == prmBrdSpi_bypass) {
    for (k_bat2 = 0; k_bat2 < 2; k_bat2++) {
      for (i = 0; i < 257; i++) {
        b_prmBrdSpi_bypass[i] =
            (prmBrdSpi_bypass == prmBrdSpi_SwitchMat_esp[i + 257 * k_bat2]);
      }
      eml_find(b_prmBrdSpi_bypass, tmp_data);
      SwitchId[k_bat2] = tmp_data[0];
    }
    for (i = 0; i < 6; i++) {
      c_prmBrdSpi_bypass[i] = (prmBrdSpi_bypass == prmBrdSpi_SwitchMat_esp2[i]);
    }
    tmp_size = b_eml_find(c_prmBrdSpi_bypass, b_SwitchId2_data);
    SwitchId2_size[0] = tmp_size;
    SwitchId2_size[1] = 1;
    for (i = 0; i < tmp_size; i++) {
      SwitchId2_data[i] = b_SwitchId2_data[i];
    }
  } else {
    findSwitchMatEsp(prmBrdSpi_SwitchMat_esp, prmBrdSpi_SwitchMat_esp2, Switch1,
                     SwitchId, SwitchId2_data, SwitchId2_size);
  }
  /* UNTITLED5 Summary of this function goes here */
  /*    Detailed explanation goes here */
  for (i = 0; i < 24; i++) {
    SpiPortRow[i] = 0U;
  }
  for (k_bat2 = 0; k_bat2 < 2; k_bat2++) {
    if (SwitchId[k_bat2] > 0.0) {
      /*          SpiPortRow =
       * bitor(squeeze(PortSpiRow_esp(SwitchId(k_bat2),k_bat2,:,:)),SpiPortRow);
       */
      for (i = 0; i < 6; i++) {
        tmp_size = i << 2;
        loop_ub = ((int)SwitchId[k_bat2] + 257 * k_bat2) + 2056 * i;
        SpiPortRow[tmp_size] |= prmBrdSpi_PortSpiRow_esp[loop_ub - 1];
        SpiPortRow[tmp_size + 1] |= prmBrdSpi_PortSpiRow_esp[loop_ub + 513];
        SpiPortRow[tmp_size + 2] |= prmBrdSpi_PortSpiRow_esp[loop_ub + 1027];
        SpiPortRow[tmp_size + 3] |= prmBrdSpi_PortSpiRow_esp[loop_ub + 1541];
      }
    }
  }
  tmp_size = SwitchId2_size[0] * SwitchId2_size[1];
  for (i = 0; i < tmp_size; i++) {
    x_data[i] = (SwitchId2_data[i] > 0.0);
  }
  y = ((SwitchId2_size[0] != 0) && (SwitchId2_size[1] != 0));
  if (y) {
    loop_ub = 0;
    exitg1 = false;
    while ((!exitg1) && (loop_ub <= tmp_size - 1)) {
      if (!x_data[loop_ub]) {
        y = false;
        exitg1 = true;
      } else {
        loop_ub++;
      }
    }
  }
  if (y) {
    /*      SpiPortRow =
     * bitor(squeeze(PortSpiRow_esp2(SwitchId2,1,:,:)),SpiPortRow); */
    for (i = 0; i < 6; i++) {
      for (loop_ub = 0; loop_ub < 4; loop_ub++) {
        for (i1 = 0; i1 < tmp_size; i1++) {
          prmBrdSpi_PortSpiRow_esp2_data[(i1 + tmp_size * loop_ub) +
                                         tmp_size * 4 * i] =
              prmBrdSpi_PortSpiRow_esp2
                  [(((int)SwitchId2_data[i1] + 12 * loop_ub) + 48 * i) - 1];
        }
      }
    }
    for (i = 0; i < 24; i++) {
      SpiPortRow[i] |= prmBrdSpi_PortSpiRow_esp2_data[i];
    }
  } else if ((SwitchId2_size[0] == 0) || (SwitchId2_size[1] == 0)) {
    SwitchId2_size[0] = 1;
    SwitchId2_size[1] = 1;
    SwitchId2_data[0] = 0.0;
    /* uint8(0); */
  }
  if ((SwitchId2_size[0] != 0) && (SwitchId2_size[1] != 0)) {
    input_sizes_idx_0 = (signed char)SwitchId2_size[0];
  } else {
    input_sizes_idx_0 = 0;
  }
  tmp_size = input_sizes_idx_0 + 2;
  b_SwitchIdOut_data[0] = SwitchId[0];
  b_SwitchIdOut_data[1] = SwitchId[1];
  loop_ub = input_sizes_idx_0;
  if (loop_ub - 1 >= 0) {
    memcpy(&b_SwitchIdOut_data[2], &SwitchId2_data[0],
           (unsigned int)loop_ub * sizeof(double));
  }
  SwitchIdOut_size[0] = input_sizes_idx_0 + 2;
  SwitchIdOut_size[1] = 1;
  memcpy(&SwitchIdOut_data[0], &b_SwitchIdOut_data[0],
         (unsigned int)tmp_size * sizeof(double));
  /* Pac ground at 16 */
  memset(&Pac2Vid0[0], 0, 256U * sizeof(unsigned char));
  k_bat2 = SwitchId2_size[0] * SwitchId2_size[1];
  for (k_bat2_col = 0; k_bat2_col < 2; k_bat2_col++) {
    for (k_bat2_row = 0; k_bat2_row < 2; k_bat2_row++) {
      for (i = 0; i < k_bat2; i++) {
        u = prmBrdSpi_Pac2Vid2[(((int)SwitchId2_data[i] + 6 * k_bat2_row) +
                                12 * k_bat2_col) -
                               1];
        k_bat2_0_data[i] = u;
        x_data[i] = (u > 0);
      }
      y = (k_bat2 != 0);
      if (y) {
        loop_ub = 0;
        exitg1 = false;
        while ((!exitg1) && (loop_ub <= k_bat2 - 1)) {
          if (!x_data[loop_ub]) {
            y = false;
            exitg1 = true;
          } else {
            loop_ub++;
          }
        }
      }
      if (y) {
        for (i = 0; i < k_bat2; i++) {
          x_data[i] = (SwitchId[k_bat2_0_data[i] - 1] > 0.0);
        }
        y = (k_bat2 != 0);
        if (y) {
          loop_ub = 0;
          exitg1 = false;
          while ((!exitg1) && (loop_ub <= k_bat2 - 1)) {
            if (!x_data[loop_ub]) {
              y = false;
              exitg1 = true;
            } else {
              loop_ub++;
            }
          }
        }
        if (y) {
          for (i = 0; i < 8; i++) {
            for (loop_ub = 0; loop_ub < k_bat2; loop_ub++) {
              for (i1 = 0; i1 < k_bat2; i1++) {
                b_tmp_data[(i1 + k_bat2 * loop_ub) + k_bat2 * k_bat2 * i] =
                    prmBrdSpi_Pac2Vid[(((int)SwitchId[k_bat2_0_data[i1] - 1] +
                                        257 * (k_bat2_0_data[loop_ub] - 1)) +
                                       514 * i) -
                                      1];
              }
            }
          }
          tmp_size = k_bat2_row << 3;
          loop_ub = k_bat2_col << 3;
          for (i = 0; i < 8; i++) {
            Pac2Vid0[(i + tmp_size) + (loop_ub << 4)] = b_tmp_data[i];
          }
          /* squeeze(Pac2Vid(SwitchId(k_bat2_0),k_bat2_0,:,:)); */
        }
      }
    }
  }
}

/*
 * File trailer for CalcPortSpiPrm.c
 *
 * [EOF]
 */
