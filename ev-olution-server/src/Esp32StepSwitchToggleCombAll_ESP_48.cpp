/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: Esp32StepSwitchToggleCombAll_ESP_48.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 10-Apr-2024 21:46:28
 */

/* Include Files */
#include "Esp32StepSwitchToggleCombAll_ESP_48.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_emxutil.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_rtwutil.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h"
#include "any.h"
#include "combineVectorElements.h"
#include "find.h"
#include "isequal.h"
#include "minOrMax.h"
#include "nchoosek.h"
#include "nonSingletonDim.h"
#include "padArr.h"
#include "remPadArr.h"
#include "rt_nonfinite.h"
#include "sort.h"
#include "sum.h"
#include "useConstantDim.h"
#include <math.h>
#include <string.h>

/* Function Declarations */
static int binary_expand_op_11(boolean_T in1_data[], const boolean_T in2_data[],
                               const int in2_size[2],
                               const boolean_T in3_data[], const int *in3_size);

static int binary_expand_op_9(double in1_data[], const double in2_data[],
                              const int *in2_size, const boolean_T in3_data[],
                              const int *in3_size);

/* Function Definitions */
/*
 * Arguments    : boolean_T in1_data[]
 *                const boolean_T in2_data[]
 *                const int in2_size[2]
 *                const boolean_T in3_data[]
 *                const int *in3_size
 * Return Type  : int
 */
static int binary_expand_op_11(boolean_T in1_data[], const boolean_T in2_data[],
                               const int in2_size[2],
                               const boolean_T in3_data[], const int *in3_size)
{
  int i;
  int in1_size;
  int in2;
  int stride_1_0;
  in2 = in2_size[0] * in2_size[1];
  if (*in3_size == 1) {
    in1_size = in2;
  } else {
    in1_size = *in3_size;
  }
  in2 = (in2 != 1);
  stride_1_0 = (*in3_size != 1);
  for (i = 0; i < in1_size; i++) {
    in1_data[i] = (in2_data[i * in2] && (!in3_data[i * stride_1_0]));
  }
  return in1_size;
}

/*
 * Arguments    : double in1_data[]
 *                const double in2_data[]
 *                const int *in2_size
 *                const boolean_T in3_data[]
 *                const int *in3_size
 * Return Type  : int
 */
static int binary_expand_op_9(double in1_data[], const double in2_data[],
                              const int *in2_size, const boolean_T in3_data[],
                              const int *in3_size)
{
  int i;
  int in1_size;
  int stride_0_0;
  int stride_1_0;
  if (*in3_size == 1) {
    in1_size = *in2_size;
  } else {
    in1_size = *in3_size;
  }
  stride_0_0 = (*in2_size != 1);
  stride_1_0 = (*in3_size != 1);
  for (i = 0; i < in1_size; i++) {
    in1_data[i] = in2_data[i * stride_0_0] * (double)in3_data[i * stride_1_0];
  }
  return in1_size;
}

/*
 * Arguments    : double Sel_data[]
 *                int Sel_size[2]
 *                float CutOffChrV
 *                float CutOffDisV
 *                double BattConfigPerIna_data[]
 *                const int BattConfigPerIna_size[3]
 *                const double Vbat_data[]
 *                int Vbat_size
 *                double Nina219
 *                short prmBrd_Nbat
 *                unsigned char prmBrd_spi_disconnect
 *                unsigned char prmBrd_spi_bypass
 *                float prmCnfg_Ttoggle
 *                short prmCnfg_NtoggleDrop
 *                short prmCnfg_minLenIna219
 *                const struct4_T prmSeq
 *                signed char VthFlag
 *                const float t_data[]
 *                const int t_size[2]
 *                double tLastToggle_data[]
 *                int tLastToggle_size[2]
 *                boolean_T BattConfigAct_data[]
 *                const int BattConfigAct_size[2]
 *                boolean_T changeConfigFlag_data[]
 *                int changeConfigFlag_size[2]
 * Return Type  : boolean_T
 */
boolean_T c_Esp32StepSwitchToggleCombAll_(
    double Sel_data[], int Sel_size[2], float CutOffChrV, float CutOffDisV,
    double BattConfigPerIna_data[], const int BattConfigPerIna_size[3],
    const double Vbat_data[], int Vbat_size, double Nina219, short prmBrd_Nbat,
    unsigned char prmBrd_spi_disconnect, unsigned char prmBrd_spi_bypass,
    float prmCnfg_Ttoggle, short prmCnfg_NtoggleDrop,
    short prmCnfg_minLenIna219, const struct4_T prmSeq, signed char VthFlag,
    const float t_data[], const int t_size[2], double tLastToggle_data[],
    int tLastToggle_size[2], boolean_T BattConfigAct_data[],
    const int BattConfigAct_size[2], boolean_T changeConfigFlag_data[],
    int changeConfigFlag_size[2])
{
  emxArray_boolean_T c_Sel_data;
  emxArray_boolean_T d_Sel_data;
  emxArray_boolean_T e_Sel_data;
  emxArray_boolean_T f_Sel_data;
  emxArray_boolean_T g_Sel_data;
  emxArray_boolean_T h_Sel_data;
  emxArray_boolean_T *x;
  emxArray_real32_T *b_x;
  emxArray_real32_T *c_y;
  emxArray_real_T *C;
  emxArray_real_T *VbatC;
  emxArray_real_T *Vc;
  emxArray_real_T *r;
  emxArray_uint16_T *b_y;
  double b_BattConfigPerIna_data[256];
  double tmp_data[256];
  double BattConfigPerInaNew_data[34];
  double b_y_data[32];
  double sVbat0_data[32];
  double b_tmp_data[16];
  double b_k_ina219;
  double d;
  double sumV;
  double *C_data;
  double *VbatC_data;
  float VthDis;
  float VthOvDis;
  float *b_x_data;
  float *c_y_data;
  int ind2Stay_data[34];
  int iidx_data[32];
  int b_BattConfigPerIna_size[3];
  int VbatC_size[2];
  int b_Sel_size[2];
  int b_Vbat_size[2];
  int tmp_size[2];
  int b_i;
  int b_loop_ub;
  int c_loop_ub;
  int d_loop_ub;
  int e_loop_ub;
  int end;
  int findId0_data;
  int i;
  int k_ina219;
  int k_nc;
  int loop_ub;
  int loop_ub_tmp;
  int nx;
  unsigned int sumNotBypass;
  int trueCount;
  short d_y;
  unsigned short *y_data;
  signed char c_tmp_data[32];
  boolean_T d_tmp_data[32767];
  boolean_T b_VbatC_data[34];
  boolean_T b_ind2out_data[32];
  boolean_T batAct_data[32];
  boolean_T e_tmp_data[32];
  boolean_T ind2out_data[16];
  boolean_T b_Sel_data[2];
  boolean_T UnOvVflag;
  boolean_T b;
  boolean_T exitg1;
  boolean_T exitg2;
  boolean_T guard1;
  boolean_T switchFlag;
  boolean_T y;
  boolean_T *x_data;
  VthDis = prmSeq.VthDis;
  /*  -1 - NA */
  /*  VthFlag         = prmSeq.VthFlag; % true - Vth mode on */
  VthOvDis = prmSeq.VthOvDis;
  changeConfigFlag_size[0] = 1;
  loop_ub_tmp = (int)Nina219;
  changeConfigFlag_size[1] = (int)Nina219;
  switchFlag = false;
  /*  SwitchToggleFlage = false; */
  k_nc = 0;
  /*  1 */
  sumV = 0.0;
  for (k_ina219 = 0; k_ina219 < loop_ub_tmp; k_ina219++) {
    changeConfigFlag_data[k_ina219] = false;
    d = BattConfigPerIna_data[k_ina219];
    if ((d != prmBrd_spi_bypass) && (d != prmBrd_spi_disconnect)) {
      b_BattConfigPerIna_size[0] = 1;
      b_BattConfigPerIna_size[1] = BattConfigPerIna_size[1];
      loop_ub = BattConfigPerIna_size[2];
      b_BattConfigPerIna_size[2] = loop_ub;
      for (i = 0; i < loop_ub; i++) {
        b_loop_ub = BattConfigPerIna_size[1];
        for (findId0_data = 0; findId0_data < b_loop_ub; findId0_data++) {
          b_BattConfigPerIna_data[findId0_data + b_BattConfigPerIna_size[1] *
                                                     i] = BattConfigPerIna_data
              [(k_ina219 + BattConfigPerIna_size[0] * findId0_data) +
               BattConfigPerIna_size[0] * BattConfigPerIna_size[1] * i];
        }
      }
      remPadArr(b_BattConfigPerIna_data, b_BattConfigPerIna_size, tmp_data,
                tmp_size);
      b_k_ina219 = (((double)k_ina219 + 1.0) - 1.0) * (double)prmBrd_Nbat;
      b_Vbat_size[0] = tmp_size[0];
      b_Vbat_size[1] = tmp_size[1];
      loop_ub = tmp_size[0] * tmp_size[1];
      for (i = 0; i < loop_ub; i++) {
        b_BattConfigPerIna_data[i] =
            Vbat_data[(int)(tmp_data[i] + b_k_ina219) - 1];
      }
      b_sum(b_BattConfigPerIna_data, b_Vbat_size, b_tmp_data, tmp_size);
      sumV += b_tmp_data[0];
    }
    /*  if BattConfigPerIna{k_ina219}(1) ~= bypass &&
     * BattConfigPerIna{k_ina219}(1) ~= disconnect */
    /*      sumV = sumV + sum(Vbat(BattConfigPerIna{k_ina219} +
     * (k_ina219-1)*Nbat)); */
    /*  end */
  }
  emxInit_boolean_T(&x);
  i = x->size[0] * x->size[1];
  x->size[0] = 1;
  x->size[1] = Sel_size[1];
  emxEnsureCapacity_boolean_T(x, i);
  x_data = x->data;
  loop_ub = Sel_size[1];
  for (i = 0; i < loop_ub; i++) {
    x_data[i] = (Sel_data[i] == 0.0);
  }
  b = b_any(x);
  if (b) {
    for (i = 0; i < Vbat_size; i++) {
      ind2out_data[i] = (Vbat_data[i] < CutOffDisV);
    }
    UnOvVflag = ((prmSeq.VthUnDis > sumV) && (VthFlag > 0));
  } else {
    b_Sel_size[0] = 1;
    b_Sel_size[1] = Sel_size[1];
    loop_ub = Sel_size[1];
    for (i = 0; i < loop_ub; i++) {
      d = Sel_data[i];
      b_Sel_data[i] = ((d >= 1.0) && (d < 2.0));
    }
    c_Sel_data.data = &b_Sel_data[0];
    c_Sel_data.size = &b_Sel_size[0];
    c_Sel_data.allocatedSize = 2;
    c_Sel_data.numDimensions = 2;
    c_Sel_data.canFreeData = false;
    if (b_any(&c_Sel_data)) {
      for (i = 0; i < Vbat_size; i++) {
        ind2out_data[i] = (Vbat_data[i] > CutOffChrV);
      }
      UnOvVflag = ((prmSeq.VthOvChr < sumV) && (VthFlag > 0));
    } else {
      if (Vbat_size - 1 >= 0) {
        memset(&ind2out_data[0], 0,
               (unsigned int)Vbat_size * sizeof(boolean_T));
      }
      UnOvVflag = false;
    }
  }
  i = x->size[0] * x->size[1];
  x->size[0] = 1;
  x->size[1] = Sel_size[1];
  emxEnsureCapacity_boolean_T(x, i);
  x_data = x->data;
  loop_ub = Sel_size[1];
  for (i = 0; i < loop_ub; i++) {
    d = Sel_data[i];
    x_data[i] = ((d == prmBrd_spi_bypass) || (d == prmBrd_spi_disconnect));
  }
  y = true;
  b_loop_ub = 1;
  exitg1 = false;
  while ((!exitg1) && (b_loop_ub <= x->size[1])) {
    if (!x_data[b_loop_ub - 1]) {
      y = false;
      exitg1 = true;
    } else {
      b_loop_ub++;
    }
  }
  emxFree_boolean_T(&x);
  if (!y) {
    b_Sel_size[0] = 1;
    if (tLastToggle_size[1] == 1) {
      loop_ub = t_size[1];
    } else {
      loop_ub = tLastToggle_size[1];
    }
    b_Sel_size[1] = loop_ub;
    b_loop_ub = (t_size[1] != 1);
    nx = (tLastToggle_size[1] != 1);
    for (i = 0; i < loop_ub; i++) {
      b_Sel_data[i] = (prmCnfg_Ttoggle <
                       t_data[i * b_loop_ub] - (float)tLastToggle_data[i * nx]);
    }
    d_Sel_data.data = &b_Sel_data[0];
    d_Sel_data.size = &b_Sel_size[0];
    d_Sel_data.allocatedSize = 2;
    d_Sel_data.numDimensions = 2;
    d_Sel_data.canFreeData = false;
    emxInit_real_T(&C, 2);
    C_data = C->data;
    emxInit_real_T(&Vc, 1);
    emxInit_real_T(&VbatC, 2);
    emxInit_real_T(&r, 2);
    emxInit_uint16_T(&b_y);
    y_data = b_y->data;
    emxInit_real32_T(&c_y);
    emxInit_real32_T(&b_x);
    guard1 = false;
    if (b_any(&d_Sel_data)) {
      guard1 = true;
    } else {
      b_loop_ub = BattConfigAct_size[0] * BattConfigAct_size[1];
      if (b_loop_ub == 1) {
        loop_ub = Vbat_size;
      } else {
        loop_ub = b_loop_ub;
      }
      nx = (Vbat_size != 1);
      b_loop_ub = (b_loop_ub != 1);
      for (i = 0; i < loop_ub; i++) {
        b_ind2out_data[i] =
            (ind2out_data[i * nx] && BattConfigAct_data[i * b_loop_ub]);
      }
      if (any(b_ind2out_data, loop_ub) || UnOvVflag) {
        guard1 = true;
      }
    }
    if (guard1) {
      loop_ub = BattConfigAct_size[0] * BattConfigAct_size[1];
      if (loop_ub == Vbat_size) {
        for (i = 0; i < loop_ub; i++) {
          batAct_data[i] = (BattConfigAct_data[i] && (!ind2out_data[i]));
        }
      } else {
        loop_ub =
            binary_expand_op_11(batAct_data, BattConfigAct_data,
                                BattConfigAct_size, ind2out_data, &Vbat_size);
      }
      if (b) {
        if (Vbat_size == loop_ub) {
          nx = Vbat_size;
          for (i = 0; i < Vbat_size; i++) {
            b_y_data[i] = Vbat_data[i] * (double)batAct_data[i];
          }
        } else {
          nx = binary_expand_op_9(b_y_data, Vbat_data, &Vbat_size, batAct_data,
                                  &loop_ub);
        }
        c_sort(b_y_data, &nx, iidx_data);
        b_loop_ub = nx - 1;
        trueCount = 0;
        nx = 0;
        for (b_i = 0; b_i <= b_loop_ub; b_i++) {
          if (b_y_data[b_i] > 0.0) {
            trueCount++;
            c_tmp_data[nx] = (signed char)b_i;
            nx++;
          }
        }
        for (i = 0; i < trueCount; i++) {
          sVbat0_data[i] = b_y_data[c_tmp_data[i]];
        }
        useConstantDim(sVbat0_data, &trueCount, nonSingletonDim(trueCount));
        switch (VthFlag) {
        case 1:
          for (i = 0; i < trueCount; i++) {
            b_ind2out_data[i] = (sVbat0_data[i] > prmSeq.VthDis);
          }
          b_loop_ub =
              g_eml_find(b_ind2out_data, trueCount, (int *)&findId0_data);
          if (b_loop_ub == 0) {
            for (i = 0; i < trueCount; i++) {
              b_ind2out_data[i] = (sVbat0_data[i] > prmSeq.VthUnDis);
            }
            g_eml_find(b_ind2out_data, trueCount, (int *)&findId0_data);
          }
          for (i = 0; i < findId0_data; i++) {
            ind2Stay_data[i] = iidx_data[c_tmp_data[i]];
          }
          if (sVbat0_data[findId0_data - 1] > prmSeq.VthOvDis) {
            UnOvVflag = false;
            b_i = 0;
            exitg1 = false;
            while ((!exitg1) && (b_i < 3)) {
              b_k_ina219 = (double)findId0_data + (double)b_i;
              if (b_k_ina219 + 4.0 <= trueCount) {
                i = (int)(b_k_ina219 + 4.0);
              } else {
                i = trueCount;
              }
              if (i < 1) {
                loop_ub = 0;
              } else {
                loop_ub = i;
              }
              for (i = 0; i < loop_ub; i++) {
                b_y_data[i] = iidx_data[c_tmp_data[i]];
              }
              nchoosek(b_y_data, loop_ub, b_k_ina219, C);
              C_data = C->data;
              i = VbatC->size[0] * VbatC->size[1];
              VbatC->size[0] = C->size[0];
              VbatC->size[1] = C->size[1];
              emxEnsureCapacity_real_T(VbatC, i);
              VbatC_data = VbatC->data;
              loop_ub = C->size[0] * C->size[1];
              for (i = 0; i < loop_ub; i++) {
                VbatC_data[i] = Vbat_data[(int)C_data[i] - 1];
              }
              b_useConstantDim(VbatC);
              VbatC_data = VbatC->data;
              b_loop_ub = 0;
              exitg2 = false;
              while ((!exitg2) && (b_loop_ub <= C->size[0] - 1)) {
                k_nc = b_loop_ub;
                VbatC_size[0] = 1;
                VbatC_size[1] = VbatC->size[1];
                loop_ub = VbatC->size[1];
                for (i = 0; i < loop_ub; i++) {
                  b_VbatC_data[i] =
                      ((VbatC_data[b_loop_ub + VbatC->size[0] * i] > VthDis) &&
                       (VbatC_data[b_loop_ub + VbatC->size[0] * i] < VthOvDis));
                }
                f_eml_find(b_VbatC_data, VbatC_size, (int *)&k_ina219,
                           tmp_size);
                if (tmp_size[1] != 0) {
                  UnOvVflag = true;
                  exitg2 = true;
                } else {
                  b_loop_ub++;
                }
              }
              if (UnOvVflag) {
                exitg1 = true;
              } else {
                b_i++;
              }
            }
            findId0_data = C->size[1];
            loop_ub = C->size[1];
            for (i = 0; i < loop_ub; i++) {
              ind2Stay_data[i] = (int)C_data[k_nc + C->size[0] * i];
            }
          }
          break;
        case 2:
          /* bestfit */
          for (i = 0; i < trueCount; i++) {
            b_ind2out_data[i] = (sVbat0_data[i] > prmSeq.VthDis);
          }
          b_loop_ub =
              g_eml_find(b_ind2out_data, trueCount, (int *)&findId0_data);
          if (b_loop_ub == 0) {
            for (i = 0; i < trueCount; i++) {
              b_ind2out_data[i] = (sVbat0_data[i] > prmSeq.VthUnDis);
            }
            g_eml_find(b_ind2out_data, trueCount, (int *)&findId0_data);
          }
          /*                          if csVbat(findId)>VthOvDis */
          b_i = 0;
          exitg1 = false;
          while ((!exitg1) && (b_i < 3)) {
            b_k_ina219 = (double)findId0_data + (double)b_i;
            if (b_k_ina219 + 6.0 <= trueCount) {
              i = (int)(b_k_ina219 + 6.0);
            } else {
              i = trueCount;
            }
            if (i < 1) {
              loop_ub = 0;
            } else {
              loop_ub = i;
            }
            for (i = 0; i < loop_ub; i++) {
              b_y_data[i] = iidx_data[c_tmp_data[i]];
            }
            nchoosek(b_y_data, loop_ub, b_k_ina219, C);
            C_data = C->data;
            /*                              VbatC = Vbat(C); */
            i = VbatC->size[0] * VbatC->size[1];
            VbatC->size[0] = C->size[0];
            VbatC->size[1] = C->size[1];
            emxEnsureCapacity_real_T(VbatC, i);
            VbatC_data = VbatC->data;
            loop_ub = C->size[0] * C->size[1];
            for (i = 0; i < loop_ub; i++) {
              VbatC_data[i] = Vbat_data[(int)C_data[i] - 1];
            }
            combineVectorElements(VbatC, Vc);
            VbatC_data = Vc->data;
            i = b_x->size[0];
            b_x->size[0] = Vc->size[0];
            emxEnsureCapacity_real32_T(b_x, i);
            b_x_data = b_x->data;
            loop_ub = Vc->size[0];
            for (i = 0; i < loop_ub; i++) {
              b_x_data[i] = (float)VbatC_data[i] - VthDis;
            }
            nx = b_x->size[0];
            i = c_y->size[0];
            c_y->size[0] = b_x->size[0];
            emxEnsureCapacity_real32_T(c_y, i);
            c_y_data = c_y->data;
            for (b_loop_ub = 0; b_loop_ub < nx; b_loop_ub++) {
              c_y_data[b_loop_ub] = (float)fabs(b_x_data[b_loop_ub]);
            }
            b_minimum(c_y, &nx);
            k_nc = nx - 1;
            /*                                  for k_nc = 1:NC */
            /*                                  idC =
             * find(Vc(k_nc,:)>VthDis&Vc(k_nc,:)<VthOvDis,1,'first'); */
            d = VbatC_data[nx - 1];
            if ((d > VthDis) && (d < VthOvDis) && (d > prmSeq.VthUnDis)) {
              exitg1 = true;
            } else {
              /*                                  end */
              b_i++;
            }
          }
          findId0_data = C->size[1];
          loop_ub = C->size[1];
          for (i = 0; i < loop_ub; i++) {
            ind2Stay_data[i] = (int)C_data[k_nc + C->size[0] * i];
          }
          /*                          end */
          break;
        case 0:
          /* toggle */
          d = rt_roundd_snf((double)prmBrd_Nbat * Nina219 -
                            (double)prmCnfg_NtoggleDrop);
          if (d < 32768.0) {
            if (d >= -32768.0) {
              d_y = (short)d;
            } else {
              d_y = MIN_int16_T;
            }
          } else if (d >= 32768.0) {
            d_y = MAX_int16_T;
          } else {
            d_y = 0;
          }
          if (trueCount <= d_y) {
            d_y = (short)trueCount;
          }
          if (d_y < 1) {
            findId0_data = 0;
          } else {
            findId0_data = d_y;
          }
          for (i = 0; i < findId0_data; i++) {
            ind2Stay_data[i] = iidx_data[c_tmp_data[i]];
          }
          break;
        default:
          findId0_data = 1;
          ind2Stay_data[0] = 0;
          break;
        }
      } else {
        b_Sel_size[0] = 1;
        b_Sel_size[1] = Sel_size[1];
        b_loop_ub = Sel_size[1];
        for (i = 0; i < b_loop_ub; i++) {
          d = Sel_data[i];
          b_Sel_data[i] = ((d >= 1.0) && (d <= 2.0));
        }
        e_Sel_data.data = &b_Sel_data[0];
        e_Sel_data.size = &b_Sel_size[0];
        e_Sel_data.allocatedSize = 2;
        e_Sel_data.numDimensions = 2;
        e_Sel_data.canFreeData = false;
        if (b_any(&e_Sel_data)) {
          if (Vbat_size == loop_ub) {
            nx = Vbat_size;
            for (i = 0; i < Vbat_size; i++) {
              b_y_data[i] = Vbat_data[i] * (double)batAct_data[i];
            }
          } else {
            nx = binary_expand_op_9(b_y_data, Vbat_data, &Vbat_size,
                                    batAct_data, &loop_ub);
          }
          d_sort(b_y_data, &nx, iidx_data);
          b_loop_ub = nx - 1;
          trueCount = 0;
          nx = 0;
          for (b_i = 0; b_i <= b_loop_ub; b_i++) {
            if (b_y_data[b_i] > 0.0) {
              trueCount++;
              c_tmp_data[nx] = (signed char)b_i;
              nx++;
            }
          }
          for (i = 0; i < trueCount; i++) {
            sVbat0_data[i] = b_y_data[c_tmp_data[i]];
          }
          useConstantDim(sVbat0_data, &trueCount, nonSingletonDim(trueCount));
          switch (VthFlag) {
          case 1:
            for (i = 0; i < trueCount; i++) {
              d = sVbat0_data[i];
              UnOvVflag = (d > prmSeq.VthUnChr);
              b_ind2out_data[i] = UnOvVflag;
              e_tmp_data[i] = (UnOvVflag && (d < prmSeq.VthChr));
            }
            b_loop_ub = g_eml_find(e_tmp_data, trueCount, (int *)&findId0_data);
            if (b_loop_ub == 0) {
              for (i = 0; i < trueCount; i++) {
                b_ind2out_data[i] =
                    (b_ind2out_data[i] && (sVbat0_data[i] < prmSeq.VthOvChr));
              }
              g_eml_find(b_ind2out_data, trueCount, (int *)&findId0_data);
            }
            /*                      findId0 = find(csVbat<VthChr,1,'last'); */
            /*                      findIdUn = find(csVbat>VthUnChr,1,'first');
             */
            /*                      findId = max(findId,findIdUn); */
            for (i = 0; i < findId0_data; i++) {
              ind2Stay_data[i] = iidx_data[c_tmp_data[i]];
            }
            break;
          case 2:
            for (i = 0; i < trueCount; i++) {
              b_ind2out_data[i] = (sVbat0_data[i] < prmSeq.VthChr);
            }
            /*                      findId0 = find(csVbat<VthChr,1,'last'); */
            /*                      findIdUn = find(csVbat>VthUnChr,1,'first');
             */
            /*                      findId = max(findId,findIdUn); */
            d_eml_find(b_ind2out_data, trueCount, (int *)&k_ina219);
            /*                          if ~isempty(findId) */
            /*                              firstBigUnder =
             * find(csVbat>VthUnChr,1,'first'); */
            /*                              lastThOver  =
             * find(csVbat<VthChr,1,'last'); */
            /*                              lastLessOver  =
             * find(csVbat<VthOvChr,1,'last'); */
            /*                              if isempty(firstBigUnder) */
            /*                                  firstBigUnder = lastThOver; */
            /*                              end */
            /*                              findId =
             * min(lastThOver,firstBigUnder); */
            /*                              if csVbat(findId)<VthUnChr */
            b_i = 0;
            exitg1 = false;
            while ((!exitg1) && (b_i < 3)) {
              b_k_ina219 = (double)k_ina219 + (double)b_i;
              if (b_k_ina219 + 6.0 <= trueCount) {
                i = (int)(b_k_ina219 + 6.0);
              } else {
                i = trueCount;
              }
              if (i < 1) {
                loop_ub = 0;
              } else {
                loop_ub = i;
              }
              for (i = 0; i < loop_ub; i++) {
                b_y_data[i] = iidx_data[c_tmp_data[i]];
              }
              nchoosek(b_y_data, loop_ub, b_k_ina219, C);
              C_data = C->data;
              i = VbatC->size[0] * VbatC->size[1];
              VbatC->size[0] = C->size[0];
              VbatC->size[1] = C->size[1];
              emxEnsureCapacity_real_T(VbatC, i);
              VbatC_data = VbatC->data;
              loop_ub = C->size[0] * C->size[1];
              for (i = 0; i < loop_ub; i++) {
                VbatC_data[i] = Vbat_data[(int)C_data[i] - 1];
              }
              combineVectorElements(VbatC, Vc);
              VbatC_data = Vc->data;
              i = b_x->size[0];
              b_x->size[0] = Vc->size[0];
              emxEnsureCapacity_real32_T(b_x, i);
              b_x_data = b_x->data;
              loop_ub = Vc->size[0];
              for (i = 0; i < loop_ub; i++) {
                b_x_data[i] = (float)VbatC_data[i] - prmSeq.VthChr;
              }
              nx = b_x->size[0];
              i = c_y->size[0];
              c_y->size[0] = b_x->size[0];
              emxEnsureCapacity_real32_T(c_y, i);
              c_y_data = c_y->data;
              for (b_loop_ub = 0; b_loop_ub < nx; b_loop_ub++) {
                c_y_data[b_loop_ub] = (float)fabs(b_x_data[b_loop_ub]);
              }
              b_minimum(c_y, &nx);
              k_nc = nx - 1;
              /*                                      for k_nc = 1:NC */
              /*                                          idC =
               * (Vc(k_nc,:)<VthChr&Vc(k_nc,:)>VthUnChr,1); */
              d = VbatC_data[nx - 1];
              if ((d > prmSeq.VthUnChr) && (d < prmSeq.VthOvChr)) {
                exitg1 = true;
              } else {
                /*                                      end */
                b_i++;
              }
            }
            findId0_data = C->size[1];
            loop_ub = C->size[1];
            for (i = 0; i < loop_ub; i++) {
              ind2Stay_data[i] = (int)C_data[k_nc + C->size[0] * i];
            }
            /*                              end */
            /*                          end */
            break;
          case 0:
            /* toggle */
            d = rt_roundd_snf((double)prmBrd_Nbat * Nina219 -
                              (double)prmCnfg_NtoggleDrop);
            if (d < 32768.0) {
              if (d >= -32768.0) {
                d_y = (short)d;
              } else {
                d_y = MIN_int16_T;
              }
            } else if (d >= 32768.0) {
              d_y = MAX_int16_T;
            } else {
              d_y = 0;
            }
            if (trueCount <= d_y) {
              d_y = (short)trueCount;
            }
            if (d_y < 1) {
              findId0_data = 0;
            } else {
              findId0_data = d_y;
            }
            for (i = 0; i < findId0_data; i++) {
              ind2Stay_data[i] = iidx_data[c_tmp_data[i]];
            }
            break;
          default:
            findId0_data = 1;
            ind2Stay_data[0] = 0;
            break;
          }
        } else {
          findId0_data = 1;
          ind2Stay_data[0] = 0;
        }
      }
      if ((int)Nina219 - 1 >= 0) {
        if (prmBrd_Nbat < 1) {
          b_y->size[0] = 1;
          b_y->size[1] = 0;
        } else {
          i = b_y->size[0] * b_y->size[1];
          b_y->size[0] = 1;
          b_y->size[1] = prmBrd_Nbat;
          emxEnsureCapacity_uint16_T(b_y, i);
          y_data = b_y->data;
          loop_ub = prmBrd_Nbat - 1;
          for (i = 0; i <= loop_ub; i++) {
            y_data[i] = (unsigned short)((unsigned int)i + 1U);
          }
        }
        d_loop_ub = b_y->size[1];
        end = findId0_data - 1;
      }
      if (loop_ub_tmp - 1 >= 0) {
        c_loop_ub = BattConfigAct_size[0];
        b_BattConfigPerIna_size[0] = 1;
        b_BattConfigPerIna_size[1] = BattConfigPerIna_size[1];
        e_loop_ub = BattConfigPerIna_size[2];
        b_BattConfigPerIna_size[2] = e_loop_ub;
      }
      for (k_ina219 = 0; k_ina219 < loop_ub_tmp; k_ina219++) {
        /*  BattConfigAct{k_ina219} = batAct([1:Nbat]+(k_ina219-1)*Nbat); */
        b_k_ina219 = (((double)k_ina219 + 1.0) - 1.0) * (double)prmBrd_Nbat;
        for (i = 0; i < d_loop_ub; i++) {
          d_tmp_data[i] =
              batAct_data[(int)((double)y_data[i] + b_k_ina219) - 1];
        }
        for (i = 0; i < c_loop_ub; i++) {
          BattConfigAct_data[i + BattConfigAct_size[0] * k_ina219] =
              d_tmp_data[i];
        }
        trueCount = 0;
        nx = 0;
        for (b_i = 0; b_i <= end; b_i++) {
          i = ind2Stay_data[b_i];
          d = (double)prmBrd_Nbat + b_k_ina219;
          if ((b_k_ina219 + 1.0 <= i) && (i <= d)) {
            trueCount++;
            BattConfigPerInaNew_data[nx] = i;
            nx++;
          }
        }
        e_sort(BattConfigPerInaNew_data, &trueCount);
        for (i = 0; i < trueCount; i++) {
          BattConfigPerInaNew_data[i] -= b_k_ina219;
        }
        for (i = 0; i < e_loop_ub; i++) {
          loop_ub = BattConfigPerIna_size[1];
          for (findId0_data = 0; findId0_data < loop_ub; findId0_data++) {
            b_BattConfigPerIna_data[findId0_data +
                                    b_BattConfigPerIna_size[1] * i] =
                BattConfigPerIna_data[(k_ina219 + BattConfigPerIna_size[0] *
                                                      findId0_data) +
                                      BattConfigPerIna_size[0] *
                                          BattConfigPerIna_size[1] * i];
          }
        }
        remPadArr(b_BattConfigPerIna_data, b_BattConfigPerIna_size, tmp_data,
                  tmp_size);
        if (!isequal(tmp_data, tmp_size, BattConfigPerInaNew_data, trueCount)) {
          if (trueCount != 0) {
            padArr(BattConfigPerInaNew_data, trueCount, prmBrd_Nbat,
                   prmBrd_Nbat, r);
            VbatC_data = r->data;
            b_loop_ub = BattConfigPerIna_size[1];
            nx = BattConfigPerIna_size[2];
            for (i = 0; i < nx; i++) {
              for (findId0_data = 0; findId0_data < b_loop_ub; findId0_data++) {
                BattConfigPerIna_data[(k_ina219 + BattConfigPerIna_size[0] *
                                                      findId0_data) +
                                      BattConfigPerIna_size[0] *
                                          BattConfigPerIna_size[1] * i] =
                    VbatC_data[findId0_data + b_loop_ub * i];
              }
            }
          } else {
            b_padArr(prmBrd_spi_bypass, prmBrd_Nbat, prmBrd_Nbat, r);
            VbatC_data = r->data;
            b_loop_ub = BattConfigPerIna_size[1];
            nx = BattConfigPerIna_size[2];
            for (i = 0; i < nx; i++) {
              for (findId0_data = 0; findId0_data < b_loop_ub; findId0_data++) {
                BattConfigPerIna_data[(k_ina219 + BattConfigPerIna_size[0] *
                                                      findId0_data) +
                                      BattConfigPerIna_size[0] *
                                          BattConfigPerIna_size[1] * i] =
                    VbatC_data[findId0_data + b_loop_ub * i];
              }
            }
          }
          changeConfigFlag_data[k_ina219] = true;
          tLastToggle_data[k_ina219] = t_data[0];
        }
      }
      sumNotBypass = 0U;
      for (k_ina219 = 0; k_ina219 < loop_ub_tmp; k_ina219++) {
        if (BattConfigPerIna_data[k_ina219] != prmBrd_spi_bypass) {
          /* >0 */
          sumNotBypass++;
        }
      }
      if ((double)sumNotBypass < prmCnfg_minLenIna219) {
        /* ||SwitchToggleFlage% goto charge */
        if (b) {
          b_loop_ub = Sel_size[1];
          Sel_size[0] = 1;
          for (i = 0; i < b_loop_ub; i++) {
            Sel_data[i] = 1.0;
          }
          switchFlag = true;
        } else {
          b_Sel_size[0] = 1;
          b_Sel_size[1] = Sel_size[1];
          loop_ub = Sel_size[1];
          for (i = 0; i < loop_ub; i++) {
            b_Sel_data[i] = (Sel_data[i] == 1.0);
          }
          f_Sel_data.data = &b_Sel_data[0];
          f_Sel_data.size = &b_Sel_size[0];
          f_Sel_data.allocatedSize = 2;
          f_Sel_data.numDimensions = 2;
          f_Sel_data.canFreeData = false;
          if (b_any(&f_Sel_data)) {
            b_loop_ub = Sel_size[1];
            Sel_size[0] = 1;
            for (i = 0; i < b_loop_ub; i++) {
              Sel_data[i] = 1.5;
            }
            switchFlag = true;
          } else {
            b_Sel_size[0] = 1;
            b_Sel_size[1] = Sel_size[1];
            loop_ub = Sel_size[1];
            for (i = 0; i < loop_ub; i++) {
              b_Sel_data[i] = (Sel_data[i] == 1.5);
            }
            g_Sel_data.data = &b_Sel_data[0];
            g_Sel_data.size = &b_Sel_size[0];
            g_Sel_data.allocatedSize = 2;
            g_Sel_data.numDimensions = 2;
            g_Sel_data.canFreeData = false;
            if (b_any(&g_Sel_data)) {
              b_loop_ub = Sel_size[1];
              Sel_size[0] = 1;
              for (i = 0; i < b_loop_ub; i++) {
                Sel_data[i] = 1.6;
              }
              /* 1.6 */
              switchFlag = true;
            } else {
              b_Sel_size[0] = 1;
              b_Sel_size[1] = Sel_size[1];
              loop_ub = Sel_size[1];
              for (i = 0; i < loop_ub; i++) {
                b_Sel_data[i] = (Sel_data[i] == 1.6);
              }
              h_Sel_data.data = &b_Sel_data[0];
              h_Sel_data.size = &b_Sel_size[0];
              h_Sel_data.allocatedSize = 2;
              h_Sel_data.numDimensions = 2;
              h_Sel_data.canFreeData = false;
              if (b_any(&h_Sel_data)) {
                b_loop_ub = Sel_size[1];
                Sel_size[0] = 1;
                if (b_loop_ub - 1 >= 0) {
                  memset(&Sel_data[0], 0,
                         (unsigned int)b_loop_ub * sizeof(double));
                }
                switchFlag = true;
              }
            }
          }
        }
        b_loop_ub = (int)Nina219;
        changeConfigFlag_size[0] = 1;
        changeConfigFlag_size[1] = (int)Nina219;
        if (b_loop_ub - 1 >= 0) {
          memset(&changeConfigFlag_data[0], 0,
                 (unsigned int)b_loop_ub * sizeof(boolean_T));
        }
        tLastToggle_size[0] = 1;
        tLastToggle_size[1] = t_size[1];
        loop_ub = t_size[1];
        for (i = 0; i < loop_ub; i++) {
          tLastToggle_data[i] = t_data[i];
        }
        for (k_ina219 = 0; k_ina219 < loop_ub_tmp; k_ina219++) {
          loop_ub = BattConfigAct_size[0];
          for (i = 0; i < loop_ub; i++) {
            BattConfigAct_data[i + BattConfigAct_size[0] * k_ina219] = true;
          }
          /* [1:Nbat]'+(k_ina219-1)*Nbat; */
        }
      }
    }
    emxFree_real32_T(&b_x);
    emxFree_real32_T(&c_y);
    emxFree_uint16_T(&b_y);
    emxFree_real_T(&r);
    emxFree_real_T(&VbatC);
    emxFree_real_T(&Vc);
    emxFree_real_T(&C);
  }
  return switchFlag;
}

/*
 * Arguments    : double Sel_data[]
 *                int Sel_size[2]
 *                float CutOffChrV
 *                float CutOffDisV
 *                double BattConfigPerIna_data[]
 *                const int BattConfigPerIna_size[3]
 *                const double Vbat_data[]
 *                int Vbat_size
 *                double Nina219
 *                short prmBrd_Nbat
 *                unsigned char prmBrd_spi_disconnect
 *                unsigned char prmBrd_spi_bypass
 *                float prmCnfg_Ttoggle
 *                short prmCnfg_NtoggleDrop
 *                short prmCnfg_minLenIna219
 *                const struct4_T prmSeq
 *                signed char VthFlag
 *                double t
 *                double tLastToggle_data[]
 *                int tLastToggle_size[2]
 *                boolean_T BattConfigAct_data[]
 *                const int BattConfigAct_size[2]
 *                boolean_T changeConfigFlag_data[]
 *                int changeConfigFlag_size[2]
 * Return Type  : boolean_T
 */
boolean_T d_Esp32StepSwitchToggleCombAll_(
    double Sel_data[], int Sel_size[2], float CutOffChrV, float CutOffDisV,
    double BattConfigPerIna_data[], const int BattConfigPerIna_size[3],
    const double Vbat_data[], int Vbat_size, double Nina219, short prmBrd_Nbat,
    unsigned char prmBrd_spi_disconnect, unsigned char prmBrd_spi_bypass,
    float prmCnfg_Ttoggle, short prmCnfg_NtoggleDrop,
    short prmCnfg_minLenIna219, const struct4_T prmSeq, signed char VthFlag,
    double t, double tLastToggle_data[], int tLastToggle_size[2],
    boolean_T BattConfigAct_data[], const int BattConfigAct_size[2],
    boolean_T changeConfigFlag_data[], int changeConfigFlag_size[2])
{
  emxArray_boolean_T c_Sel_data;
  emxArray_boolean_T d_Sel_data;
  emxArray_boolean_T e_Sel_data;
  emxArray_boolean_T f_Sel_data;
  emxArray_boolean_T g_Sel_data;
  emxArray_boolean_T h_Sel_data;
  emxArray_boolean_T *x;
  emxArray_real32_T *b_x;
  emxArray_real32_T *c_y;
  emxArray_real_T *C;
  emxArray_real_T *VbatC;
  emxArray_real_T *Vc;
  emxArray_real_T *r;
  emxArray_uint16_T *b_y;
  double b_BattConfigPerIna_data[256];
  double tmp_data[256];
  double BattConfigPerInaNew_data[34];
  double b_y_data[32];
  double sVbat0_data[32];
  double b_tmp_data[16];
  double b_k_ina219;
  double d;
  double sumV;
  double *C_data;
  double *VbatC_data;
  float VthDis;
  float VthOvDis;
  float *b_x_data;
  float *c_y_data;
  int ind2Stay_data[34];
  int iidx_data[32];
  int b_BattConfigPerIna_size[3];
  int VbatC_size[2];
  int b_Sel_size[2];
  int b_Vbat_size[2];
  int tmp_size[2];
  int b_i;
  int b_loop_ub;
  int c_loop_ub;
  int d_loop_ub;
  int end;
  int findId0_data;
  int i;
  int k_ina219;
  int k_nc;
  int loop_ub;
  int loop_ub_tmp;
  int nx;
  int stride_0_0;
  unsigned int sumNotBypass;
  int trueCount;
  short d_y;
  unsigned short *y_data;
  signed char c_tmp_data[32];
  boolean_T d_tmp_data[32767];
  boolean_T b_VbatC_data[34];
  boolean_T b_ind2out_data[32];
  boolean_T batAct_data[32];
  boolean_T e_tmp_data[32];
  boolean_T ind2out_data[16];
  boolean_T b_Sel_data[2];
  boolean_T UnOvVflag;
  boolean_T b;
  boolean_T exitg1;
  boolean_T exitg2;
  boolean_T guard1;
  boolean_T switchFlag;
  boolean_T y;
  boolean_T *x_data;
  VthDis = prmSeq.VthDis;
  /*  -1 - NA */
  /*  VthFlag         = prmSeq.VthFlag; % true - Vth mode on */
  VthOvDis = prmSeq.VthOvDis;
  changeConfigFlag_size[0] = 1;
  loop_ub_tmp = (int)Nina219;
  changeConfigFlag_size[1] = (int)Nina219;
  switchFlag = false;
  /*  SwitchToggleFlage = false; */
  k_nc = 0;
  /*  1 */
  sumV = 0.0;
  for (k_ina219 = 0; k_ina219 < loop_ub_tmp; k_ina219++) {
    changeConfigFlag_data[k_ina219] = false;
    d = BattConfigPerIna_data[k_ina219];
    if ((d != prmBrd_spi_bypass) && (d != prmBrd_spi_disconnect)) {
      b_BattConfigPerIna_size[0] = 1;
      b_BattConfigPerIna_size[1] = BattConfigPerIna_size[1];
      loop_ub = BattConfigPerIna_size[2];
      b_BattConfigPerIna_size[2] = loop_ub;
      for (i = 0; i < loop_ub; i++) {
        nx = BattConfigPerIna_size[1];
        for (findId0_data = 0; findId0_data < nx; findId0_data++) {
          b_BattConfigPerIna_data[findId0_data + b_BattConfigPerIna_size[1] *
                                                     i] = BattConfigPerIna_data
              [(k_ina219 + BattConfigPerIna_size[0] * findId0_data) +
               BattConfigPerIna_size[0] * BattConfigPerIna_size[1] * i];
        }
      }
      remPadArr(b_BattConfigPerIna_data, b_BattConfigPerIna_size, tmp_data,
                tmp_size);
      b_k_ina219 = (((double)k_ina219 + 1.0) - 1.0) * (double)prmBrd_Nbat;
      b_Vbat_size[0] = tmp_size[0];
      b_Vbat_size[1] = tmp_size[1];
      loop_ub = tmp_size[0] * tmp_size[1];
      for (i = 0; i < loop_ub; i++) {
        b_BattConfigPerIna_data[i] =
            Vbat_data[(int)(tmp_data[i] + b_k_ina219) - 1];
      }
      b_sum(b_BattConfigPerIna_data, b_Vbat_size, b_tmp_data, tmp_size);
      sumV += b_tmp_data[0];
    }
    /*  if BattConfigPerIna{k_ina219}(1) ~= bypass &&
     * BattConfigPerIna{k_ina219}(1) ~= disconnect */
    /*      sumV = sumV + sum(Vbat(BattConfigPerIna{k_ina219} +
     * (k_ina219-1)*Nbat)); */
    /*  end */
  }
  emxInit_boolean_T(&x);
  i = x->size[0] * x->size[1];
  x->size[0] = 1;
  x->size[1] = Sel_size[1];
  emxEnsureCapacity_boolean_T(x, i);
  x_data = x->data;
  loop_ub = Sel_size[1];
  for (i = 0; i < loop_ub; i++) {
    x_data[i] = (Sel_data[i] == 0.0);
  }
  b = b_any(x);
  if (b) {
    for (i = 0; i < Vbat_size; i++) {
      ind2out_data[i] = (Vbat_data[i] < CutOffDisV);
    }
    UnOvVflag = ((prmSeq.VthUnDis > sumV) && (VthFlag > 0));
  } else {
    b_Sel_size[0] = 1;
    b_Sel_size[1] = Sel_size[1];
    loop_ub = Sel_size[1];
    for (i = 0; i < loop_ub; i++) {
      d = Sel_data[i];
      b_Sel_data[i] = ((d >= 1.0) && (d < 2.0));
    }
    c_Sel_data.data = &b_Sel_data[0];
    c_Sel_data.size = &b_Sel_size[0];
    c_Sel_data.allocatedSize = 2;
    c_Sel_data.numDimensions = 2;
    c_Sel_data.canFreeData = false;
    if (b_any(&c_Sel_data)) {
      for (i = 0; i < Vbat_size; i++) {
        ind2out_data[i] = (Vbat_data[i] > CutOffChrV);
      }
      UnOvVflag = ((prmSeq.VthOvChr < sumV) && (VthFlag > 0));
    } else {
      if (Vbat_size - 1 >= 0) {
        memset(&ind2out_data[0], 0,
               (unsigned int)Vbat_size * sizeof(boolean_T));
      }
      UnOvVflag = false;
    }
  }
  i = x->size[0] * x->size[1];
  x->size[0] = 1;
  x->size[1] = Sel_size[1];
  emxEnsureCapacity_boolean_T(x, i);
  x_data = x->data;
  loop_ub = Sel_size[1];
  for (i = 0; i < loop_ub; i++) {
    d = Sel_data[i];
    x_data[i] = ((d == prmBrd_spi_bypass) || (d == prmBrd_spi_disconnect));
  }
  y = true;
  nx = 1;
  exitg1 = false;
  while ((!exitg1) && (nx <= x->size[1])) {
    if (!x_data[nx - 1]) {
      y = false;
      exitg1 = true;
    } else {
      nx++;
    }
  }
  emxFree_boolean_T(&x);
  if (!y) {
    b_Sel_size[0] = 1;
    b_Sel_size[1] = tLastToggle_size[1];
    loop_ub = tLastToggle_size[1];
    for (i = 0; i < loop_ub; i++) {
      b_Sel_data[i] = (prmCnfg_Ttoggle < t - tLastToggle_data[i]);
    }
    d_Sel_data.data = &b_Sel_data[0];
    d_Sel_data.size = &b_Sel_size[0];
    d_Sel_data.allocatedSize = 2;
    d_Sel_data.numDimensions = 2;
    d_Sel_data.canFreeData = false;
    emxInit_real_T(&C, 2);
    C_data = C->data;
    emxInit_real_T(&Vc, 1);
    emxInit_real_T(&VbatC, 2);
    emxInit_real_T(&r, 2);
    emxInit_uint16_T(&b_y);
    y_data = b_y->data;
    emxInit_real32_T(&c_y);
    emxInit_real32_T(&b_x);
    guard1 = false;
    if (b_any(&d_Sel_data)) {
      guard1 = true;
    } else {
      nx = BattConfigAct_size[0] * BattConfigAct_size[1];
      if (nx == 1) {
        loop_ub = Vbat_size;
      } else {
        loop_ub = nx;
      }
      stride_0_0 = (Vbat_size != 1);
      nx = (nx != 1);
      for (i = 0; i < loop_ub; i++) {
        b_ind2out_data[i] =
            (ind2out_data[i * stride_0_0] && BattConfigAct_data[i * nx]);
      }
      if (any(b_ind2out_data, loop_ub) || UnOvVflag) {
        guard1 = true;
      }
    }
    if (guard1) {
      loop_ub = BattConfigAct_size[0] * BattConfigAct_size[1];
      if (loop_ub == Vbat_size) {
        for (i = 0; i < loop_ub; i++) {
          batAct_data[i] = (BattConfigAct_data[i] && (!ind2out_data[i]));
        }
      } else {
        loop_ub =
            binary_expand_op_11(batAct_data, BattConfigAct_data,
                                BattConfigAct_size, ind2out_data, &Vbat_size);
      }
      if (b) {
        if (Vbat_size == loop_ub) {
          stride_0_0 = Vbat_size;
          for (i = 0; i < Vbat_size; i++) {
            b_y_data[i] = Vbat_data[i] * (double)batAct_data[i];
          }
        } else {
          stride_0_0 = binary_expand_op_9(b_y_data, Vbat_data, &Vbat_size,
                                          batAct_data, &loop_ub);
        }
        c_sort(b_y_data, &stride_0_0, iidx_data);
        nx = stride_0_0 - 1;
        trueCount = 0;
        stride_0_0 = 0;
        for (b_i = 0; b_i <= nx; b_i++) {
          if (b_y_data[b_i] > 0.0) {
            trueCount++;
            c_tmp_data[stride_0_0] = (signed char)b_i;
            stride_0_0++;
          }
        }
        for (i = 0; i < trueCount; i++) {
          sVbat0_data[i] = b_y_data[c_tmp_data[i]];
        }
        useConstantDim(sVbat0_data, &trueCount, nonSingletonDim(trueCount));
        switch (VthFlag) {
        case 1:
          for (i = 0; i < trueCount; i++) {
            b_ind2out_data[i] = (sVbat0_data[i] > prmSeq.VthDis);
          }
          nx = g_eml_find(b_ind2out_data, trueCount, (int *)&findId0_data);
          if (nx == 0) {
            for (i = 0; i < trueCount; i++) {
              b_ind2out_data[i] = (sVbat0_data[i] > prmSeq.VthUnDis);
            }
            g_eml_find(b_ind2out_data, trueCount, (int *)&findId0_data);
          }
          for (i = 0; i < findId0_data; i++) {
            ind2Stay_data[i] = iidx_data[c_tmp_data[i]];
          }
          if (sVbat0_data[findId0_data - 1] > prmSeq.VthOvDis) {
            UnOvVflag = false;
            b_i = 0;
            exitg1 = false;
            while ((!exitg1) && (b_i < 3)) {
              b_k_ina219 = (double)findId0_data + (double)b_i;
              if (b_k_ina219 + 4.0 <= trueCount) {
                i = (int)(b_k_ina219 + 4.0);
              } else {
                i = trueCount;
              }
              if (i < 1) {
                loop_ub = 0;
              } else {
                loop_ub = i;
              }
              for (i = 0; i < loop_ub; i++) {
                b_y_data[i] = iidx_data[c_tmp_data[i]];
              }
              nchoosek(b_y_data, loop_ub, b_k_ina219, C);
              C_data = C->data;
              i = VbatC->size[0] * VbatC->size[1];
              VbatC->size[0] = C->size[0];
              VbatC->size[1] = C->size[1];
              emxEnsureCapacity_real_T(VbatC, i);
              VbatC_data = VbatC->data;
              loop_ub = C->size[0] * C->size[1];
              for (i = 0; i < loop_ub; i++) {
                VbatC_data[i] = Vbat_data[(int)C_data[i] - 1];
              }
              b_useConstantDim(VbatC);
              VbatC_data = VbatC->data;
              stride_0_0 = 0;
              exitg2 = false;
              while ((!exitg2) && (stride_0_0 <= C->size[0] - 1)) {
                k_nc = stride_0_0;
                VbatC_size[0] = 1;
                VbatC_size[1] = VbatC->size[1];
                loop_ub = VbatC->size[1];
                for (i = 0; i < loop_ub; i++) {
                  b_VbatC_data[i] =
                      ((VbatC_data[stride_0_0 + VbatC->size[0] * i] > VthDis) &&
                       (VbatC_data[stride_0_0 + VbatC->size[0] * i] <
                        VthOvDis));
                }
                f_eml_find(b_VbatC_data, VbatC_size, (int *)&k_ina219,
                           tmp_size);
                if (tmp_size[1] != 0) {
                  UnOvVflag = true;
                  exitg2 = true;
                } else {
                  stride_0_0++;
                }
              }
              if (UnOvVflag) {
                exitg1 = true;
              } else {
                b_i++;
              }
            }
            findId0_data = C->size[1];
            loop_ub = C->size[1];
            for (i = 0; i < loop_ub; i++) {
              ind2Stay_data[i] = (int)C_data[k_nc + C->size[0] * i];
            }
          }
          break;
        case 2:
          /* bestfit */
          for (i = 0; i < trueCount; i++) {
            b_ind2out_data[i] = (sVbat0_data[i] > prmSeq.VthDis);
          }
          nx = g_eml_find(b_ind2out_data, trueCount, (int *)&findId0_data);
          if (nx == 0) {
            for (i = 0; i < trueCount; i++) {
              b_ind2out_data[i] = (sVbat0_data[i] > prmSeq.VthUnDis);
            }
            g_eml_find(b_ind2out_data, trueCount, (int *)&findId0_data);
          }
          /*                          if csVbat(findId)>VthOvDis */
          b_i = 0;
          exitg1 = false;
          while ((!exitg1) && (b_i < 3)) {
            b_k_ina219 = (double)findId0_data + (double)b_i;
            if (b_k_ina219 + 6.0 <= trueCount) {
              i = (int)(b_k_ina219 + 6.0);
            } else {
              i = trueCount;
            }
            if (i < 1) {
              loop_ub = 0;
            } else {
              loop_ub = i;
            }
            for (i = 0; i < loop_ub; i++) {
              b_y_data[i] = iidx_data[c_tmp_data[i]];
            }
            nchoosek(b_y_data, loop_ub, b_k_ina219, C);
            C_data = C->data;
            /*                              VbatC = Vbat(C); */
            i = VbatC->size[0] * VbatC->size[1];
            VbatC->size[0] = C->size[0];
            VbatC->size[1] = C->size[1];
            emxEnsureCapacity_real_T(VbatC, i);
            VbatC_data = VbatC->data;
            loop_ub = C->size[0] * C->size[1];
            for (i = 0; i < loop_ub; i++) {
              VbatC_data[i] = Vbat_data[(int)C_data[i] - 1];
            }
            combineVectorElements(VbatC, Vc);
            VbatC_data = Vc->data;
            i = b_x->size[0];
            b_x->size[0] = Vc->size[0];
            emxEnsureCapacity_real32_T(b_x, i);
            b_x_data = b_x->data;
            loop_ub = Vc->size[0];
            for (i = 0; i < loop_ub; i++) {
              b_x_data[i] = (float)VbatC_data[i] - VthDis;
            }
            nx = b_x->size[0];
            i = c_y->size[0];
            c_y->size[0] = b_x->size[0];
            emxEnsureCapacity_real32_T(c_y, i);
            c_y_data = c_y->data;
            for (stride_0_0 = 0; stride_0_0 < nx; stride_0_0++) {
              c_y_data[stride_0_0] = (float)fabs(b_x_data[stride_0_0]);
            }
            b_minimum(c_y, &nx);
            k_nc = nx - 1;
            /*                                  for k_nc = 1:NC */
            /*                                  idC =
             * find(Vc(k_nc,:)>VthDis&Vc(k_nc,:)<VthOvDis,1,'first'); */
            d = VbatC_data[nx - 1];
            if ((d > VthDis) && (d < VthOvDis) && (d > prmSeq.VthUnDis)) {
              exitg1 = true;
            } else {
              /*                                  end */
              b_i++;
            }
          }
          findId0_data = C->size[1];
          loop_ub = C->size[1];
          for (i = 0; i < loop_ub; i++) {
            ind2Stay_data[i] = (int)C_data[k_nc + C->size[0] * i];
          }
          /*                          end */
          break;
        case 0:
          /* toggle */
          d = rt_roundd_snf((double)prmBrd_Nbat * Nina219 -
                            (double)prmCnfg_NtoggleDrop);
          if (d < 32768.0) {
            if (d >= -32768.0) {
              d_y = (short)d;
            } else {
              d_y = MIN_int16_T;
            }
          } else if (d >= 32768.0) {
            d_y = MAX_int16_T;
          } else {
            d_y = 0;
          }
          if (trueCount <= d_y) {
            d_y = (short)trueCount;
          }
          if (d_y < 1) {
            findId0_data = 0;
          } else {
            findId0_data = d_y;
          }
          for (i = 0; i < findId0_data; i++) {
            ind2Stay_data[i] = iidx_data[c_tmp_data[i]];
          }
          break;
        default:
          findId0_data = 1;
          ind2Stay_data[0] = 0;
          break;
        }
      } else {
        b_Sel_size[0] = 1;
        b_Sel_size[1] = Sel_size[1];
        nx = Sel_size[1];
        for (i = 0; i < nx; i++) {
          d = Sel_data[i];
          b_Sel_data[i] = ((d >= 1.0) && (d <= 2.0));
        }
        e_Sel_data.data = &b_Sel_data[0];
        e_Sel_data.size = &b_Sel_size[0];
        e_Sel_data.allocatedSize = 2;
        e_Sel_data.numDimensions = 2;
        e_Sel_data.canFreeData = false;
        if (b_any(&e_Sel_data)) {
          if (Vbat_size == loop_ub) {
            stride_0_0 = Vbat_size;
            for (i = 0; i < Vbat_size; i++) {
              b_y_data[i] = Vbat_data[i] * (double)batAct_data[i];
            }
          } else {
            stride_0_0 = binary_expand_op_9(b_y_data, Vbat_data, &Vbat_size,
                                            batAct_data, &loop_ub);
          }
          d_sort(b_y_data, &stride_0_0, iidx_data);
          nx = stride_0_0 - 1;
          trueCount = 0;
          stride_0_0 = 0;
          for (b_i = 0; b_i <= nx; b_i++) {
            if (b_y_data[b_i] > 0.0) {
              trueCount++;
              c_tmp_data[stride_0_0] = (signed char)b_i;
              stride_0_0++;
            }
          }
          for (i = 0; i < trueCount; i++) {
            sVbat0_data[i] = b_y_data[c_tmp_data[i]];
          }
          useConstantDim(sVbat0_data, &trueCount, nonSingletonDim(trueCount));
          switch (VthFlag) {
          case 1:
            for (i = 0; i < trueCount; i++) {
              d = sVbat0_data[i];
              UnOvVflag = (d > prmSeq.VthUnChr);
              b_ind2out_data[i] = UnOvVflag;
              e_tmp_data[i] = (UnOvVflag && (d < prmSeq.VthChr));
            }
            nx = g_eml_find(e_tmp_data, trueCount, (int *)&findId0_data);
            if (nx == 0) {
              for (i = 0; i < trueCount; i++) {
                b_ind2out_data[i] =
                    (b_ind2out_data[i] && (sVbat0_data[i] < prmSeq.VthOvChr));
              }
              g_eml_find(b_ind2out_data, trueCount, (int *)&findId0_data);
            }
            /*                      findId0 = find(csVbat<VthChr,1,'last'); */
            /*                      findIdUn = find(csVbat>VthUnChr,1,'first');
             */
            /*                      findId = max(findId,findIdUn); */
            for (i = 0; i < findId0_data; i++) {
              ind2Stay_data[i] = iidx_data[c_tmp_data[i]];
            }
            break;
          case 2:
            for (i = 0; i < trueCount; i++) {
              b_ind2out_data[i] = (sVbat0_data[i] < prmSeq.VthChr);
            }
            /*                      findId0 = find(csVbat<VthChr,1,'last'); */
            /*                      findIdUn = find(csVbat>VthUnChr,1,'first');
             */
            /*                      findId = max(findId,findIdUn); */
            d_eml_find(b_ind2out_data, trueCount, (int *)&k_ina219);
            /*                          if ~isempty(findId) */
            /*                              firstBigUnder =
             * find(csVbat>VthUnChr,1,'first'); */
            /*                              lastThOver  =
             * find(csVbat<VthChr,1,'last'); */
            /*                              lastLessOver  =
             * find(csVbat<VthOvChr,1,'last'); */
            /*                              if isempty(firstBigUnder) */
            /*                                  firstBigUnder = lastThOver; */
            /*                              end */
            /*                              findId =
             * min(lastThOver,firstBigUnder); */
            /*                              if csVbat(findId)<VthUnChr */
            b_i = 0;
            exitg1 = false;
            while ((!exitg1) && (b_i < 3)) {
              b_k_ina219 = (double)k_ina219 + (double)b_i;
              if (b_k_ina219 + 6.0 <= trueCount) {
                i = (int)(b_k_ina219 + 6.0);
              } else {
                i = trueCount;
              }
              if (i < 1) {
                loop_ub = 0;
              } else {
                loop_ub = i;
              }
              for (i = 0; i < loop_ub; i++) {
                b_y_data[i] = iidx_data[c_tmp_data[i]];
              }
              nchoosek(b_y_data, loop_ub, b_k_ina219, C);
              C_data = C->data;
              i = VbatC->size[0] * VbatC->size[1];
              VbatC->size[0] = C->size[0];
              VbatC->size[1] = C->size[1];
              emxEnsureCapacity_real_T(VbatC, i);
              VbatC_data = VbatC->data;
              loop_ub = C->size[0] * C->size[1];
              for (i = 0; i < loop_ub; i++) {
                VbatC_data[i] = Vbat_data[(int)C_data[i] - 1];
              }
              combineVectorElements(VbatC, Vc);
              VbatC_data = Vc->data;
              i = b_x->size[0];
              b_x->size[0] = Vc->size[0];
              emxEnsureCapacity_real32_T(b_x, i);
              b_x_data = b_x->data;
              loop_ub = Vc->size[0];
              for (i = 0; i < loop_ub; i++) {
                b_x_data[i] = (float)VbatC_data[i] - prmSeq.VthChr;
              }
              nx = b_x->size[0];
              i = c_y->size[0];
              c_y->size[0] = b_x->size[0];
              emxEnsureCapacity_real32_T(c_y, i);
              c_y_data = c_y->data;
              for (stride_0_0 = 0; stride_0_0 < nx; stride_0_0++) {
                c_y_data[stride_0_0] = (float)fabs(b_x_data[stride_0_0]);
              }
              b_minimum(c_y, &nx);
              k_nc = nx - 1;
              /*                                      for k_nc = 1:NC */
              /*                                          idC =
               * (Vc(k_nc,:)<VthChr&Vc(k_nc,:)>VthUnChr,1); */
              d = VbatC_data[nx - 1];
              if ((d > prmSeq.VthUnChr) && (d < prmSeq.VthOvChr)) {
                exitg1 = true;
              } else {
                /*                                      end */
                b_i++;
              }
            }
            findId0_data = C->size[1];
            loop_ub = C->size[1];
            for (i = 0; i < loop_ub; i++) {
              ind2Stay_data[i] = (int)C_data[k_nc + C->size[0] * i];
            }
            /*                              end */
            /*                          end */
            break;
          case 0:
            /* toggle */
            d = rt_roundd_snf((double)prmBrd_Nbat * Nina219 -
                              (double)prmCnfg_NtoggleDrop);
            if (d < 32768.0) {
              if (d >= -32768.0) {
                d_y = (short)d;
              } else {
                d_y = MIN_int16_T;
              }
            } else if (d >= 32768.0) {
              d_y = MAX_int16_T;
            } else {
              d_y = 0;
            }
            if (trueCount <= d_y) {
              d_y = (short)trueCount;
            }
            if (d_y < 1) {
              findId0_data = 0;
            } else {
              findId0_data = d_y;
            }
            for (i = 0; i < findId0_data; i++) {
              ind2Stay_data[i] = iidx_data[c_tmp_data[i]];
            }
            break;
          default:
            findId0_data = 1;
            ind2Stay_data[0] = 0;
            break;
          }
        } else {
          findId0_data = 1;
          ind2Stay_data[0] = 0;
        }
      }
      if ((int)Nina219 - 1 >= 0) {
        if (prmBrd_Nbat < 1) {
          b_y->size[0] = 1;
          b_y->size[1] = 0;
        } else {
          i = b_y->size[0] * b_y->size[1];
          b_y->size[0] = 1;
          b_y->size[1] = prmBrd_Nbat;
          emxEnsureCapacity_uint16_T(b_y, i);
          y_data = b_y->data;
          loop_ub = prmBrd_Nbat - 1;
          for (i = 0; i <= loop_ub; i++) {
            y_data[i] = (unsigned short)((unsigned int)i + 1U);
          }
        }
        c_loop_ub = b_y->size[1];
        end = findId0_data - 1;
      }
      if (loop_ub_tmp - 1 >= 0) {
        b_loop_ub = BattConfigAct_size[0];
        b_BattConfigPerIna_size[0] = 1;
        b_BattConfigPerIna_size[1] = BattConfigPerIna_size[1];
        d_loop_ub = BattConfigPerIna_size[2];
        b_BattConfigPerIna_size[2] = d_loop_ub;
      }
      for (k_ina219 = 0; k_ina219 < loop_ub_tmp; k_ina219++) {
        /*  BattConfigAct{k_ina219} = batAct([1:Nbat]+(k_ina219-1)*Nbat); */
        b_k_ina219 = (((double)k_ina219 + 1.0) - 1.0) * (double)prmBrd_Nbat;
        for (i = 0; i < c_loop_ub; i++) {
          d_tmp_data[i] =
              batAct_data[(int)((double)y_data[i] + b_k_ina219) - 1];
        }
        for (i = 0; i < b_loop_ub; i++) {
          BattConfigAct_data[i + BattConfigAct_size[0] * k_ina219] =
              d_tmp_data[i];
        }
        trueCount = 0;
        stride_0_0 = 0;
        for (b_i = 0; b_i <= end; b_i++) {
          i = ind2Stay_data[b_i];
          d = (double)prmBrd_Nbat + b_k_ina219;
          if ((b_k_ina219 + 1.0 <= i) && (i <= d)) {
            trueCount++;
            BattConfigPerInaNew_data[stride_0_0] = i;
            stride_0_0++;
          }
        }
        e_sort(BattConfigPerInaNew_data, &trueCount);
        for (i = 0; i < trueCount; i++) {
          BattConfigPerInaNew_data[i] -= b_k_ina219;
        }
        for (i = 0; i < d_loop_ub; i++) {
          loop_ub = BattConfigPerIna_size[1];
          for (findId0_data = 0; findId0_data < loop_ub; findId0_data++) {
            b_BattConfigPerIna_data[findId0_data +
                                    b_BattConfigPerIna_size[1] * i] =
                BattConfigPerIna_data[(k_ina219 + BattConfigPerIna_size[0] *
                                                      findId0_data) +
                                      BattConfigPerIna_size[0] *
                                          BattConfigPerIna_size[1] * i];
          }
        }
        remPadArr(b_BattConfigPerIna_data, b_BattConfigPerIna_size, tmp_data,
                  tmp_size);
        if (!isequal(tmp_data, tmp_size, BattConfigPerInaNew_data, trueCount)) {
          if (trueCount != 0) {
            padArr(BattConfigPerInaNew_data, trueCount, prmBrd_Nbat,
                   prmBrd_Nbat, r);
            VbatC_data = r->data;
            nx = BattConfigPerIna_size[1];
            stride_0_0 = BattConfigPerIna_size[2];
            for (i = 0; i < stride_0_0; i++) {
              for (findId0_data = 0; findId0_data < nx; findId0_data++) {
                BattConfigPerIna_data[(k_ina219 + BattConfigPerIna_size[0] *
                                                      findId0_data) +
                                      BattConfigPerIna_size[0] *
                                          BattConfigPerIna_size[1] * i] =
                    VbatC_data[findId0_data + nx * i];
              }
            }
          } else {
            b_padArr(prmBrd_spi_bypass, prmBrd_Nbat, prmBrd_Nbat, r);
            VbatC_data = r->data;
            nx = BattConfigPerIna_size[1];
            stride_0_0 = BattConfigPerIna_size[2];
            for (i = 0; i < stride_0_0; i++) {
              for (findId0_data = 0; findId0_data < nx; findId0_data++) {
                BattConfigPerIna_data[(k_ina219 + BattConfigPerIna_size[0] *
                                                      findId0_data) +
                                      BattConfigPerIna_size[0] *
                                          BattConfigPerIna_size[1] * i] =
                    VbatC_data[findId0_data + nx * i];
              }
            }
          }
          changeConfigFlag_data[k_ina219] = true;
          tLastToggle_data[k_ina219] = t;
        }
      }
      sumNotBypass = 0U;
      for (k_ina219 = 0; k_ina219 < loop_ub_tmp; k_ina219++) {
        if (BattConfigPerIna_data[k_ina219] != prmBrd_spi_bypass) {
          /* >0 */
          sumNotBypass++;
        }
      }
      if ((double)sumNotBypass < prmCnfg_minLenIna219) {
        /* ||SwitchToggleFlage% goto charge */
        if (b) {
          nx = Sel_size[1];
          Sel_size[0] = 1;
          for (i = 0; i < nx; i++) {
            Sel_data[i] = 1.0;
          }
          switchFlag = true;
        } else {
          b_Sel_size[0] = 1;
          b_Sel_size[1] = Sel_size[1];
          loop_ub = Sel_size[1];
          for (i = 0; i < loop_ub; i++) {
            b_Sel_data[i] = (Sel_data[i] == 1.0);
          }
          f_Sel_data.data = &b_Sel_data[0];
          f_Sel_data.size = &b_Sel_size[0];
          f_Sel_data.allocatedSize = 2;
          f_Sel_data.numDimensions = 2;
          f_Sel_data.canFreeData = false;
          if (b_any(&f_Sel_data)) {
            nx = Sel_size[1];
            Sel_size[0] = 1;
            for (i = 0; i < nx; i++) {
              Sel_data[i] = 1.5;
            }
            switchFlag = true;
          } else {
            b_Sel_size[0] = 1;
            b_Sel_size[1] = Sel_size[1];
            loop_ub = Sel_size[1];
            for (i = 0; i < loop_ub; i++) {
              b_Sel_data[i] = (Sel_data[i] == 1.5);
            }
            g_Sel_data.data = &b_Sel_data[0];
            g_Sel_data.size = &b_Sel_size[0];
            g_Sel_data.allocatedSize = 2;
            g_Sel_data.numDimensions = 2;
            g_Sel_data.canFreeData = false;
            if (b_any(&g_Sel_data)) {
              nx = Sel_size[1];
              Sel_size[0] = 1;
              for (i = 0; i < nx; i++) {
                Sel_data[i] = 1.6;
              }
              /* 1.6 */
              switchFlag = true;
            } else {
              b_Sel_size[0] = 1;
              b_Sel_size[1] = Sel_size[1];
              loop_ub = Sel_size[1];
              for (i = 0; i < loop_ub; i++) {
                b_Sel_data[i] = (Sel_data[i] == 1.6);
              }
              h_Sel_data.data = &b_Sel_data[0];
              h_Sel_data.size = &b_Sel_size[0];
              h_Sel_data.allocatedSize = 2;
              h_Sel_data.numDimensions = 2;
              h_Sel_data.canFreeData = false;
              if (b_any(&h_Sel_data)) {
                nx = Sel_size[1];
                Sel_size[0] = 1;
                if (nx - 1 >= 0) {
                  memset(&Sel_data[0], 0, (unsigned int)nx * sizeof(double));
                }
                switchFlag = true;
              }
            }
          }
        }
        nx = (int)Nina219;
        changeConfigFlag_size[0] = 1;
        changeConfigFlag_size[1] = (int)Nina219;
        if (nx - 1 >= 0) {
          memset(&changeConfigFlag_data[0], 0,
                 (unsigned int)nx * sizeof(boolean_T));
        }
        nx = tLastToggle_size[1];
        tLastToggle_size[0] = 1;
        for (i = 0; i < nx; i++) {
          tLastToggle_data[i] = t;
        }
        for (k_ina219 = 0; k_ina219 < loop_ub_tmp; k_ina219++) {
          loop_ub = BattConfigAct_size[0];
          for (i = 0; i < loop_ub; i++) {
            BattConfigAct_data[i + BattConfigAct_size[0] * k_ina219] = true;
          }
          /* [1:Nbat]'+(k_ina219-1)*Nbat; */
        }
      }
    }
    emxFree_real32_T(&b_x);
    emxFree_real32_T(&c_y);
    emxFree_uint16_T(&b_y);
    emxFree_real_T(&r);
    emxFree_real_T(&VbatC);
    emxFree_real_T(&Vc);
    emxFree_real_T(&C);
  }
  return switchFlag;
}

/*
 * File trailer for Esp32StepSwitchToggleCombAll_ESP_48.c
 *
 * [EOF]
 */
