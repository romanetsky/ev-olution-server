/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: Esp32StepSwitchToggleCombAll_ESP_48.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 31-Mar-2024 07:04:52
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

static void binary_expand_op_9(emxArray_real_T *in1, const emxArray_real_T *in2,
                               const boolean_T in3_data[], const int *in3_size);

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
 * Arguments    : emxArray_real_T *in1
 *                const emxArray_real_T *in2
 *                const boolean_T in3_data[]
 *                const int *in3_size
 * Return Type  : void
 */
static void binary_expand_op_9(emxArray_real_T *in1, const emxArray_real_T *in2,
                               const boolean_T in3_data[], const int *in3_size)
{
  const double *in2_data;
  double *in1_data;
  int i;
  int loop_ub;
  int stride_0_0;
  int stride_1_0;
  in2_data = in2->data;
  if (*in3_size == 1) {
    loop_ub = in2->size[0];
  } else {
    loop_ub = *in3_size;
  }
  i = in1->size[0];
  in1->size[0] = loop_ub;
  emxEnsureCapacity_real_T(in1, i);
  in1_data = in1->data;
  stride_0_0 = (in2->size[0] != 1);
  stride_1_0 = (*in3_size != 1);
  for (i = 0; i < loop_ub; i++) {
    in1_data[i] = in2_data[i * stride_0_0] * (double)in3_data[i * stride_1_0];
  }
}

/*
 * Arguments    : double Sel_data[]
 *                int Sel_size[2]
 *                float CutOffChrV
 *                float CutOffDisV
 *                emxArray_real_T *BattConfigPerIna
 *                const emxArray_real_T *Vbat
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
    emxArray_real_T *BattConfigPerIna, const emxArray_real_T *Vbat,
    double Nina219, short prmBrd_Nbat, unsigned char prmBrd_spi_disconnect,
    unsigned char prmBrd_spi_bypass, float prmCnfg_Ttoggle,
    short prmCnfg_NtoggleDrop, short prmCnfg_minLenIna219,
    const struct4_T prmSeq, signed char VthFlag, const float t_data[],
    const int t_size[2], double tLastToggle_data[], int tLastToggle_size[2],
    boolean_T BattConfigAct_data[], const int BattConfigAct_size[2],
    boolean_T changeConfigFlag_data[], int changeConfigFlag_size[2])
{
  static boolean_T b_ind2out_data[65534];
  static boolean_T batAct_data[65534];
  static boolean_T tmp_data[65534];
  emxArray_boolean_T c_Sel_data;
  emxArray_boolean_T c_ind2out_data;
  emxArray_boolean_T d_Sel_data;
  emxArray_boolean_T e_Sel_data;
  emxArray_boolean_T f_Sel_data;
  emxArray_boolean_T g_Sel_data;
  emxArray_boolean_T h_Sel_data;
  emxArray_boolean_T *b_VbatC;
  emxArray_boolean_T *b_x;
  emxArray_int32_T *iidx;
  emxArray_int32_T *ind2Stay;
  emxArray_real32_T *c_x;
  emxArray_real32_T *d_y;
  emxArray_real_T *BattConfigPerInaNew;
  emxArray_real_T *C;
  emxArray_real_T *VbatC;
  emxArray_real_T *Vc;
  emxArray_real_T *b_BattConfigPerIna;
  emxArray_real_T *b_y;
  emxArray_real_T *r;
  emxArray_real_T *r1;
  emxArray_real_T *sVbat0;
  emxArray_real_T *x;
  emxArray_uint16_T *c_y;
  emxArray_uint16_T *r2;
  const double *Vbat_data;
  double b_k_ina219;
  double sumV;
  double *BattConfigPerIna_data;
  double *C_data;
  double *VbatC_data;
  double *sVbat0_data;
  float VthDis;
  float VthOvDis;
  float *b_x_data;
  float *b_y_data;
  int b_Sel_size[2];
  int tmp_size[2];
  int Nact;
  int b_ind2out_size;
  int b_loop_ub;
  int end;
  int findId0_data;
  int i;
  int ind2out_size;
  int k_ina219;
  int k_nc;
  int loop_ub;
  int loop_ub_tmp;
  int nx;
  int stride_0_1;
  unsigned int sumNotBypass;
  int *iidx_data;
  int *ind2Stay_data;
  short e_y;
  unsigned short *r3;
  unsigned short *y_data;
  boolean_T ind2out_data[32767];
  boolean_T b_Sel_data[2];
  boolean_T UnOvVflag;
  boolean_T b;
  boolean_T exitg1;
  boolean_T exitg2;
  boolean_T guard1;
  boolean_T switchFlag;
  boolean_T y;
  boolean_T *x_data;
  Vbat_data = Vbat->data;
  BattConfigPerIna_data = BattConfigPerIna->data;
  VthDis = prmSeq.VthDis;
  /*  -1 - NA */
  /*  VthFlag         = prmSeq.VthFlag; % true - Vth mode on */
  VthOvDis = prmSeq.VthOvDis;
  changeConfigFlag_size[0] = 1;
  loop_ub_tmp = (int)Nina219;
  changeConfigFlag_size[1] = (int)Nina219;
  if (loop_ub_tmp - 1 >= 0) {
    memset(&changeConfigFlag_data[0], 0,
           (unsigned int)loop_ub_tmp * sizeof(boolean_T));
  }
  switchFlag = false;
  /*  SwitchToggleFlage = false; */
  k_nc = 0;
  /*  1 */
  sumV = 0.0;
  emxInit_real_T(&r, 2);
  emxInit_real_T(&x, 2);
  emxInit_real_T(&b_BattConfigPerIna, 3);
  emxInit_real_T(&r1, 2);
  for (k_ina219 = 0; k_ina219 < loop_ub_tmp; k_ina219++) {
    b_k_ina219 = BattConfigPerIna_data[k_ina219];
    if ((b_k_ina219 != prmBrd_spi_bypass) &&
        (b_k_ina219 != prmBrd_spi_disconnect)) {
      i = b_BattConfigPerIna->size[0] * b_BattConfigPerIna->size[1] *
          b_BattConfigPerIna->size[2];
      b_BattConfigPerIna->size[0] = 1;
      b_BattConfigPerIna->size[1] = BattConfigPerIna->size[1];
      b_BattConfigPerIna->size[2] = BattConfigPerIna->size[2];
      emxEnsureCapacity_real_T(b_BattConfigPerIna, i);
      VbatC_data = b_BattConfigPerIna->data;
      loop_ub = BattConfigPerIna->size[2];
      for (i = 0; i < loop_ub; i++) {
        nx = BattConfigPerIna->size[1];
        for (ind2out_size = 0; ind2out_size < nx; ind2out_size++) {
          VbatC_data[ind2out_size + b_BattConfigPerIna->size[1] * i] =
              BattConfigPerIna_data[(k_ina219 +
                                     BattConfigPerIna->size[0] * ind2out_size) +
                                    BattConfigPerIna->size[0] *
                                        BattConfigPerIna->size[1] * i];
        }
      }
      remPadArr(b_BattConfigPerIna, r);
      VbatC_data = r->data;
      i = x->size[0] * x->size[1];
      x->size[0] = r->size[0];
      x->size[1] = r->size[1];
      emxEnsureCapacity_real_T(x, i);
      sVbat0_data = x->data;
      b_k_ina219 = (((double)k_ina219 + 1.0) - 1.0) * (double)prmBrd_Nbat;
      loop_ub = r->size[0] * r->size[1];
      for (i = 0; i < loop_ub; i++) {
        sVbat0_data[i] = Vbat_data[(int)(VbatC_data[i] + b_k_ina219) - 1];
      }
      combineVectorElements(x, r1);
      VbatC_data = r1->data;
      sumV += VbatC_data[0];
    }
    /*  if BattConfigPerIna{k_ina219}(1) ~= bypass &&
     * BattConfigPerIna{k_ina219}(1) ~= disconnect */
    /*      sumV = sumV + sum(Vbat(BattConfigPerIna{k_ina219} +
     * (k_ina219-1)*Nbat)); */
    /*  end */
  }
  emxFree_real_T(&r1);
  emxFree_real_T(&x);
  emxInit_boolean_T(&b_x, 2);
  i = b_x->size[0] * b_x->size[1];
  b_x->size[0] = 1;
  b_x->size[1] = Sel_size[1];
  emxEnsureCapacity_boolean_T(b_x, i);
  x_data = b_x->data;
  loop_ub = Sel_size[1];
  for (i = 0; i < loop_ub; i++) {
    x_data[i] = (Sel_data[i] == 0.0);
  }
  b = b_any(b_x);
  if (b) {
    ind2out_size = Vbat->size[0];
    loop_ub = Vbat->size[0];
    for (i = 0; i < loop_ub; i++) {
      ind2out_data[i] = (Vbat_data[i] < CutOffDisV);
    }
    UnOvVflag = ((prmSeq.VthUnDis > sumV) && (VthFlag > 0));
  } else {
    b_Sel_size[0] = 1;
    b_Sel_size[1] = Sel_size[1];
    loop_ub = Sel_size[1];
    for (i = 0; i < loop_ub; i++) {
      b_k_ina219 = Sel_data[i];
      b_Sel_data[i] = ((b_k_ina219 >= 1.0) && (b_k_ina219 < 2.0));
    }
    c_Sel_data.data = &b_Sel_data[0];
    c_Sel_data.size = &b_Sel_size[0];
    c_Sel_data.allocatedSize = 2;
    c_Sel_data.numDimensions = 2;
    c_Sel_data.canFreeData = false;
    if (b_any(&c_Sel_data)) {
      ind2out_size = Vbat->size[0];
      loop_ub = Vbat->size[0];
      for (i = 0; i < loop_ub; i++) {
        ind2out_data[i] = (Vbat_data[i] > CutOffChrV);
      }
      UnOvVflag = ((prmSeq.VthOvChr < sumV) && (VthFlag > 0));
    } else {
      ind2out_size = Vbat->size[0];
      loop_ub = Vbat->size[0];
      if (loop_ub - 1 >= 0) {
        memset(&ind2out_data[0], 0, (unsigned int)loop_ub * sizeof(boolean_T));
      }
      UnOvVflag = false;
    }
  }
  i = b_x->size[0] * b_x->size[1];
  b_x->size[0] = 1;
  b_x->size[1] = Sel_size[1];
  emxEnsureCapacity_boolean_T(b_x, i);
  x_data = b_x->data;
  loop_ub = Sel_size[1];
  for (i = 0; i < loop_ub; i++) {
    b_k_ina219 = Sel_data[i];
    x_data[i] = ((b_k_ina219 == prmBrd_spi_bypass) ||
                 (b_k_ina219 == prmBrd_spi_disconnect));
  }
  y = true;
  nx = 1;
  exitg1 = false;
  while ((!exitg1) && (nx <= b_x->size[1])) {
    if (!x_data[nx - 1]) {
      y = false;
      exitg1 = true;
    } else {
      nx++;
    }
  }
  emxFree_boolean_T(&b_x);
  if (!y) {
    b_Sel_size[0] = 1;
    if (tLastToggle_size[1] == 1) {
      loop_ub = t_size[1];
    } else {
      loop_ub = tLastToggle_size[1];
    }
    b_Sel_size[1] = loop_ub;
    stride_0_1 = (t_size[1] != 1);
    nx = (tLastToggle_size[1] != 1);
    for (i = 0; i < loop_ub; i++) {
      b_Sel_data[i] = (prmCnfg_Ttoggle < t_data[i * stride_0_1] -
                                             (float)tLastToggle_data[i * nx]);
    }
    d_Sel_data.data = &b_Sel_data[0];
    d_Sel_data.size = &b_Sel_size[0];
    d_Sel_data.allocatedSize = 2;
    d_Sel_data.numDimensions = 2;
    d_Sel_data.canFreeData = false;
    emxInit_real_T(&BattConfigPerInaNew, 1);
    emxInit_real_T(&sVbat0, 1);
    emxInit_int32_T(&ind2Stay, 1);
    emxInit_real_T(&C, 2);
    C_data = C->data;
    emxInit_real_T(&Vc, 1);
    emxInit_real_T(&VbatC, 2);
    emxInit_uint16_T(&r2, 1);
    emxInit_real_T(&b_y, 1);
    emxInit_int32_T(&iidx, 1);
    emxInit_uint16_T(&c_y, 2);
    y_data = c_y->data;
    emxInit_real32_T(&d_y, 1);
    emxInit_real32_T(&c_x, 1);
    emxInit_boolean_T(&b_VbatC, 2);
    guard1 = false;
    if (b_any(&d_Sel_data)) {
      guard1 = true;
    } else {
      nx = BattConfigAct_size[0] * BattConfigAct_size[1];
      if (nx == 1) {
        loop_ub = ind2out_size;
      } else {
        loop_ub = nx;
      }
      b_ind2out_size = loop_ub;
      stride_0_1 = (ind2out_size != 1);
      nx = (nx != 1);
      for (i = 0; i < loop_ub; i++) {
        b_ind2out_data[i] =
            (ind2out_data[i * stride_0_1] && BattConfigAct_data[i * nx]);
      }
      c_ind2out_data.data = &b_ind2out_data[0];
      c_ind2out_data.size = &b_ind2out_size;
      c_ind2out_data.allocatedSize = 65534;
      c_ind2out_data.numDimensions = 1;
      c_ind2out_data.canFreeData = false;
      if (any(&c_ind2out_data) || UnOvVflag) {
        guard1 = true;
      }
    }
    if (guard1) {
      loop_ub = BattConfigAct_size[0] * BattConfigAct_size[1];
      if (loop_ub == ind2out_size) {
        for (i = 0; i < loop_ub; i++) {
          batAct_data[i] = (BattConfigAct_data[i] && (!ind2out_data[i]));
        }
      } else {
        loop_ub = binary_expand_op_11(batAct_data, BattConfigAct_data,
                                      BattConfigAct_size, ind2out_data,
                                      &ind2out_size);
      }
      if (b) {
        if (Vbat->size[0] == loop_ub) {
          i = b_y->size[0];
          b_y->size[0] = Vbat->size[0];
          emxEnsureCapacity_real_T(b_y, i);
          VbatC_data = b_y->data;
          loop_ub = Vbat->size[0];
          for (i = 0; i < loop_ub; i++) {
            VbatC_data[i] = Vbat_data[i] * (double)batAct_data[i];
          }
        } else {
          binary_expand_op_9(b_y, Vbat, batAct_data, &loop_ub);
        }
        c_sort(b_y, iidx);
        iidx_data = iidx->data;
        VbatC_data = b_y->data;
        nx = b_y->size[0] - 1;
        stride_0_1 = 0;
        for (ind2out_size = 0; ind2out_size <= nx; ind2out_size++) {
          if (VbatC_data[ind2out_size] > 0.0) {
            stride_0_1++;
          }
        }
        i = r2->size[0];
        r2->size[0] = stride_0_1;
        emxEnsureCapacity_uint16_T(r2, i);
        r3 = r2->data;
        stride_0_1 = 0;
        for (ind2out_size = 0; ind2out_size <= nx; ind2out_size++) {
          if (VbatC_data[ind2out_size] > 0.0) {
            r3[stride_0_1] = (unsigned short)ind2out_size;
            stride_0_1++;
          }
        }
        i = sVbat0->size[0];
        sVbat0->size[0] = r2->size[0];
        emxEnsureCapacity_real_T(sVbat0, i);
        sVbat0_data = sVbat0->data;
        loop_ub = r2->size[0];
        for (i = 0; i < loop_ub; i++) {
          sVbat0_data[i] = VbatC_data[r3[i]];
        }
        useConstantDim(sVbat0, nonSingletonDim(sVbat0));
        sVbat0_data = sVbat0->data;
        Nact = r2->size[0];
        switch (VthFlag) {
        case 1:
          b_ind2out_size = sVbat0->size[0];
          loop_ub = sVbat0->size[0];
          for (i = 0; i < loop_ub; i++) {
            b_ind2out_data[i] = (sVbat0_data[i] > prmSeq.VthDis);
          }
          nx = g_eml_find(b_ind2out_data, b_ind2out_size, (int *)&findId0_data);
          if (nx == 0) {
            b_ind2out_size = sVbat0->size[0];
            loop_ub = sVbat0->size[0];
            for (i = 0; i < loop_ub; i++) {
              b_ind2out_data[i] = (sVbat0_data[i] > prmSeq.VthUnDis);
            }
            g_eml_find(b_ind2out_data, b_ind2out_size, (int *)&findId0_data);
          }
          i = ind2Stay->size[0];
          ind2Stay->size[0] = findId0_data;
          emxEnsureCapacity_int32_T(ind2Stay, i);
          ind2Stay_data = ind2Stay->data;
          for (i = 0; i < findId0_data; i++) {
            ind2Stay_data[i] = iidx_data[r3[i]];
          }
          if (sVbat0_data[findId0_data - 1] > prmSeq.VthOvDis) {
            UnOvVflag = false;
            ind2out_size = 0;
            exitg1 = false;
            while ((!exitg1) && (ind2out_size < 3)) {
              b_k_ina219 = (double)findId0_data + (double)ind2out_size;
              if (b_k_ina219 + 4.0 <= Nact) {
                i = (int)(b_k_ina219 + 4.0);
              } else {
                i = Nact;
              }
              if (i < 1) {
                loop_ub = 0;
              } else {
                loop_ub = i;
              }
              i = b_y->size[0];
              b_y->size[0] = loop_ub;
              emxEnsureCapacity_real_T(b_y, i);
              VbatC_data = b_y->data;
              for (i = 0; i < loop_ub; i++) {
                VbatC_data[i] = iidx_data[r3[i]];
              }
              nchoosek(b_y, b_k_ina219, C);
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
              stride_0_1 = 0;
              exitg2 = false;
              while ((!exitg2) && (stride_0_1 <= C->size[0] - 1)) {
                k_nc = stride_0_1;
                i = b_VbatC->size[0] * b_VbatC->size[1];
                b_VbatC->size[0] = 1;
                b_VbatC->size[1] = VbatC->size[1];
                emxEnsureCapacity_boolean_T(b_VbatC, i);
                x_data = b_VbatC->data;
                loop_ub = VbatC->size[1];
                for (i = 0; i < loop_ub; i++) {
                  x_data[i] =
                      ((VbatC_data[stride_0_1 + VbatC->size[0] * i] > VthDis) &&
                       (VbatC_data[stride_0_1 + VbatC->size[0] * i] <
                        VthOvDis));
                }
                f_eml_find(b_VbatC, (int *)&k_ina219, tmp_size);
                if (tmp_size[1] != 0) {
                  UnOvVflag = true;
                  exitg2 = true;
                } else {
                  stride_0_1++;
                }
              }
              if (UnOvVflag) {
                exitg1 = true;
              } else {
                ind2out_size++;
              }
            }
            i = ind2Stay->size[0];
            ind2Stay->size[0] = C->size[1];
            emxEnsureCapacity_int32_T(ind2Stay, i);
            ind2Stay_data = ind2Stay->data;
            loop_ub = C->size[1];
            for (i = 0; i < loop_ub; i++) {
              ind2Stay_data[i] = (int)C_data[k_nc + C->size[0] * i];
            }
          }
          break;
        case 2:
          /* bestfit */
          b_ind2out_size = sVbat0->size[0];
          loop_ub = sVbat0->size[0];
          for (i = 0; i < loop_ub; i++) {
            b_ind2out_data[i] = (sVbat0_data[i] > prmSeq.VthDis);
          }
          nx = g_eml_find(b_ind2out_data, b_ind2out_size, (int *)&findId0_data);
          if (nx == 0) {
            b_ind2out_size = sVbat0->size[0];
            loop_ub = sVbat0->size[0];
            for (i = 0; i < loop_ub; i++) {
              b_ind2out_data[i] = (sVbat0_data[i] > prmSeq.VthUnDis);
            }
            g_eml_find(b_ind2out_data, b_ind2out_size, (int *)&findId0_data);
          }
          /*                          if csVbat(findId)>VthOvDis */
          ind2out_size = 0;
          exitg1 = false;
          while ((!exitg1) && (ind2out_size < 3)) {
            b_k_ina219 = (double)findId0_data + (double)ind2out_size;
            if (b_k_ina219 + 6.0 <= Nact) {
              i = (int)(b_k_ina219 + 6.0);
            } else {
              i = Nact;
            }
            if (i < 1) {
              loop_ub = 0;
            } else {
              loop_ub = i;
            }
            i = b_y->size[0];
            b_y->size[0] = loop_ub;
            emxEnsureCapacity_real_T(b_y, i);
            VbatC_data = b_y->data;
            for (i = 0; i < loop_ub; i++) {
              VbatC_data[i] = iidx_data[r3[i]];
            }
            nchoosek(b_y, b_k_ina219, C);
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
            sum(VbatC, Vc);
            sVbat0_data = Vc->data;
            i = c_x->size[0];
            c_x->size[0] = Vc->size[0];
            emxEnsureCapacity_real32_T(c_x, i);
            b_x_data = c_x->data;
            loop_ub = Vc->size[0];
            for (i = 0; i < loop_ub; i++) {
              b_x_data[i] = (float)sVbat0_data[i] - VthDis;
            }
            nx = c_x->size[0];
            i = d_y->size[0];
            d_y->size[0] = c_x->size[0];
            emxEnsureCapacity_real32_T(d_y, i);
            b_y_data = d_y->data;
            for (stride_0_1 = 0; stride_0_1 < nx; stride_0_1++) {
              b_y_data[stride_0_1] = (float)fabs(b_x_data[stride_0_1]);
            }
            b_minimum(d_y, &nx);
            k_nc = nx - 1;
            /*                                  for k_nc = 1:NC */
            /*                                  idC =
             * find(Vc(k_nc,:)>VthDis&Vc(k_nc,:)<VthOvDis,1,'first'); */
            b_k_ina219 = sVbat0_data[nx - 1];
            if ((b_k_ina219 > VthDis) && (b_k_ina219 < VthOvDis) &&
                (b_k_ina219 > prmSeq.VthUnDis)) {
              exitg1 = true;
            } else {
              /*                                  end */
              ind2out_size++;
            }
          }
          i = ind2Stay->size[0];
          ind2Stay->size[0] = C->size[1];
          emxEnsureCapacity_int32_T(ind2Stay, i);
          ind2Stay_data = ind2Stay->data;
          loop_ub = C->size[1];
          for (i = 0; i < loop_ub; i++) {
            ind2Stay_data[i] = (int)C_data[k_nc + C->size[0] * i];
          }
          /*                          end */
          break;
        case 0:
          /* toggle */
          b_k_ina219 = rt_roundd_snf((double)prmBrd_Nbat * Nina219 -
                                     (double)prmCnfg_NtoggleDrop);
          if (b_k_ina219 < 32768.0) {
            if (b_k_ina219 >= -32768.0) {
              e_y = (short)b_k_ina219;
            } else {
              e_y = MIN_int16_T;
            }
          } else if (b_k_ina219 >= 32768.0) {
            e_y = MAX_int16_T;
          } else {
            e_y = 0;
          }
          if (r2->size[0] <= e_y) {
            e_y = (short)r2->size[0];
          }
          if (e_y < 1) {
            loop_ub = 0;
          } else {
            loop_ub = e_y;
          }
          i = ind2Stay->size[0];
          ind2Stay->size[0] = loop_ub;
          emxEnsureCapacity_int32_T(ind2Stay, i);
          ind2Stay_data = ind2Stay->data;
          for (i = 0; i < loop_ub; i++) {
            ind2Stay_data[i] = iidx_data[r3[i]];
          }
          break;
        default:
          i = ind2Stay->size[0];
          ind2Stay->size[0] = 1;
          emxEnsureCapacity_int32_T(ind2Stay, i);
          ind2Stay_data = ind2Stay->data;
          ind2Stay_data[0] = 0;
          break;
        }
      } else {
        b_Sel_size[0] = 1;
        b_Sel_size[1] = Sel_size[1];
        nx = Sel_size[1];
        for (i = 0; i < nx; i++) {
          b_k_ina219 = Sel_data[i];
          b_Sel_data[i] = ((b_k_ina219 >= 1.0) && (b_k_ina219 <= 2.0));
        }
        e_Sel_data.data = &b_Sel_data[0];
        e_Sel_data.size = &b_Sel_size[0];
        e_Sel_data.allocatedSize = 2;
        e_Sel_data.numDimensions = 2;
        e_Sel_data.canFreeData = false;
        if (b_any(&e_Sel_data)) {
          if (Vbat->size[0] == loop_ub) {
            i = b_y->size[0];
            b_y->size[0] = Vbat->size[0];
            emxEnsureCapacity_real_T(b_y, i);
            VbatC_data = b_y->data;
            loop_ub = Vbat->size[0];
            for (i = 0; i < loop_ub; i++) {
              VbatC_data[i] = Vbat_data[i] * (double)batAct_data[i];
            }
          } else {
            binary_expand_op_9(b_y, Vbat, batAct_data, &loop_ub);
          }
          d_sort(b_y, iidx);
          iidx_data = iidx->data;
          VbatC_data = b_y->data;
          nx = b_y->size[0] - 1;
          stride_0_1 = 0;
          for (ind2out_size = 0; ind2out_size <= nx; ind2out_size++) {
            if (VbatC_data[ind2out_size] > 0.0) {
              stride_0_1++;
            }
          }
          i = r2->size[0];
          r2->size[0] = stride_0_1;
          emxEnsureCapacity_uint16_T(r2, i);
          r3 = r2->data;
          stride_0_1 = 0;
          for (ind2out_size = 0; ind2out_size <= nx; ind2out_size++) {
            if (VbatC_data[ind2out_size] > 0.0) {
              r3[stride_0_1] = (unsigned short)ind2out_size;
              stride_0_1++;
            }
          }
          i = sVbat0->size[0];
          sVbat0->size[0] = r2->size[0];
          emxEnsureCapacity_real_T(sVbat0, i);
          sVbat0_data = sVbat0->data;
          loop_ub = r2->size[0];
          for (i = 0; i < loop_ub; i++) {
            sVbat0_data[i] = VbatC_data[r3[i]];
          }
          useConstantDim(sVbat0, nonSingletonDim(sVbat0));
          sVbat0_data = sVbat0->data;
          Nact = r2->size[0];
          switch (VthFlag) {
          case 1:
            b_ind2out_size = sVbat0->size[0];
            loop_ub = sVbat0->size[0];
            for (i = 0; i < loop_ub; i++) {
              b_ind2out_data[i] = (sVbat0_data[i] > prmSeq.VthUnChr);
            }
            stride_0_1 = b_ind2out_size;
            loop_ub = b_ind2out_size;
            for (i = 0; i < loop_ub; i++) {
              tmp_data[i] =
                  (b_ind2out_data[i] && (sVbat0_data[i] < prmSeq.VthChr));
            }
            nx = g_eml_find(tmp_data, stride_0_1, (int *)&findId0_data);
            if (nx == 0) {
              loop_ub = b_ind2out_size;
              for (i = 0; i < loop_ub; i++) {
                b_ind2out_data[i] =
                    (b_ind2out_data[i] && (sVbat0_data[i] < prmSeq.VthOvChr));
              }
              g_eml_find(b_ind2out_data, b_ind2out_size, (int *)&findId0_data);
            }
            /*                      findId0 = find(csVbat<VthChr,1,'last'); */
            /*                      findIdUn = find(csVbat>VthUnChr,1,'first');
             */
            /*                      findId = max(findId,findIdUn); */
            i = ind2Stay->size[0];
            ind2Stay->size[0] = findId0_data;
            emxEnsureCapacity_int32_T(ind2Stay, i);
            ind2Stay_data = ind2Stay->data;
            for (i = 0; i < findId0_data; i++) {
              ind2Stay_data[i] = iidx_data[r3[i]];
            }
            break;
          case 2:
            b_ind2out_size = sVbat0->size[0];
            loop_ub = sVbat0->size[0];
            for (i = 0; i < loop_ub; i++) {
              b_ind2out_data[i] = (sVbat0_data[i] < prmSeq.VthChr);
            }
            /*                      findId0 = find(csVbat<VthChr,1,'last'); */
            /*                      findIdUn = find(csVbat>VthUnChr,1,'first');
             */
            /*                      findId = max(findId,findIdUn); */
            h_eml_find(b_ind2out_data, b_ind2out_size, (int *)&k_ina219);
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
            ind2out_size = 0;
            exitg1 = false;
            while ((!exitg1) && (ind2out_size < 3)) {
              b_k_ina219 = (double)k_ina219 + (double)ind2out_size;
              if (b_k_ina219 + 6.0 <= Nact) {
                i = (int)(b_k_ina219 + 6.0);
              } else {
                i = Nact;
              }
              if (i < 1) {
                loop_ub = 0;
              } else {
                loop_ub = i;
              }
              i = b_y->size[0];
              b_y->size[0] = loop_ub;
              emxEnsureCapacity_real_T(b_y, i);
              VbatC_data = b_y->data;
              for (i = 0; i < loop_ub; i++) {
                VbatC_data[i] = iidx_data[r3[i]];
              }
              nchoosek(b_y, b_k_ina219, C);
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
              sum(VbatC, Vc);
              sVbat0_data = Vc->data;
              i = c_x->size[0];
              c_x->size[0] = Vc->size[0];
              emxEnsureCapacity_real32_T(c_x, i);
              b_x_data = c_x->data;
              loop_ub = Vc->size[0];
              for (i = 0; i < loop_ub; i++) {
                b_x_data[i] = (float)sVbat0_data[i] - prmSeq.VthChr;
              }
              nx = c_x->size[0];
              i = d_y->size[0];
              d_y->size[0] = c_x->size[0];
              emxEnsureCapacity_real32_T(d_y, i);
              b_y_data = d_y->data;
              for (stride_0_1 = 0; stride_0_1 < nx; stride_0_1++) {
                b_y_data[stride_0_1] = (float)fabs(b_x_data[stride_0_1]);
              }
              b_minimum(d_y, &nx);
              k_nc = nx - 1;
              /*                                      for k_nc = 1:NC */
              /*                                          idC =
               * (Vc(k_nc,:)<VthChr&Vc(k_nc,:)>VthUnChr,1); */
              b_k_ina219 = sVbat0_data[nx - 1];
              if ((b_k_ina219 > prmSeq.VthUnChr) &&
                  (b_k_ina219 < prmSeq.VthOvChr)) {
                exitg1 = true;
              } else {
                /*                                      end */
                ind2out_size++;
              }
            }
            i = ind2Stay->size[0];
            ind2Stay->size[0] = C->size[1];
            emxEnsureCapacity_int32_T(ind2Stay, i);
            ind2Stay_data = ind2Stay->data;
            loop_ub = C->size[1];
            for (i = 0; i < loop_ub; i++) {
              ind2Stay_data[i] = (int)C_data[k_nc + C->size[0] * i];
            }
            /*                              end */
            /*                          end */
            break;
          case 0:
            /* toggle */
            b_k_ina219 = rt_roundd_snf((double)prmBrd_Nbat * Nina219 -
                                       (double)prmCnfg_NtoggleDrop);
            if (b_k_ina219 < 32768.0) {
              if (b_k_ina219 >= -32768.0) {
                e_y = (short)b_k_ina219;
              } else {
                e_y = MIN_int16_T;
              }
            } else if (b_k_ina219 >= 32768.0) {
              e_y = MAX_int16_T;
            } else {
              e_y = 0;
            }
            if (r2->size[0] <= e_y) {
              e_y = (short)r2->size[0];
            }
            if (e_y < 1) {
              loop_ub = 0;
            } else {
              loop_ub = e_y;
            }
            i = ind2Stay->size[0];
            ind2Stay->size[0] = loop_ub;
            emxEnsureCapacity_int32_T(ind2Stay, i);
            ind2Stay_data = ind2Stay->data;
            for (i = 0; i < loop_ub; i++) {
              ind2Stay_data[i] = iidx_data[r3[i]];
            }
            break;
          default:
            i = ind2Stay->size[0];
            ind2Stay->size[0] = 1;
            emxEnsureCapacity_int32_T(ind2Stay, i);
            ind2Stay_data = ind2Stay->data;
            ind2Stay_data[0] = 0;
            break;
          }
        } else {
          i = ind2Stay->size[0];
          ind2Stay->size[0] = 1;
          emxEnsureCapacity_int32_T(ind2Stay, i);
          ind2Stay_data = ind2Stay->data;
          ind2Stay_data[0] = 0;
        }
      }
      if ((int)Nina219 - 1 >= 0) {
        if (prmBrd_Nbat < 1) {
          c_y->size[0] = 1;
          c_y->size[1] = 0;
        } else {
          i = c_y->size[0] * c_y->size[1];
          c_y->size[0] = 1;
          c_y->size[1] = prmBrd_Nbat;
          emxEnsureCapacity_uint16_T(c_y, i);
          y_data = c_y->data;
          loop_ub = prmBrd_Nbat - 1;
          for (i = 0; i <= loop_ub; i++) {
            y_data[i] = (unsigned short)((unsigned int)i + 1U);
          }
        }
        b_loop_ub = c_y->size[1];
        end = ind2Stay->size[0] - 1;
      }
      for (k_ina219 = 0; k_ina219 < loop_ub_tmp; k_ina219++) {
        /*  BattConfigAct{k_ina219} = batAct([1:Nbat]+(k_ina219-1)*Nbat); */
        b_k_ina219 = (((double)k_ina219 + 1.0) - 1.0) * (double)prmBrd_Nbat;
        for (i = 0; i < b_loop_ub; i++) {
          ind2out_data[i] =
              batAct_data[(int)((double)y_data[i] + b_k_ina219) - 1];
        }
        loop_ub = BattConfigAct_size[0];
        for (i = 0; i < loop_ub; i++) {
          BattConfigAct_data[i + BattConfigAct_size[0] * k_ina219] =
              ind2out_data[i];
        }
        stride_0_1 = 0;
        for (ind2out_size = 0; ind2out_size <= end; ind2out_size++) {
          i = ind2Stay_data[ind2out_size];
          if ((b_k_ina219 + 1.0 <= i) &&
              (i <= (double)prmBrd_Nbat + b_k_ina219)) {
            stride_0_1++;
          }
        }
        i = BattConfigPerInaNew->size[0];
        BattConfigPerInaNew->size[0] = stride_0_1;
        emxEnsureCapacity_real_T(BattConfigPerInaNew, i);
        VbatC_data = BattConfigPerInaNew->data;
        stride_0_1 = 0;
        for (ind2out_size = 0; ind2out_size <= end; ind2out_size++) {
          i = ind2Stay_data[ind2out_size];
          if ((b_k_ina219 + 1.0 <= i) &&
              (i <= (double)prmBrd_Nbat + b_k_ina219)) {
            VbatC_data[stride_0_1] = i;
            stride_0_1++;
          }
        }
        e_sort(BattConfigPerInaNew);
        VbatC_data = BattConfigPerInaNew->data;
        loop_ub = BattConfigPerInaNew->size[0];
        for (i = 0; i < loop_ub; i++) {
          VbatC_data[i] -= b_k_ina219;
        }
        i = b_BattConfigPerIna->size[0] * b_BattConfigPerIna->size[1] *
            b_BattConfigPerIna->size[2];
        b_BattConfigPerIna->size[0] = 1;
        b_BattConfigPerIna->size[1] = BattConfigPerIna->size[1];
        loop_ub = BattConfigPerIna->size[2];
        b_BattConfigPerIna->size[2] = loop_ub;
        emxEnsureCapacity_real_T(b_BattConfigPerIna, i);
        VbatC_data = b_BattConfigPerIna->data;
        for (i = 0; i < loop_ub; i++) {
          nx = BattConfigPerIna->size[1];
          for (ind2out_size = 0; ind2out_size < nx; ind2out_size++) {
            VbatC_data[ind2out_size + b_BattConfigPerIna->size[1] * i] =
                BattConfigPerIna_data[(k_ina219 + BattConfigPerIna->size[0] *
                                                      ind2out_size) +
                                      BattConfigPerIna->size[0] *
                                          BattConfigPerIna->size[1] * i];
          }
        }
        remPadArr(b_BattConfigPerIna, r);
        if (!isequal(r, BattConfigPerInaNew)) {
          if (BattConfigPerInaNew->size[0] != 0) {
            padArr(BattConfigPerInaNew, prmBrd_Nbat, prmBrd_Nbat, r);
            VbatC_data = r->data;
            nx = BattConfigPerIna->size[1];
            stride_0_1 = BattConfigPerIna->size[2];
            for (i = 0; i < stride_0_1; i++) {
              for (ind2out_size = 0; ind2out_size < nx; ind2out_size++) {
                BattConfigPerIna_data[(k_ina219 + BattConfigPerIna->size[0] *
                                                      ind2out_size) +
                                      BattConfigPerIna->size[0] *
                                          BattConfigPerIna->size[1] * i] =
                    VbatC_data[ind2out_size + nx * i];
              }
            }
          } else {
            b_padArr(prmBrd_spi_bypass, prmBrd_Nbat, prmBrd_Nbat, r);
            VbatC_data = r->data;
            nx = BattConfigPerIna->size[1];
            stride_0_1 = BattConfigPerIna->size[2];
            for (i = 0; i < stride_0_1; i++) {
              for (ind2out_size = 0; ind2out_size < nx; ind2out_size++) {
                BattConfigPerIna_data[(k_ina219 + BattConfigPerIna->size[0] *
                                                      ind2out_size) +
                                      BattConfigPerIna->size[0] *
                                          BattConfigPerIna->size[1] * i] =
                    VbatC_data[ind2out_size + nx * i];
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
    emxFree_boolean_T(&b_VbatC);
    emxFree_real32_T(&c_x);
    emxFree_real32_T(&d_y);
    emxFree_uint16_T(&c_y);
    emxFree_int32_T(&iidx);
    emxFree_real_T(&b_y);
    emxFree_uint16_T(&r2);
    emxFree_real_T(&VbatC);
    emxFree_real_T(&Vc);
    emxFree_real_T(&C);
    emxFree_int32_T(&ind2Stay);
    emxFree_real_T(&sVbat0);
    emxFree_real_T(&BattConfigPerInaNew);
  }
  emxFree_real_T(&b_BattConfigPerIna);
  emxFree_real_T(&r);
  return switchFlag;
}

/*
 * Arguments    : double Sel_data[]
 *                int Sel_size[2]
 *                float CutOffChrV
 *                float CutOffDisV
 *                emxArray_real_T *BattConfigPerIna
 *                const emxArray_real_T *Vbat
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
    emxArray_real_T *BattConfigPerIna, const emxArray_real_T *Vbat,
    double Nina219, short prmBrd_Nbat, unsigned char prmBrd_spi_disconnect,
    unsigned char prmBrd_spi_bypass, float prmCnfg_Ttoggle,
    short prmCnfg_NtoggleDrop, short prmCnfg_minLenIna219,
    const struct4_T prmSeq, signed char VthFlag, double t,
    double tLastToggle_data[], int tLastToggle_size[2],
    boolean_T BattConfigAct_data[], const int BattConfigAct_size[2],
    boolean_T changeConfigFlag_data[], int changeConfigFlag_size[2])
{
  static boolean_T b_ind2out_data[65534];
  static boolean_T batAct_data[65534];
  static boolean_T tmp_data[65534];
  emxArray_boolean_T c_Sel_data;
  emxArray_boolean_T c_ind2out_data;
  emxArray_boolean_T d_Sel_data;
  emxArray_boolean_T e_Sel_data;
  emxArray_boolean_T f_Sel_data;
  emxArray_boolean_T g_Sel_data;
  emxArray_boolean_T h_Sel_data;
  emxArray_boolean_T *b_VbatC;
  emxArray_boolean_T *b_x;
  emxArray_int32_T *iidx;
  emxArray_int32_T *ind2Stay;
  emxArray_real32_T *c_x;
  emxArray_real32_T *d_y;
  emxArray_real_T *BattConfigPerInaNew;
  emxArray_real_T *C;
  emxArray_real_T *VbatC;
  emxArray_real_T *Vc;
  emxArray_real_T *b_BattConfigPerIna;
  emxArray_real_T *b_y;
  emxArray_real_T *r;
  emxArray_real_T *r1;
  emxArray_real_T *sVbat0;
  emxArray_real_T *x;
  emxArray_uint16_T *c_y;
  emxArray_uint16_T *r2;
  const double *Vbat_data;
  double b_k_ina219;
  double sumV;
  double *BattConfigPerIna_data;
  double *C_data;
  double *VbatC_data;
  double *sVbat0_data;
  float VthDis;
  float VthOvDis;
  float *b_x_data;
  float *b_y_data;
  int b_Sel_size[2];
  int tmp_size[2];
  int Nact;
  int b_loop_ub;
  int end;
  int findId0_data;
  int i;
  int ind2out_size;
  int k_ina219;
  int k_nc;
  int loop_ub;
  int loop_ub_tmp;
  int nx;
  int stride_0_0;
  int stride_1_0;
  unsigned int sumNotBypass;
  int *iidx_data;
  int *ind2Stay_data;
  short e_y;
  unsigned short *r3;
  unsigned short *y_data;
  boolean_T ind2out_data[32767];
  boolean_T b_Sel_data[2];
  boolean_T UnOvVflag;
  boolean_T b;
  boolean_T exitg1;
  boolean_T exitg2;
  boolean_T guard1;
  boolean_T switchFlag;
  boolean_T y;
  boolean_T *x_data;
  Vbat_data = Vbat->data;
  BattConfigPerIna_data = BattConfigPerIna->data;
  VthDis = prmSeq.VthDis;
  /*  -1 - NA */
  /*  VthFlag         = prmSeq.VthFlag; % true - Vth mode on */
  VthOvDis = prmSeq.VthOvDis;
  changeConfigFlag_size[0] = 1;
  loop_ub_tmp = (int)Nina219;
  changeConfigFlag_size[1] = (int)Nina219;
  if (loop_ub_tmp - 1 >= 0) {
    memset(&changeConfigFlag_data[0], 0,
           (unsigned int)loop_ub_tmp * sizeof(boolean_T));
  }
  switchFlag = false;
  /*  SwitchToggleFlage = false; */
  k_nc = 0;
  /*  1 */
  sumV = 0.0;
  emxInit_real_T(&r, 2);
  emxInit_real_T(&x, 2);
  emxInit_real_T(&b_BattConfigPerIna, 3);
  emxInit_real_T(&r1, 2);
  for (k_ina219 = 0; k_ina219 < loop_ub_tmp; k_ina219++) {
    b_k_ina219 = BattConfigPerIna_data[k_ina219];
    if ((b_k_ina219 != prmBrd_spi_bypass) &&
        (b_k_ina219 != prmBrd_spi_disconnect)) {
      i = b_BattConfigPerIna->size[0] * b_BattConfigPerIna->size[1] *
          b_BattConfigPerIna->size[2];
      b_BattConfigPerIna->size[0] = 1;
      b_BattConfigPerIna->size[1] = BattConfigPerIna->size[1];
      b_BattConfigPerIna->size[2] = BattConfigPerIna->size[2];
      emxEnsureCapacity_real_T(b_BattConfigPerIna, i);
      VbatC_data = b_BattConfigPerIna->data;
      loop_ub = BattConfigPerIna->size[2];
      for (i = 0; i < loop_ub; i++) {
        nx = BattConfigPerIna->size[1];
        for (stride_0_0 = 0; stride_0_0 < nx; stride_0_0++) {
          VbatC_data[stride_0_0 + b_BattConfigPerIna->size[1] * i] =
              BattConfigPerIna_data[(k_ina219 +
                                     BattConfigPerIna->size[0] * stride_0_0) +
                                    BattConfigPerIna->size[0] *
                                        BattConfigPerIna->size[1] * i];
        }
      }
      remPadArr(b_BattConfigPerIna, r);
      VbatC_data = r->data;
      i = x->size[0] * x->size[1];
      x->size[0] = r->size[0];
      x->size[1] = r->size[1];
      emxEnsureCapacity_real_T(x, i);
      sVbat0_data = x->data;
      b_k_ina219 = (((double)k_ina219 + 1.0) - 1.0) * (double)prmBrd_Nbat;
      loop_ub = r->size[0] * r->size[1];
      for (i = 0; i < loop_ub; i++) {
        sVbat0_data[i] = Vbat_data[(int)(VbatC_data[i] + b_k_ina219) - 1];
      }
      combineVectorElements(x, r1);
      VbatC_data = r1->data;
      sumV += VbatC_data[0];
    }
    /*  if BattConfigPerIna{k_ina219}(1) ~= bypass &&
     * BattConfigPerIna{k_ina219}(1) ~= disconnect */
    /*      sumV = sumV + sum(Vbat(BattConfigPerIna{k_ina219} +
     * (k_ina219-1)*Nbat)); */
    /*  end */
  }
  emxFree_real_T(&r1);
  emxFree_real_T(&x);
  emxInit_boolean_T(&b_x, 2);
  i = b_x->size[0] * b_x->size[1];
  b_x->size[0] = 1;
  b_x->size[1] = Sel_size[1];
  emxEnsureCapacity_boolean_T(b_x, i);
  x_data = b_x->data;
  loop_ub = Sel_size[1];
  for (i = 0; i < loop_ub; i++) {
    x_data[i] = (Sel_data[i] == 0.0);
  }
  b = b_any(b_x);
  if (b) {
    k_ina219 = Vbat->size[0];
    loop_ub = Vbat->size[0];
    for (i = 0; i < loop_ub; i++) {
      ind2out_data[i] = (Vbat_data[i] < CutOffDisV);
    }
    UnOvVflag = ((prmSeq.VthUnDis > sumV) && (VthFlag > 0));
  } else {
    b_Sel_size[0] = 1;
    b_Sel_size[1] = Sel_size[1];
    loop_ub = Sel_size[1];
    for (i = 0; i < loop_ub; i++) {
      b_k_ina219 = Sel_data[i];
      b_Sel_data[i] = ((b_k_ina219 >= 1.0) && (b_k_ina219 < 2.0));
    }
    c_Sel_data.data = &b_Sel_data[0];
    c_Sel_data.size = &b_Sel_size[0];
    c_Sel_data.allocatedSize = 2;
    c_Sel_data.numDimensions = 2;
    c_Sel_data.canFreeData = false;
    if (b_any(&c_Sel_data)) {
      k_ina219 = Vbat->size[0];
      loop_ub = Vbat->size[0];
      for (i = 0; i < loop_ub; i++) {
        ind2out_data[i] = (Vbat_data[i] > CutOffChrV);
      }
      UnOvVflag = ((prmSeq.VthOvChr < sumV) && (VthFlag > 0));
    } else {
      k_ina219 = Vbat->size[0];
      loop_ub = Vbat->size[0];
      if (loop_ub - 1 >= 0) {
        memset(&ind2out_data[0], 0, (unsigned int)loop_ub * sizeof(boolean_T));
      }
      UnOvVflag = false;
    }
  }
  i = b_x->size[0] * b_x->size[1];
  b_x->size[0] = 1;
  b_x->size[1] = Sel_size[1];
  emxEnsureCapacity_boolean_T(b_x, i);
  x_data = b_x->data;
  loop_ub = Sel_size[1];
  for (i = 0; i < loop_ub; i++) {
    b_k_ina219 = Sel_data[i];
    x_data[i] = ((b_k_ina219 == prmBrd_spi_bypass) ||
                 (b_k_ina219 == prmBrd_spi_disconnect));
  }
  y = true;
  nx = 1;
  exitg1 = false;
  while ((!exitg1) && (nx <= b_x->size[1])) {
    if (!x_data[nx - 1]) {
      y = false;
      exitg1 = true;
    } else {
      nx++;
    }
  }
  emxFree_boolean_T(&b_x);
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
    emxInit_real_T(&BattConfigPerInaNew, 1);
    emxInit_real_T(&sVbat0, 1);
    emxInit_int32_T(&ind2Stay, 1);
    emxInit_real_T(&C, 2);
    C_data = C->data;
    emxInit_real_T(&Vc, 1);
    emxInit_real_T(&VbatC, 2);
    emxInit_uint16_T(&r2, 1);
    emxInit_real_T(&b_y, 1);
    emxInit_int32_T(&iidx, 1);
    emxInit_uint16_T(&c_y, 2);
    y_data = c_y->data;
    emxInit_real32_T(&d_y, 1);
    emxInit_real32_T(&c_x, 1);
    emxInit_boolean_T(&b_VbatC, 2);
    guard1 = false;
    if (b_any(&d_Sel_data)) {
      guard1 = true;
    } else {
      nx = BattConfigAct_size[0] * BattConfigAct_size[1];
      if (nx == 1) {
        loop_ub = k_ina219;
      } else {
        loop_ub = nx;
      }
      ind2out_size = loop_ub;
      stride_0_0 = (k_ina219 != 1);
      stride_1_0 = (nx != 1);
      for (i = 0; i < loop_ub; i++) {
        b_ind2out_data[i] = (ind2out_data[i * stride_0_0] &&
                             BattConfigAct_data[i * stride_1_0]);
      }
      c_ind2out_data.data = &b_ind2out_data[0];
      c_ind2out_data.size = &ind2out_size;
      c_ind2out_data.allocatedSize = 65534;
      c_ind2out_data.numDimensions = 1;
      c_ind2out_data.canFreeData = false;
      if (any(&c_ind2out_data) || UnOvVflag) {
        guard1 = true;
      }
    }
    if (guard1) {
      loop_ub = BattConfigAct_size[0] * BattConfigAct_size[1];
      if (loop_ub == k_ina219) {
        for (i = 0; i < loop_ub; i++) {
          batAct_data[i] = (BattConfigAct_data[i] && (!ind2out_data[i]));
        }
      } else {
        loop_ub =
            binary_expand_op_11(batAct_data, BattConfigAct_data,
                                BattConfigAct_size, ind2out_data, &k_ina219);
      }
      if (b) {
        if (Vbat->size[0] == loop_ub) {
          i = b_y->size[0];
          b_y->size[0] = Vbat->size[0];
          emxEnsureCapacity_real_T(b_y, i);
          VbatC_data = b_y->data;
          loop_ub = Vbat->size[0];
          for (i = 0; i < loop_ub; i++) {
            VbatC_data[i] = Vbat_data[i] * (double)batAct_data[i];
          }
        } else {
          binary_expand_op_9(b_y, Vbat, batAct_data, &loop_ub);
        }
        c_sort(b_y, iidx);
        iidx_data = iidx->data;
        VbatC_data = b_y->data;
        nx = b_y->size[0] - 1;
        stride_1_0 = 0;
        for (stride_0_0 = 0; stride_0_0 <= nx; stride_0_0++) {
          if (VbatC_data[stride_0_0] > 0.0) {
            stride_1_0++;
          }
        }
        i = r2->size[0];
        r2->size[0] = stride_1_0;
        emxEnsureCapacity_uint16_T(r2, i);
        r3 = r2->data;
        stride_1_0 = 0;
        for (stride_0_0 = 0; stride_0_0 <= nx; stride_0_0++) {
          if (VbatC_data[stride_0_0] > 0.0) {
            r3[stride_1_0] = (unsigned short)stride_0_0;
            stride_1_0++;
          }
        }
        i = sVbat0->size[0];
        sVbat0->size[0] = r2->size[0];
        emxEnsureCapacity_real_T(sVbat0, i);
        sVbat0_data = sVbat0->data;
        loop_ub = r2->size[0];
        for (i = 0; i < loop_ub; i++) {
          sVbat0_data[i] = VbatC_data[r3[i]];
        }
        useConstantDim(sVbat0, nonSingletonDim(sVbat0));
        sVbat0_data = sVbat0->data;
        Nact = r2->size[0];
        switch (VthFlag) {
        case 1:
          ind2out_size = sVbat0->size[0];
          loop_ub = sVbat0->size[0];
          for (i = 0; i < loop_ub; i++) {
            b_ind2out_data[i] = (sVbat0_data[i] > prmSeq.VthDis);
          }
          nx = g_eml_find(b_ind2out_data, ind2out_size, (int *)&findId0_data);
          if (nx == 0) {
            ind2out_size = sVbat0->size[0];
            loop_ub = sVbat0->size[0];
            for (i = 0; i < loop_ub; i++) {
              b_ind2out_data[i] = (sVbat0_data[i] > prmSeq.VthUnDis);
            }
            g_eml_find(b_ind2out_data, ind2out_size, (int *)&findId0_data);
          }
          i = ind2Stay->size[0];
          ind2Stay->size[0] = findId0_data;
          emxEnsureCapacity_int32_T(ind2Stay, i);
          ind2Stay_data = ind2Stay->data;
          for (i = 0; i < findId0_data; i++) {
            ind2Stay_data[i] = iidx_data[r3[i]];
          }
          if (sVbat0_data[findId0_data - 1] > prmSeq.VthOvDis) {
            UnOvVflag = false;
            stride_0_0 = 0;
            exitg1 = false;
            while ((!exitg1) && (stride_0_0 < 3)) {
              b_k_ina219 = (double)findId0_data + (double)stride_0_0;
              if (b_k_ina219 + 4.0 <= Nact) {
                i = (int)(b_k_ina219 + 4.0);
              } else {
                i = Nact;
              }
              if (i < 1) {
                loop_ub = 0;
              } else {
                loop_ub = i;
              }
              i = b_y->size[0];
              b_y->size[0] = loop_ub;
              emxEnsureCapacity_real_T(b_y, i);
              VbatC_data = b_y->data;
              for (i = 0; i < loop_ub; i++) {
                VbatC_data[i] = iidx_data[r3[i]];
              }
              nchoosek(b_y, b_k_ina219, C);
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
              stride_1_0 = 0;
              exitg2 = false;
              while ((!exitg2) && (stride_1_0 <= C->size[0] - 1)) {
                k_nc = stride_1_0;
                i = b_VbatC->size[0] * b_VbatC->size[1];
                b_VbatC->size[0] = 1;
                b_VbatC->size[1] = VbatC->size[1];
                emxEnsureCapacity_boolean_T(b_VbatC, i);
                x_data = b_VbatC->data;
                loop_ub = VbatC->size[1];
                for (i = 0; i < loop_ub; i++) {
                  x_data[i] =
                      ((VbatC_data[stride_1_0 + VbatC->size[0] * i] > VthDis) &&
                       (VbatC_data[stride_1_0 + VbatC->size[0] * i] <
                        VthOvDis));
                }
                f_eml_find(b_VbatC, (int *)&k_ina219, tmp_size);
                if (tmp_size[1] != 0) {
                  UnOvVflag = true;
                  exitg2 = true;
                } else {
                  stride_1_0++;
                }
              }
              if (UnOvVflag) {
                exitg1 = true;
              } else {
                stride_0_0++;
              }
            }
            i = ind2Stay->size[0];
            ind2Stay->size[0] = C->size[1];
            emxEnsureCapacity_int32_T(ind2Stay, i);
            ind2Stay_data = ind2Stay->data;
            loop_ub = C->size[1];
            for (i = 0; i < loop_ub; i++) {
              ind2Stay_data[i] = (int)C_data[k_nc + C->size[0] * i];
            }
          }
          break;
        case 2:
          /* bestfit */
          ind2out_size = sVbat0->size[0];
          loop_ub = sVbat0->size[0];
          for (i = 0; i < loop_ub; i++) {
            b_ind2out_data[i] = (sVbat0_data[i] > prmSeq.VthDis);
          }
          nx = g_eml_find(b_ind2out_data, ind2out_size, (int *)&findId0_data);
          if (nx == 0) {
            ind2out_size = sVbat0->size[0];
            loop_ub = sVbat0->size[0];
            for (i = 0; i < loop_ub; i++) {
              b_ind2out_data[i] = (sVbat0_data[i] > prmSeq.VthUnDis);
            }
            g_eml_find(b_ind2out_data, ind2out_size, (int *)&findId0_data);
          }
          /*                          if csVbat(findId)>VthOvDis */
          stride_0_0 = 0;
          exitg1 = false;
          while ((!exitg1) && (stride_0_0 < 3)) {
            b_k_ina219 = (double)findId0_data + (double)stride_0_0;
            if (b_k_ina219 + 6.0 <= Nact) {
              i = (int)(b_k_ina219 + 6.0);
            } else {
              i = Nact;
            }
            if (i < 1) {
              loop_ub = 0;
            } else {
              loop_ub = i;
            }
            i = b_y->size[0];
            b_y->size[0] = loop_ub;
            emxEnsureCapacity_real_T(b_y, i);
            VbatC_data = b_y->data;
            for (i = 0; i < loop_ub; i++) {
              VbatC_data[i] = iidx_data[r3[i]];
            }
            nchoosek(b_y, b_k_ina219, C);
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
            sum(VbatC, Vc);
            sVbat0_data = Vc->data;
            i = c_x->size[0];
            c_x->size[0] = Vc->size[0];
            emxEnsureCapacity_real32_T(c_x, i);
            b_x_data = c_x->data;
            loop_ub = Vc->size[0];
            for (i = 0; i < loop_ub; i++) {
              b_x_data[i] = (float)sVbat0_data[i] - VthDis;
            }
            nx = c_x->size[0];
            i = d_y->size[0];
            d_y->size[0] = c_x->size[0];
            emxEnsureCapacity_real32_T(d_y, i);
            b_y_data = d_y->data;
            for (stride_1_0 = 0; stride_1_0 < nx; stride_1_0++) {
              b_y_data[stride_1_0] = (float)fabs(b_x_data[stride_1_0]);
            }
            b_minimum(d_y, &nx);
            k_nc = nx - 1;
            /*                                  for k_nc = 1:NC */
            /*                                  idC =
             * find(Vc(k_nc,:)>VthDis&Vc(k_nc,:)<VthOvDis,1,'first'); */
            b_k_ina219 = sVbat0_data[nx - 1];
            if ((b_k_ina219 > VthDis) && (b_k_ina219 < VthOvDis) &&
                (b_k_ina219 > prmSeq.VthUnDis)) {
              exitg1 = true;
            } else {
              /*                                  end */
              stride_0_0++;
            }
          }
          i = ind2Stay->size[0];
          ind2Stay->size[0] = C->size[1];
          emxEnsureCapacity_int32_T(ind2Stay, i);
          ind2Stay_data = ind2Stay->data;
          loop_ub = C->size[1];
          for (i = 0; i < loop_ub; i++) {
            ind2Stay_data[i] = (int)C_data[k_nc + C->size[0] * i];
          }
          /*                          end */
          break;
        case 0:
          /* toggle */
          b_k_ina219 = rt_roundd_snf((double)prmBrd_Nbat * Nina219 -
                                     (double)prmCnfg_NtoggleDrop);
          if (b_k_ina219 < 32768.0) {
            if (b_k_ina219 >= -32768.0) {
              e_y = (short)b_k_ina219;
            } else {
              e_y = MIN_int16_T;
            }
          } else if (b_k_ina219 >= 32768.0) {
            e_y = MAX_int16_T;
          } else {
            e_y = 0;
          }
          if (r2->size[0] <= e_y) {
            e_y = (short)r2->size[0];
          }
          if (e_y < 1) {
            loop_ub = 0;
          } else {
            loop_ub = e_y;
          }
          i = ind2Stay->size[0];
          ind2Stay->size[0] = loop_ub;
          emxEnsureCapacity_int32_T(ind2Stay, i);
          ind2Stay_data = ind2Stay->data;
          for (i = 0; i < loop_ub; i++) {
            ind2Stay_data[i] = iidx_data[r3[i]];
          }
          break;
        default:
          i = ind2Stay->size[0];
          ind2Stay->size[0] = 1;
          emxEnsureCapacity_int32_T(ind2Stay, i);
          ind2Stay_data = ind2Stay->data;
          ind2Stay_data[0] = 0;
          break;
        }
      } else {
        b_Sel_size[0] = 1;
        b_Sel_size[1] = Sel_size[1];
        nx = Sel_size[1];
        for (i = 0; i < nx; i++) {
          b_k_ina219 = Sel_data[i];
          b_Sel_data[i] = ((b_k_ina219 >= 1.0) && (b_k_ina219 <= 2.0));
        }
        e_Sel_data.data = &b_Sel_data[0];
        e_Sel_data.size = &b_Sel_size[0];
        e_Sel_data.allocatedSize = 2;
        e_Sel_data.numDimensions = 2;
        e_Sel_data.canFreeData = false;
        if (b_any(&e_Sel_data)) {
          if (Vbat->size[0] == loop_ub) {
            i = b_y->size[0];
            b_y->size[0] = Vbat->size[0];
            emxEnsureCapacity_real_T(b_y, i);
            VbatC_data = b_y->data;
            loop_ub = Vbat->size[0];
            for (i = 0; i < loop_ub; i++) {
              VbatC_data[i] = Vbat_data[i] * (double)batAct_data[i];
            }
          } else {
            binary_expand_op_9(b_y, Vbat, batAct_data, &loop_ub);
          }
          d_sort(b_y, iidx);
          iidx_data = iidx->data;
          VbatC_data = b_y->data;
          nx = b_y->size[0] - 1;
          stride_1_0 = 0;
          for (stride_0_0 = 0; stride_0_0 <= nx; stride_0_0++) {
            if (VbatC_data[stride_0_0] > 0.0) {
              stride_1_0++;
            }
          }
          i = r2->size[0];
          r2->size[0] = stride_1_0;
          emxEnsureCapacity_uint16_T(r2, i);
          r3 = r2->data;
          stride_1_0 = 0;
          for (stride_0_0 = 0; stride_0_0 <= nx; stride_0_0++) {
            if (VbatC_data[stride_0_0] > 0.0) {
              r3[stride_1_0] = (unsigned short)stride_0_0;
              stride_1_0++;
            }
          }
          i = sVbat0->size[0];
          sVbat0->size[0] = r2->size[0];
          emxEnsureCapacity_real_T(sVbat0, i);
          sVbat0_data = sVbat0->data;
          loop_ub = r2->size[0];
          for (i = 0; i < loop_ub; i++) {
            sVbat0_data[i] = VbatC_data[r3[i]];
          }
          useConstantDim(sVbat0, nonSingletonDim(sVbat0));
          sVbat0_data = sVbat0->data;
          Nact = r2->size[0];
          switch (VthFlag) {
          case 1:
            ind2out_size = sVbat0->size[0];
            loop_ub = sVbat0->size[0];
            for (i = 0; i < loop_ub; i++) {
              b_ind2out_data[i] = (sVbat0_data[i] > prmSeq.VthUnChr);
            }
            stride_1_0 = ind2out_size;
            loop_ub = ind2out_size;
            for (i = 0; i < loop_ub; i++) {
              tmp_data[i] =
                  (b_ind2out_data[i] && (sVbat0_data[i] < prmSeq.VthChr));
            }
            nx = g_eml_find(tmp_data, stride_1_0, (int *)&findId0_data);
            if (nx == 0) {
              loop_ub = ind2out_size;
              for (i = 0; i < loop_ub; i++) {
                b_ind2out_data[i] =
                    (b_ind2out_data[i] && (sVbat0_data[i] < prmSeq.VthOvChr));
              }
              g_eml_find(b_ind2out_data, ind2out_size, (int *)&findId0_data);
            }
            /*                      findId0 = find(csVbat<VthChr,1,'last'); */
            /*                      findIdUn = find(csVbat>VthUnChr,1,'first');
             */
            /*                      findId = max(findId,findIdUn); */
            i = ind2Stay->size[0];
            ind2Stay->size[0] = findId0_data;
            emxEnsureCapacity_int32_T(ind2Stay, i);
            ind2Stay_data = ind2Stay->data;
            for (i = 0; i < findId0_data; i++) {
              ind2Stay_data[i] = iidx_data[r3[i]];
            }
            break;
          case 2:
            ind2out_size = sVbat0->size[0];
            loop_ub = sVbat0->size[0];
            for (i = 0; i < loop_ub; i++) {
              b_ind2out_data[i] = (sVbat0_data[i] < prmSeq.VthChr);
            }
            /*                      findId0 = find(csVbat<VthChr,1,'last'); */
            /*                      findIdUn = find(csVbat>VthUnChr,1,'first');
             */
            /*                      findId = max(findId,findIdUn); */
            h_eml_find(b_ind2out_data, ind2out_size, (int *)&k_ina219);
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
            stride_0_0 = 0;
            exitg1 = false;
            while ((!exitg1) && (stride_0_0 < 3)) {
              b_k_ina219 = (double)k_ina219 + (double)stride_0_0;
              if (b_k_ina219 + 6.0 <= Nact) {
                i = (int)(b_k_ina219 + 6.0);
              } else {
                i = Nact;
              }
              if (i < 1) {
                loop_ub = 0;
              } else {
                loop_ub = i;
              }
              i = b_y->size[0];
              b_y->size[0] = loop_ub;
              emxEnsureCapacity_real_T(b_y, i);
              VbatC_data = b_y->data;
              for (i = 0; i < loop_ub; i++) {
                VbatC_data[i] = iidx_data[r3[i]];
              }
              nchoosek(b_y, b_k_ina219, C);
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
              sum(VbatC, Vc);
              sVbat0_data = Vc->data;
              i = c_x->size[0];
              c_x->size[0] = Vc->size[0];
              emxEnsureCapacity_real32_T(c_x, i);
              b_x_data = c_x->data;
              loop_ub = Vc->size[0];
              for (i = 0; i < loop_ub; i++) {
                b_x_data[i] = (float)sVbat0_data[i] - prmSeq.VthChr;
              }
              nx = c_x->size[0];
              i = d_y->size[0];
              d_y->size[0] = c_x->size[0];
              emxEnsureCapacity_real32_T(d_y, i);
              b_y_data = d_y->data;
              for (stride_1_0 = 0; stride_1_0 < nx; stride_1_0++) {
                b_y_data[stride_1_0] = (float)fabs(b_x_data[stride_1_0]);
              }
              b_minimum(d_y, &nx);
              k_nc = nx - 1;
              /*                                      for k_nc = 1:NC */
              /*                                          idC =
               * (Vc(k_nc,:)<VthChr&Vc(k_nc,:)>VthUnChr,1); */
              b_k_ina219 = sVbat0_data[nx - 1];
              if ((b_k_ina219 > prmSeq.VthUnChr) &&
                  (b_k_ina219 < prmSeq.VthOvChr)) {
                exitg1 = true;
              } else {
                /*                                      end */
                stride_0_0++;
              }
            }
            i = ind2Stay->size[0];
            ind2Stay->size[0] = C->size[1];
            emxEnsureCapacity_int32_T(ind2Stay, i);
            ind2Stay_data = ind2Stay->data;
            loop_ub = C->size[1];
            for (i = 0; i < loop_ub; i++) {
              ind2Stay_data[i] = (int)C_data[k_nc + C->size[0] * i];
            }
            /*                              end */
            /*                          end */
            break;
          case 0:
            /* toggle */
            b_k_ina219 = rt_roundd_snf((double)prmBrd_Nbat * Nina219 -
                                       (double)prmCnfg_NtoggleDrop);
            if (b_k_ina219 < 32768.0) {
              if (b_k_ina219 >= -32768.0) {
                e_y = (short)b_k_ina219;
              } else {
                e_y = MIN_int16_T;
              }
            } else if (b_k_ina219 >= 32768.0) {
              e_y = MAX_int16_T;
            } else {
              e_y = 0;
            }
            if (r2->size[0] <= e_y) {
              e_y = (short)r2->size[0];
            }
            if (e_y < 1) {
              loop_ub = 0;
            } else {
              loop_ub = e_y;
            }
            i = ind2Stay->size[0];
            ind2Stay->size[0] = loop_ub;
            emxEnsureCapacity_int32_T(ind2Stay, i);
            ind2Stay_data = ind2Stay->data;
            for (i = 0; i < loop_ub; i++) {
              ind2Stay_data[i] = iidx_data[r3[i]];
            }
            break;
          default:
            i = ind2Stay->size[0];
            ind2Stay->size[0] = 1;
            emxEnsureCapacity_int32_T(ind2Stay, i);
            ind2Stay_data = ind2Stay->data;
            ind2Stay_data[0] = 0;
            break;
          }
        } else {
          i = ind2Stay->size[0];
          ind2Stay->size[0] = 1;
          emxEnsureCapacity_int32_T(ind2Stay, i);
          ind2Stay_data = ind2Stay->data;
          ind2Stay_data[0] = 0;
        }
      }
      if ((int)Nina219 - 1 >= 0) {
        if (prmBrd_Nbat < 1) {
          c_y->size[0] = 1;
          c_y->size[1] = 0;
        } else {
          i = c_y->size[0] * c_y->size[1];
          c_y->size[0] = 1;
          c_y->size[1] = prmBrd_Nbat;
          emxEnsureCapacity_uint16_T(c_y, i);
          y_data = c_y->data;
          loop_ub = prmBrd_Nbat - 1;
          for (i = 0; i <= loop_ub; i++) {
            y_data[i] = (unsigned short)((unsigned int)i + 1U);
          }
        }
        b_loop_ub = c_y->size[1];
        end = ind2Stay->size[0] - 1;
      }
      for (k_ina219 = 0; k_ina219 < loop_ub_tmp; k_ina219++) {
        /*  BattConfigAct{k_ina219} = batAct([1:Nbat]+(k_ina219-1)*Nbat); */
        b_k_ina219 = (((double)k_ina219 + 1.0) - 1.0) * (double)prmBrd_Nbat;
        for (i = 0; i < b_loop_ub; i++) {
          ind2out_data[i] =
              batAct_data[(int)((double)y_data[i] + b_k_ina219) - 1];
        }
        loop_ub = BattConfigAct_size[0];
        for (i = 0; i < loop_ub; i++) {
          BattConfigAct_data[i + BattConfigAct_size[0] * k_ina219] =
              ind2out_data[i];
        }
        stride_1_0 = 0;
        for (stride_0_0 = 0; stride_0_0 <= end; stride_0_0++) {
          i = ind2Stay_data[stride_0_0];
          if ((b_k_ina219 + 1.0 <= i) &&
              (i <= (double)prmBrd_Nbat + b_k_ina219)) {
            stride_1_0++;
          }
        }
        i = BattConfigPerInaNew->size[0];
        BattConfigPerInaNew->size[0] = stride_1_0;
        emxEnsureCapacity_real_T(BattConfigPerInaNew, i);
        VbatC_data = BattConfigPerInaNew->data;
        stride_1_0 = 0;
        for (stride_0_0 = 0; stride_0_0 <= end; stride_0_0++) {
          i = ind2Stay_data[stride_0_0];
          if ((b_k_ina219 + 1.0 <= i) &&
              (i <= (double)prmBrd_Nbat + b_k_ina219)) {
            VbatC_data[stride_1_0] = i;
            stride_1_0++;
          }
        }
        e_sort(BattConfigPerInaNew);
        VbatC_data = BattConfigPerInaNew->data;
        loop_ub = BattConfigPerInaNew->size[0];
        for (i = 0; i < loop_ub; i++) {
          VbatC_data[i] -= b_k_ina219;
        }
        i = b_BattConfigPerIna->size[0] * b_BattConfigPerIna->size[1] *
            b_BattConfigPerIna->size[2];
        b_BattConfigPerIna->size[0] = 1;
        b_BattConfigPerIna->size[1] = BattConfigPerIna->size[1];
        loop_ub = BattConfigPerIna->size[2];
        b_BattConfigPerIna->size[2] = loop_ub;
        emxEnsureCapacity_real_T(b_BattConfigPerIna, i);
        VbatC_data = b_BattConfigPerIna->data;
        for (i = 0; i < loop_ub; i++) {
          nx = BattConfigPerIna->size[1];
          for (stride_0_0 = 0; stride_0_0 < nx; stride_0_0++) {
            VbatC_data[stride_0_0 + b_BattConfigPerIna->size[1] * i] =
                BattConfigPerIna_data[(k_ina219 +
                                       BattConfigPerIna->size[0] * stride_0_0) +
                                      BattConfigPerIna->size[0] *
                                          BattConfigPerIna->size[1] * i];
          }
        }
        remPadArr(b_BattConfigPerIna, r);
        if (!isequal(r, BattConfigPerInaNew)) {
          if (BattConfigPerInaNew->size[0] != 0) {
            padArr(BattConfigPerInaNew, prmBrd_Nbat, prmBrd_Nbat, r);
            VbatC_data = r->data;
            nx = BattConfigPerIna->size[1];
            stride_1_0 = BattConfigPerIna->size[2];
            for (i = 0; i < stride_1_0; i++) {
              for (stride_0_0 = 0; stride_0_0 < nx; stride_0_0++) {
                BattConfigPerIna_data[(k_ina219 +
                                       BattConfigPerIna->size[0] * stride_0_0) +
                                      BattConfigPerIna->size[0] *
                                          BattConfigPerIna->size[1] * i] =
                    VbatC_data[stride_0_0 + nx * i];
              }
            }
          } else {
            b_padArr(prmBrd_spi_bypass, prmBrd_Nbat, prmBrd_Nbat, r);
            VbatC_data = r->data;
            nx = BattConfigPerIna->size[1];
            stride_1_0 = BattConfigPerIna->size[2];
            for (i = 0; i < stride_1_0; i++) {
              for (stride_0_0 = 0; stride_0_0 < nx; stride_0_0++) {
                BattConfigPerIna_data[(k_ina219 +
                                       BattConfigPerIna->size[0] * stride_0_0) +
                                      BattConfigPerIna->size[0] *
                                          BattConfigPerIna->size[1] * i] =
                    VbatC_data[stride_0_0 + nx * i];
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
    emxFree_boolean_T(&b_VbatC);
    emxFree_real32_T(&c_x);
    emxFree_real32_T(&d_y);
    emxFree_uint16_T(&c_y);
    emxFree_int32_T(&iidx);
    emxFree_real_T(&b_y);
    emxFree_uint16_T(&r2);
    emxFree_real_T(&VbatC);
    emxFree_real_T(&Vc);
    emxFree_real_T(&C);
    emxFree_int32_T(&ind2Stay);
    emxFree_real_T(&sVbat0);
    emxFree_real_T(&BattConfigPerInaNew);
  }
  emxFree_real_T(&b_BattConfigPerIna);
  emxFree_real_T(&r);
  return switchFlag;
}

/*
 * File trailer for Esp32StepSwitchToggleCombAll_ESP_48.c
 *
 * [EOF]
 */
