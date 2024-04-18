/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 31-Mar-2024 07:04:52
 */

/* Include Files */
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229.h"
#include "BuildBitCnfg.h"
#include "CalcItByVout.h"
#include "CalcK.h"
#include "CalcPortSpiPrm.h"
#include "CalcStructKalman.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_data.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_emxutil.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_initialize.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_rtwutil.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h"
#include "ControlKp184.h"
#include "Esp32StepSwitchToggleCombAll_ESP_48.h"
#include "InitKalman.h"
#include "ReadVI.h"
#include "abs.h"
#include "all.h"
#include "allOrAny.h"
#include "any.h"
#include "combineVectorElements.h"
#include "controlJuntekDPH8920.h"
#include "datetime.h"
#include "defineOutStruct.h"
#include "find.h"
#include "ifWhileCond.h"
#include "isequal.h"
#include "ismember.h"
#include "mean.h"
#include "minOrMax.h"
#include "mod.h"
#include "padArr.h"
#include "padArrUint8.h"
#include "pause.h"
#include "polyfit.h"
#include "rand.h"
#include "randn.h"
#include "readI2cVIfastTicTocV5_Rratio_ser.h"
#include "remPadArr.h"
#include "rt_nonfinite.h"
#include "sprintf.h"
#include "squeeze.h"
#include "sum.h"
#include "tic.h"
#include "toc.h"
#include "unique.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* Type Definitions */
#ifndef struct_emxArray_real_T_1x2
#define struct_emxArray_real_T_1x2

struct emxArray_real_T_1x2
{
  double data[2];
  int size[2];
};

#endif                                 /* struct_emxArray_real_T_1x2 */

#ifndef typedef_emxArray_real_T_1x2
#define typedef_emxArray_real_T_1x2

typedef struct emxArray_real_T_1x2 emxArray_real_T_1x2;

#endif                                 /* typedef_emxArray_real_T_1x2 */

/* Function Declarations */
static void binary_expand_op(emxArray_real_T *in1, const signed char in2_data[],
  const int in2_size[2]);
static void binary_expand_op_1(emxArray_real32_T *in1, int in2, const
  emxArray_real_T *in3, const struct0_T *in4);
static void binary_expand_op_2(emxArray_real_T *in1, const struct0_T *in2, const
  int in3_data[], int in4, int in5);
static boolean_T binary_expand_op_3(double in1_data[], int in1_size[2], const
  struct0_T *in2, int in3, emxArray_real_T *in4, const emxArray_real_T *in5,
  const emxArray_uint16_T *in6, const emxArray_real_T *in7, const signed char
  in8_data[], const int *in8_size, int in9, int in10, const emxArray_real_T
  *in11, double in12_data[], int in12_size[2], boolean_T in13_data[], const int
  in13_size[2], boolean_T in14_data[], int in14_size[2]);
static boolean_T binary_expand_op_4(double in1_data[], int in1_size[2], const
  struct0_T *in2, int in3, emxArray_real_T *in4, const emxArray_real_T *in5,
  const emxArray_uint16_T *in6, const emxArray_real_T *in7, const signed char
  in8_data[], const int *in8_size, int in9, int in10, const emxArray_real_T
  *in11, double in12_data[], int in12_size[2], boolean_T in13_data[], const int
  in13_size[2], boolean_T in14_data[], int in14_size[2]);
static float rt_roundf_snf(float u);

/* Function Definitions */
/*
 * Arguments    : emxArray_real_T *in1
 *                const signed char in2_data[]
 *                const int in2_size[2]
 * Return Type  : void
 */
static void binary_expand_op(emxArray_real_T *in1, const signed char in2_data[],
  const int in2_size[2])
{
  emxArray_real_T *b_in1;
  double *b_in1_data;
  double *in1_data;
  int aux_0_1;
  int aux_1_1;
  int b_loop_ub;
  int i;
  int i1;
  int loop_ub;
  int stride_0_1;
  int stride_1_1;
  in1_data = in1->data;
  emxInit_real_T(&b_in1, 2);
  i = b_in1->size[0] * b_in1->size[1];
  b_in1->size[0] = in1->size[0];
  if (in2_size[1] == 1) {
    loop_ub = in1->size[1];
  } else {
    loop_ub = in2_size[1];
  }

  b_in1->size[1] = loop_ub;
  emxEnsureCapacity_real_T(b_in1, i);
  b_in1_data = b_in1->data;
  stride_0_1 = (in1->size[1] != 1);
  stride_1_1 = (in2_size[1] != 1);
  aux_0_1 = 0;
  aux_1_1 = 0;
  for (i = 0; i < loop_ub; i++) {
    b_loop_ub = in1->size[0];
    for (i1 = 0; i1 < b_loop_ub; i1++) {
      b_in1_data[i1 + b_in1->size[0] * i] = in1_data[i1 + in1->size[0] * aux_0_1]
        * (double)in2_data[aux_1_1];
    }

    aux_1_1 += stride_1_1;
    aux_0_1 += stride_0_1;
  }

  i = in1->size[0] * in1->size[1];
  in1->size[0] = b_in1->size[0];
  in1->size[1] = b_in1->size[1];
  emxEnsureCapacity_real_T(in1, i);
  in1_data = in1->data;
  loop_ub = b_in1->size[1];
  for (i = 0; i < loop_ub; i++) {
    b_loop_ub = b_in1->size[0];
    for (i1 = 0; i1 < b_loop_ub; i1++) {
      in1_data[i1 + in1->size[0] * i] = b_in1_data[i1 + b_in1->size[0] * i];
    }
  }

  emxFree_real_T(&b_in1);
}

/*
 * Arguments    : emxArray_real32_T *in1
 *                int in2
 *                const emxArray_real_T *in3
 *                const struct0_T *in4
 * Return Type  : void
 */
static void binary_expand_op_1(emxArray_real32_T *in1, int in2, const
  emxArray_real_T *in3, const struct0_T *in4)
{
  const double *in3_data;
  float *in1_data;
  int i;
  int loop_ub;
  int stride_0_1;
  in3_data = in3->data;
  in1_data = in1->data;
  stride_0_1 = (in3->size[1] != 1);
  loop_ub = in1->size[1];
  for (i = 0; i < loop_ub; i++) {
    in1_data[in2 + in1->size[0] * i] = (float)in3_data[in2 + in3->size[0] * (i *
      stride_0_1)] - in4->bat.Rint[in2 + (i << 1)];
  }
}

/*
 * Arguments    : emxArray_real_T *in1
 *                const struct0_T *in2
 *                const int in3_data[]
 *                int in4
 *                int in5
 * Return Type  : void
 */
static void binary_expand_op_2(emxArray_real_T *in1, const struct0_T *in2, const
  int in3_data[], int in4, int in5)
{
  emxArray_real_T *b_in1;
  double *b_in1_data;
  double *in1_data;
  int aux_0_1;
  int aux_1_1;
  int b_loop_ub;
  int i;
  int i1;
  int in3;
  int loop_ub;
  int stride_0_1;
  int stride_1_1;
  in1_data = in1->data;
  in3 = in3_data[in4];
  emxInit_real_T(&b_in1, 2);
  i = b_in1->size[0] * b_in1->size[1];
  b_in1->size[0] = in1->size[0];
  if (in5 + 1 == 1) {
    loop_ub = in1->size[1];
  } else {
    loop_ub = in5 + 1;
  }

  b_in1->size[1] = loop_ub;
  emxEnsureCapacity_real_T(b_in1, i);
  b_in1_data = b_in1->data;
  stride_0_1 = (in1->size[1] != 1);
  stride_1_1 = (in5 + 1 != 1);
  aux_0_1 = 0;
  aux_1_1 = 0;
  for (i = 0; i < loop_ub; i++) {
    b_loop_ub = in1->size[0];
    for (i1 = 0; i1 < b_loop_ub; i1++) {
      b_in1_data[i1 + b_in1->size[0] * i] = in1_data[i1 + in1->size[0] * aux_0_1]
        * (double)in2->seq.tst.i.NegIflag[(in3 + (aux_1_1 << 3)) - 1];
    }

    aux_1_1 += stride_1_1;
    aux_0_1 += stride_0_1;
  }

  i = in1->size[0] * in1->size[1];
  in1->size[0] = b_in1->size[0];
  in1->size[1] = b_in1->size[1];
  emxEnsureCapacity_real_T(in1, i);
  in1_data = in1->data;
  loop_ub = b_in1->size[1];
  for (i = 0; i < loop_ub; i++) {
    b_loop_ub = b_in1->size[0];
    for (i1 = 0; i1 < b_loop_ub; i1++) {
      in1_data[i1 + in1->size[0] * i] = b_in1_data[i1 + b_in1->size[0] * i];
    }
  }

  emxFree_real_T(&b_in1);
}

/*
 * Arguments    : double in1_data[]
 *                int in1_size[2]
 *                const struct0_T *in2
 *                int in3
 *                emxArray_real_T *in4
 *                const emxArray_real_T *in5
 *                const emxArray_uint16_T *in6
 *                const emxArray_real_T *in7
 *                const signed char in8_data[]
 *                const int *in8_size
 *                int in9
 *                int in10
 *                const emxArray_real_T *in11
 *                double in12_data[]
 *                int in12_size[2]
 *                boolean_T in13_data[]
 *                const int in13_size[2]
 *                boolean_T in14_data[]
 *                int in14_size[2]
 * Return Type  : boolean_T
 */
static boolean_T binary_expand_op_3(double in1_data[], int in1_size[2], const
  struct0_T *in2, int in3, emxArray_real_T *in4, const emxArray_real_T *in5,
  const emxArray_uint16_T *in6, const emxArray_real_T *in7, const signed char
  in8_data[], const int *in8_size, int in9, int in10, const emxArray_real_T
  *in11, double in12_data[], int in12_size[2], boolean_T in13_data[], const int
  in13_size[2], boolean_T in14_data[], int in14_size[2])
{
  emxArray_int32_T *b_in6;
  emxArray_real_T *b_in5;
  double b_in10[2];
  const double *in11_data;
  const double *in5_data;
  const double *in7_data;
  double *b_in5_data;
  float b_in11_data[2];
  int in11_size[2];
  int i;
  int loop_ub;
  int stride_0_0;
  int stride_1_0;
  int *b_in6_data;
  const unsigned short *in6_data;
  boolean_T out1;
  in11_data = in11->data;
  in7_data = in7->data;
  in6_data = in6->data;
  in5_data = in5->data;
  b_in10[0] = (double)(in10 + 1) - 1.0;
  b_in10[1] = 1.0;
  emxInit_int32_T(&b_in6, 1);
  if (*in8_size == 1) {
    loop_ub = in6->size[1];
  } else {
    loop_ub = *in8_size;
  }

  i = b_in6->size[0];
  b_in6->size[0] = loop_ub;
  emxEnsureCapacity_int32_T(b_in6, i);
  b_in6_data = b_in6->data;
  stride_0_0 = (in6->size[1] != 1);
  stride_1_0 = (*in8_size != 1);
  for (i = 0; i < loop_ub; i++) {
    b_in6_data[i] = (int)((double)in6_data[i * stride_0_0] +
                          (in7_data[in8_data[i * stride_1_0]] - 1.0) * (double)
                          in9) - 1;
  }

  i = (int)b_maximum(b_in10);
  emxInit_real_T(&b_in5, 1);
  stride_0_0 = b_in5->size[0];
  b_in5->size[0] = b_in6->size[0];
  emxEnsureCapacity_real_T(b_in5, stride_0_0);
  b_in5_data = b_in5->data;
  loop_ub = b_in6->size[0];
  for (stride_0_0 = 0; stride_0_0 < loop_ub; stride_0_0++) {
    b_in5_data[stride_0_0] = in5_data[b_in6_data[stride_0_0] + in5->size[0] * (i
      - 1)];
  }

  emxFree_int32_T(&b_in6);
  in11_size[0] = 1;
  in11_size[1] = *in8_size;
  for (i = 0; i < *in8_size; i++) {
    b_in11_data[i] = ((float)in11_data[(int)in7_data[in8_data[i]] - 1] +
                      in2->cnfg.Ttoggle) + 1.0F;
  }

  i = (int)in2->seq.vth[in3] - 1;
  out1 = c_Esp32StepSwitchToggleCombAll_(in1_data, in1_size, in2->
    bat.CutOffChrV[i], in2->bat.CutOffDisV[i], in4, b_in5, *in8_size,
    in2->brd.Nbat, in2->brd.spi.disconnect, in2->brd.spi.bypass,
    in2->cnfg.Ttoggle, in2->cnfg.NtoggleDrop, in2->cnfg.minLenIna219,
    in2->seq.pwr, in2->seq.pwr.VthFlag[in3], b_in11_data, in11_size, in12_data,
    in12_size, in13_data, in13_size, in14_data, in14_size);
  emxFree_real_T(&b_in5);
  return out1;
}

/*
 * Arguments    : double in1_data[]
 *                int in1_size[2]
 *                const struct0_T *in2
 *                int in3
 *                emxArray_real_T *in4
 *                const emxArray_real_T *in5
 *                const emxArray_uint16_T *in6
 *                const emxArray_real_T *in7
 *                const signed char in8_data[]
 *                const int *in8_size
 *                int in9
 *                int in10
 *                const emxArray_real_T *in11
 *                double in12_data[]
 *                int in12_size[2]
 *                boolean_T in13_data[]
 *                const int in13_size[2]
 *                boolean_T in14_data[]
 *                int in14_size[2]
 * Return Type  : boolean_T
 */
static boolean_T binary_expand_op_4(double in1_data[], int in1_size[2], const
  struct0_T *in2, int in3, emxArray_real_T *in4, const emxArray_real_T *in5,
  const emxArray_uint16_T *in6, const emxArray_real_T *in7, const signed char
  in8_data[], const int *in8_size, int in9, int in10, const emxArray_real_T
  *in11, double in12_data[], int in12_size[2], boolean_T in13_data[], const int
  in13_size[2], boolean_T in14_data[], int in14_size[2])
{
  emxArray_int32_T *b_in6;
  emxArray_real_T *b_in5;
  const double *in11_data;
  const double *in5_data;
  const double *in7_data;
  double *b_in5_data;
  int i;
  int loop_ub;
  int stride_0_0;
  int stride_1_0;
  int *b_in6_data;
  const unsigned short *in6_data;
  boolean_T out1;
  in11_data = in11->data;
  in7_data = in7->data;
  in6_data = in6->data;
  in5_data = in5->data;
  emxInit_int32_T(&b_in6, 1);
  if (*in8_size == 1) {
    loop_ub = in6->size[1];
  } else {
    loop_ub = *in8_size;
  }

  i = b_in6->size[0];
  b_in6->size[0] = loop_ub;
  emxEnsureCapacity_int32_T(b_in6, i);
  b_in6_data = b_in6->data;
  stride_0_0 = (in6->size[1] != 1);
  stride_1_0 = (*in8_size != 1);
  for (i = 0; i < loop_ub; i++) {
    b_in6_data[i] = (int)((double)in6_data[i * stride_0_0] +
                          (in7_data[in8_data[i * stride_1_0]] - 1.0) * (double)
                          in9) - 1;
  }

  emxInit_real_T(&b_in5, 1);
  i = b_in5->size[0];
  b_in5->size[0] = b_in6->size[0];
  emxEnsureCapacity_real_T(b_in5, i);
  b_in5_data = b_in5->data;
  loop_ub = b_in6->size[0];
  for (i = 0; i < loop_ub; i++) {
    b_in5_data[i] = in5_data[b_in6_data[i] + in5->size[0] * in10];
  }

  emxFree_int32_T(&b_in6);
  i = (int)in2->seq.vth[in3] - 1;
  out1 = d_Esp32StepSwitchToggleCombAll_(in1_data, in1_size, in2->
    bat.CutOffChrV[i], in2->bat.CutOffDisV[i], in4, b_in5, *in8_size,
    in2->brd.Nbat, in2->brd.spi.disconnect, in2->brd.spi.bypass,
    in2->cnfg.Ttoggle, in2->cnfg.NtoggleDrop, in2->cnfg.minLenIna219,
    in2->seq.pwr, in2->seq.pwr.VthFlag[in3], in11_data[in11->size[0] *
    in11->size[1] * in10], in12_data, in12_size, in13_data, in13_size, in14_data,
    in14_size);
  emxFree_real_T(&b_in5);
  return out1;
}

/*
 * Arguments    : float u
 * Return Type  : float
 */
static float rt_roundf_snf(float u)
{
  float y;
  if ((float)fabs(u) < 8.388608E+6F) {
    if (u >= 0.5F) {
      y = (float)floor(u + 0.5F);
    } else if (u > -0.5F) {
      y = u * 0.0F;
    } else {
      y = (float)ceil(u - 0.5F);
    }
  } else {
    y = u;
  }

  return y;
}

/*
 * Arguments    : struct0_T *prm
 *                struct30_T *outStruct
 * Return Type  : void
 */
void Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229(struct0_T *prm, struct30_T
  *outStruct)
{
  static short j_tmp_data[32767];
  static short m_tmp_data[32767];
  static short n_tmp_data[32767];
  static unsigned char b_y_data[32767];
  static boolean_T p_tmp_data[65534];
  static boolean_T l_tmp_data[32767];
  emxArray_boolean_T b_SelDual_nm1_data;
  emxArray_boolean_T b_VbusTest0_data;
  emxArray_boolean_T c_SelDual_nm1_data;
  emxArray_boolean_T c_VbusTest0_data;
  emxArray_boolean_T d_SelDual_nm1_data;
  emxArray_boolean_T d_VbusTest_data;
  emxArray_boolean_T e_VbusTest_data;
  emxArray_boolean_T f_VbusTest_data;
  emxArray_boolean_T g_VbusTest_data;
  emxArray_boolean_T *BattConfigAct;
  emxArray_boolean_T *b_changeConfigFlag;
  emxArray_boolean_T *b_x;
  emxArray_boolean_T *changeConfigFlag;
  emxArray_boolean_T *switchFlag;
  emxArray_boolean_T *switchFlagHelp;
  emxArray_char_T *IchargeAct;
  emxArray_char_T *Vcharge0;
  emxArray_char_T *b_IchargeAct;
  emxArray_char_T *b_IshuntTest2;
  emxArray_char_T *b_VbusTest_data;
  emxArray_char_T *b_Vcharge0;
  emxArray_char_T *b_VmKp184Test_data;
  emxArray_char_T *b_k_Ttest;
  emxArray_char_T *b_k_bat;
  emxArray_char_T *b_prm;
  emxArray_char_T *b_vSumMax;
  emxArray_char_T *c_IchargeAct;
  emxArray_char_T *c_VbusTest2;
  emxArray_char_T *c_Vcharge0;
  emxArray_char_T *c_prm;
  emxArray_char_T *d_IchargeAct;
  emxArray_char_T *d_Vcharge0;
  emxArray_char_T *d_prm;
  emxArray_char_T *dv0;
  emxArray_char_T *e_IchargeAct;
  emxArray_char_T *f_IchargeAct;
  emxArray_char_T *k_Ttest;
  emxArray_char_T *k_bat;
  emxArray_char_T *meanIbrd0;
  emxArray_char_T *r10;
  emxArray_char_T *r11;
  emxArray_char_T *r12;
  emxArray_char_T *r13;
  emxArray_char_T *r14;
  emxArray_char_T *r15;
  emxArray_char_T *r16;
  emxArray_char_T *r17;
  emxArray_char_T *r18;
  emxArray_char_T *r19;
  emxArray_char_T *r20;
  emxArray_char_T *r21;
  emxArray_char_T *r22;
  emxArray_char_T *r23;
  emxArray_char_T *r24;
  emxArray_char_T *r25;
  emxArray_char_T *r26;
  emxArray_char_T *r27;
  emxArray_char_T *r28;
  emxArray_char_T *r29;
  emxArray_char_T *r30;
  emxArray_char_T *r5;
  emxArray_char_T *r6;
  emxArray_char_T *r7;
  emxArray_char_T *r8;
  emxArray_char_T *r9;
  emxArray_char_T *vSumMax;
  emxArray_int32_T *r3;
  emxArray_int8_T *b_Vbat;
  emxArray_real32_T *Rwire;
  emxArray_real32_T *a__36;
  emxArray_real32_T *a__37;
  emxArray_real32_T *a__64;
  emxArray_real32_T *a__67;
  emxArray_real32_T *b_Rwire;
  emxArray_real32_T *pIacs758;
  emxArray_real_T b_ImKp184Test2_0;
  emxArray_real_T c_ImKp184Test2_bat_0;
  emxArray_real_T c_VdebugVec_data;
  emxArray_real_T *BattConfigPerIna;
  emxArray_real_T *BattConfigPerInaHelp;
  emxArray_real_T *Iacs758_cal;
  emxArray_real_T *Iacs758_cal0;
  emxArray_real_T *IbatMat;
  emxArray_real_T *Ibrd;
  emxArray_real_T *ImKp184Test2;
  emxArray_real_T *ImKp184Test2_0;
  emxArray_real_T *ImKp184Test2_bat_0;
  emxArray_real_T *IshuntTest2;
  emxArray_real_T *Rtot;
  emxArray_real_T *RtotHelpPoly;
  emxArray_real_T *SelDual;
  emxArray_real_T *SelDualHelp;
  emxArray_real_T *SelDual_nm1;
  emxArray_real_T *Vbat;
  emxArray_real_T *VbatMat;
  emxArray_real_T *VbatMat0;
  emxArray_real_T *VbitInsMeas;
  emxArray_real_T *Vbrd;
  emxArray_real_T *VbusTest0;
  emxArray_real_T *VbusTest2;
  emxArray_real_T *Vdebug;
  emxArray_real_T *Vdebug1N;
  emxArray_real_T *VecIna219;
  emxArray_real_T *VmV;
  emxArray_real_T *a__38;
  emxArray_real_T *a__39;
  emxArray_real_T *a__40;
  emxArray_real_T *a__42;
  emxArray_real_T *a__45;
  emxArray_real_T *a__65;
  emxArray_real_T *a__66;
  emxArray_real_T *a__68;
  emxArray_real_T *a__69;
  emxArray_real_T *a__70;
  emxArray_real_T *a__71;
  emxArray_real_T *a__72;
  emxArray_real_T *a__73;
  emxArray_real_T *b_BattConfigPerInaHelp;
  emxArray_real_T *b_I;
  emxArray_real_T *b_ImKp184Test2_bat_0;
  emxArray_real_T *b_VbusTest2;
  emxArray_real_T *meanIbrd;
  emxArray_real_T *pIshunt;
  emxArray_real_T *r;
  emxArray_real_T *r1;
  emxArray_real_T *r2;
  emxArray_real_T *r4;
  emxArray_real_T *tLastToggle;
  emxArray_real_T *tV;
  emxArray_real_T *tV1;
  emxArray_real_T *tvv;
  emxArray_real_T *x;
  emxArray_real_T_1x2 r31;
  emxArray_uint16_T *y;
  emxArray_uint8_T c_bitCnfg;
  emxArray_uint8_T *Pac2Vid0All;
  emxArray_uint8_T *Pac2Vid31;
  emxArray_uint8_T *b_bitCnfg;
  emxArray_uint8_T *bitCnfg;
  creal_T t2_data;
  double VdebugVec_data[1024];
  double kEst_data[1024];
  double VbusTest_data[512];
  double VmKp184Test_data[512];
  double e_tmp_data[127];
  double g_tmp_data[127];
  double b_VdebugVec_data[64];
  double VbatMat_data[32];
  double b_IbatMat_data[32];
  double c_VmKp184Test_data[32];
  double b_tvv[16];
  double tii[16];
  double a__1_data[8];
  double dateVec[6];
  double b_k_t0[2];
  double o_tmp_data[2];
  double outVI[2];
  double N0;
  double Nina219_tmp_tmp;
  double V_chr_RI;
  double V_dis_all;
  double a__16_data;
  double a__3_data;
  double b_meanIbrd0;
  double d;
  double d1;
  double errI2C_data;
  double g_IchargeAct;
  double k_t;
  double toc00;
  double *BattConfigPerInaHelp_data;
  double *BattConfigPerIna_data;
  double *Iacs758_cal_data;
  double *IbatMat_data;
  double *ImKp184Test2_data;
  double *IshuntTest2_data;
  double *RtotHelpPoly_data;
  double *Rtot_data;
  double *SelDualHelp_data;
  double *SelDual_data;
  double *VbusTest2_data;
  double *Vdebug_data;
  double *VecIna219_data;
  double *VmV_data;
  double *pIshunt_data;
  double *tLastToggle_data;
  double *tV_data;
  float varargin_1[1024];
  float f_prm[496];
  float h_prm[38];
  float fv[31];
  float e_prm[16];
  float i_prm[2];
  float Icharge;
  float IchargePhase2_tmp_tmp;
  float IchargePhase2t;
  float IchargePhase3;
  float ImaxAcDC;
  float ImaxChrB2B;
  float ImaxDis;
  float Ta;
  float c_vSumMax;
  float e_Vcharge0;
  float f;
  float juntekEfficencyFactor;
  float *Rwire_data;
  float *b_Rwire_data;
  float *pIacs758_data;
  int VdebugVec_size[3];
  int IbatMat_size[2];
  int Nina219[2];
  int SelDual_nm1_size[2];
  int VmKp184Test_size[2];
  int a__16_size[2];
  int a__3_size[2];
  int a__4_size[2];
  int b_Nina219[2];
  int b_tmp_size[2];
  int c_Nina219[2];
  int c_tmp_size[2];
  int d_Nina219[2];
  int d_tmp_size[2];
  int e_Nina219[2];
  int e_tmp_size[2];
  int errI2C_size[2];
  int f_Nina219[2];
  int id2Bypass_data[2];
  int ipos_data[2];
  int nextStatePerGroup_size[2];
  int tLastToggle_size[2];
  int tmp_size[2];
  int N_bat_tmp_tmp_tmp;
  int VbusTest0_size;
  int VbusTest_size;
  int VbusTest_size_idx_0;
  int VdebugVec_size_idx_0_tmp;
  int VmKp184Test_size_idx_0;
  int VmKp184Test_size_idx_1;
  int b_VecIna219;
  int b_end;
  int b_i;
  int b_k_tstState;
  int b_loop_ub;
  int b_loop_ub_tmp;
  int b_size;
  int b_trueCount;
  int c_ImKp184Test2_0;
  int c_VecIna219;
  int c_end;
  int c_k_Ttest;
  int c_k_bat;
  int c_loop_ub;
  int c_trueCount;
  int d_ImKp184Test2_0;
  int d_end;
  int e_end;
  int end;
  int end_tmp;
  int i;
  int i1;
  int i2;
  int i3;
  int i4;
  int id2Bypass_size;
  int ipos_size;
  int k0;
  int k_groups;
  int k_ina219_;
  int k_t0;
  int k_tstState;
  int loop_ub;
  int loop_ub_tmp;
  int state_k;
  int trueCount;
  int *r32;
  short b_data[2];
  short prm_data[2];
  short N_bat1;
  short N_bitCnfg;
  short i5;
  unsigned short u;
  unsigned short *y_data;
  unsigned char a__2[256];
  signed char h_tmp_data[255];
  signed char i_tmp_data[255];
  unsigned char disConAll[24];
  signed char g_prm[16];
  signed char iv[8];
  signed char b_tmp_data[2];
  char bitCnfgStr[2];
  signed char c_tmp_data[2];
  signed char d_tmp_data[2];
  signed char f_tmp_data[2];
  signed char k_tmp_data[2];
  signed char q_tmp_data[2];
  signed char r_tmp_data[2];
  signed char s_tmp_data[2];
  signed char t_tmp_data[2];
  signed char tmp_data[2];
  signed char u_tmp_data[2];
  signed char Iacs758Flag;
  unsigned char NstTst_tmp_tmp;
  signed char ProjectFlag;
  signed char k_groups_tmp;
  unsigned char *Pac2Vid0All_data;
  unsigned char *Pac2Vid31_data;
  signed char *Vbat_data;
  unsigned char *bitCnfg_data;
  boolean_T VbusTest0_data[32385];
  boolean_T c_VbusTest_data[512];
  boolean_T b_prm_data[256];
  boolean_T ismember0_data[255];
  boolean_T AllBypassPerGroup_data[2];
  boolean_T SelDual_nm1_data[2];
  boolean_T b_VecIna219_data[2];
  boolean_T nextStatePerGroup_data[2];
  boolean_T b_guard1;
  boolean_T exitg1;
  boolean_T exitg2;
  boolean_T guard1;
  boolean_T guard2;
  boolean_T guard3;
  boolean_T guard4;
  boolean_T guard5;
  boolean_T kalmanFlag;
  boolean_T keepMeas;
  boolean_T newState;
  boolean_T onPowerFlag;
  boolean_T showVdiff;
  boolean_T *BattConfigAct_data;
  boolean_T *b_changeConfigFlag_data;
  boolean_T *changeConfigFlag_data;
  boolean_T *switchFlagHelp_data;
  boolean_T *switchFlag_data;
  if (!isInitialized_Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229) {
    Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_initialize();
  }

  outStruct->VpassFlag = defineOutStruct(prm->Nmax.NbrdMax, prm->Nmax.NbatMax,
    outStruct->VbusTest, outStruct->VmKp184Test, &outStruct->VresetFlag,
    &outStruct->Rint);

  /*  []; */
  /*  init */
  ProjectFlag = prm->ins.ProjectFlag;

  /*  disconnect All */
  Nina219_tmp_tmp = prm->brd.Nina219;

  /* length(Esp32_v1); */
  pause(1.0);

  /* repmat(prm.struct.KalmanStruct,Nina219);%nadav Coder */
  VbusTest_size_idx_0 = (int)prm->brd.Nina219;
  for (k_ina219_ = 0; k_ina219_ < VbusTest_size_idx_0; k_ina219_++) {
    /*      [disConAll,~,~] = CalcPortSpiPrm(prm.brd.spi,prm.brd.spi.disconnect); */
    switch (ProjectFlag) {
     case 2:
      /* esp32 */
      break;

     case 3:
      /* esp32 ser */
      /*  writePortToSpi4RowMask_ser(disConAll,Esp32_v1(k_ina219),false); */
      /* UNTITLED Summary of this function goes here */
      /*    Detailed explanation goes here */
      /*  if coder.target('MATLAB') */
      /*  read current */
      /*   chip1     |   chip2     |   chip3 */
      /* %comN2 Port4 chip1,comN1 Port11 chip1 */
      /* %comP1 Port18 chip1 ,comP2 Port18 chip3 */
      /*  */
      /*  for k_row = 1:N_row */
      /*      out123(k_row,:) = readSPI(SPI,PortSpiRow(k_row,1),N_chip); */
      /*      oldPortSpiRow(k_row,1:2:end) = PortSpiRow(k_row,1:2:end); */
      /*      oldPortSpiRow(k_row,2:2:end) = out123(k_row,2:2:end); */
      /*  end */
      /* mask current */
      /* write current mask */
      /*  for k_row = 1:N_row */
      /*      write(SPI,oldPortSpiRowMask(k_row,:)); */
      /*  end */
      pause(0.001);

      /* mask new */
      /* write mask new */
      /*  for k_row = 1:N_row */
      /*      write(SPI0,PortSpiRowMask(k_row,:)); */
      /*  end */
      /* write new */
      /*  for k_row = 1:N_row */
      /*      write(SPI0,PortSpiRow(k_row,:)); */
      /*  end */
      /*  Rmsg = SPI_writeread(SPI0, PortSpiRow); */
      /*  ReadPortSpiRow = RmsgData([[2:4] , 1] ,:); */
      /*  ReadPortSpiRow(:,1:2:end) = ReadPortSpiRow(:,1:2:end);% - 128; */
      /* mask current */
      /* %chip1 Port6 Alert1, chip1 port 7 Alert3, chip1 port 8 Alert8, chip1 port 9 Alert6 */
      /* %chip1 Port12 Alert7, chip1 Port13 Alert5, chip 1 port19 NC */
      /* %chip1 Port20-23 NC */
      /* chip1 Port28 Alert1, chip1 Port29 Alert2 */
      /*  if sum(sum(sum(abs(double(PortSpiRow)-double(ReadPortSpiRow)))))>0 */
      /*  end */
      break;
    }
  }

  N_bat_tmp_tmp_tmp = prm->brd.N_bat;
  N_bat1 = prm->brd.N_bat1;

  /*  juntek_Flag        = prm.ins.juntek; */
  /*  juntekOnlyChr_Flag = prm.ins.juntekOnlyChr_Flag; */
  /*  OnlyDis_Flag       = prm.ins.OnlyDis_Flag; */
  /*  CalibrateBaordFlag = prm.ins.CalibrateBaordFlag; */
  /*  PwSwFalg           = prm.ins.PwSwFalg; */
  /*  SwitchMat_esp   = prm.brd.spi.SwitchMat_esp; */
  /*  SwitchMat_esp2  = prm.brd.spi.SwitchMat_esp2; */
  /*  PortSpiRow_esp  = prm.brd.spi.PortSpiRow_esp; */
  /*  PortSpiRow_esp2 = prm.brd.spi.PortSpiRow_esp2; */
  /*  Pac2Vid         = prm.brd.spi.Pac2Vid; */
  /*  Pac2Vid2        = prm.brd.spi.Pac2Vid2; */
  kalmanFlag = prm->klm.kalmanFlag;
  Ta = prm->klm.Ta;
  Iacs758Flag = prm->brd.pac.Iacs758Flag;

  /*  seq             = prm.seq(prm.run.seq); */
  /*  vTestFlag       = seq.tst.v; */
  /*  ITestFlag       = seq.tst.i; */
  ImaxAcDC = prm->ins.prm.jun.ImaxAcDC;

  /* 15;%35;%maximum c ACDC current [A] */
  emxInit_real32_T(&pIacs758, 3);
  i = pIacs758->size[0] * pIacs758->size[1] * pIacs758->size[2];
  pIacs758->size[0] = 2;
  pIacs758->size[1] = 1;
  pIacs758->size[2] = 2;
  emxEnsureCapacity_real32_T(pIacs758, i);
  pIacs758_data = pIacs758->data;
  pIacs758_data[0] = prm->brd.pac.pIacs758[0];
  pIacs758_data[1] = prm->brd.pac.pIacs758[1];
  pIacs758_data[2] = prm->brd.pac.pIacs758[2];
  pIacs758_data[3] = prm->brd.pac.pIacs758[3];
  emxInit_real32_T(&Rwire, 2);
  if (prm->brd.useRwireFlag) {
    i = Rwire->size[0] * Rwire->size[1];
    Rwire->size[0] = 2;
    Rwire->size[1] = 16;
    emxEnsureCapacity_real32_T(Rwire, i);
    Rwire_data = Rwire->data;
    for (i = 0; i < 32; i++) {
      Rwire_data[i] = prm->brd.Rwire[i];
    }
  } else {
    i = Rwire->size[0] * Rwire->size[1];
    Rwire->size[0] = (int)prm->brd.Nina219;
    Rwire->size[1] = prm->brd.N_bat;
    emxEnsureCapacity_real32_T(Rwire, i);
    Rwire_data = Rwire->data;
    loop_ub = (int)prm->brd.Nina219 * prm->brd.N_bat;
    for (i = 0; i < loop_ub; i++) {
      Rwire_data[i] = 0.0F;
    }
  }

  juntekEfficencyFactor = prm->ins.prm.jun.juntekEfficencyFactor;

  /* 0.85; */
  trueCount = 0;
  if (prm->ser.com.grp[0] > 0) {
    trueCount = 1;
  }

  if (prm->ser.com.grp[1] > 0) {
    trueCount++;
  }

  loop_ub = 0;
  if (prm->ser.com.grp[0] > 0) {
    tmp_data[0] = 0;
    loop_ub = 1;
  }

  if (prm->ser.com.grp[1] > 0) {
    tmp_data[loop_ub] = 1;
  }

  end_tmp = trueCount - 1;
  trueCount = 0;
  loop_ub = 0;
  for (b_i = 0; b_i <= end_tmp; b_i++) {
    i = prm->ser.com.grp[tmp_data[b_i]];
    if (i > 0) {
      trueCount++;
      b_tmp_data[loop_ub] = (signed char)b_i;
      loop_ub++;
    }
  }

  for (i = 0; i < trueCount; i++) {
    prm_data[i] = prm->ser.com.grp[tmp_data[b_tmp_data[i]]];
  }

  b_size = unique_vector(prm_data, trueCount, b_data, id2Bypass_data,
    &id2Bypass_size, ipos_data, &ipos_size);
  loop_ub_tmp = (int)prm->brd.Nina219 * prm->brd.N_bat;
  VmKp184Test_size_idx_0 = (int)prm->brd.Nina219;
  VmKp184Test_size_idx_1 = prm->brd.N_bat;
  if (loop_ub_tmp - 1 >= 0) {
    memset(&VbusTest_data[0], 0, (unsigned int)loop_ub_tmp * sizeof(double));
    memset(&VmKp184Test_data[0], 0, (unsigned int)loop_ub_tmp * sizeof(double));
  }

  /*  if juntek_Flag */
  /*  %     [s_juntek] = Init_juntekDPH8920(COM_juntek); */
  /*      vTestFlag = false; */
  /*      ImaxAcDC = prm.ins.jun.ImaxAcDC;%15;%35;%maximum c ACDC current [A] */
  /*      ITestFlag = false; */
  /*      prm.brd.pac.pIacs758 = pIacs758; */
  /*      prm.brd.pac.Rval     = Rval; */
  /*      if PwSwFalg */
  /*          [s_PwrSw] = Init_PwrSw(prm.ser.com.COM_PwrSw); */
  /*      end */
  /*      s_PwrSw = prm.ser.s_PwrSw; */
  /*      if juntekOnlyChr_Flag */
  /*          AcDcFlag = true; */
  /*      else */
  /*          AcDcFlag = false; */
  /*      end */
  /*  elseif ~OnlyDis_Flag */
  /*      s_kp184 = prm.ser.s_kp184; */
  /*      s_ka6005p = prm.ser.s_ka6005p; */
  /*      vTestFlag = true; */
  /*      ITestFlag = true; */
  /*      Rval     = prm.brd.pac.Rval; */
  /*      pIacs758 = prm.brd.pac.pIacs758; */
  /*  elseif OnlyDis_Flag */
  /*      vTestFlag = false; */
  /*      ITestFlag = false; */
  /*      Rval     = prm.brd.pac.Rval; */
  /*      pIacs758 = prm.brd.pac.pIacs758; */
  /*  end */
  /* init parameters */
  /* true;%false;%true; */
  /* 60;%sec FIX (ORG 60) */
  /* [1:Nbat]';%ORG([1:N_bat-NtoggleDrop]');%[1:N_bat]';%[1 ; 2 ; 3 ; 4]; */
  /* [1:N_bat-NtoggleDrop]';%[1:N_bat]';%[1 ; 2 ; 3 ; 4];%[1 , 2 , 3 , 4]; */
  /* [1:N_bat-NtoggleDrop]';%[1:N_bat]';%[1 ; 2 ; 3 ; 4]; */
  /* [1:N_bat]';%[1 ; 2 ; 3 ; 4]; */
  /*  */
  /*  for k_ina219 = 1:Nina219 %Dis Configuration */
  /*      BattConfigDis{k_ina219,1} = BattConfigDis1 + (k_ina219-1)*N_bat;%Serial */
  /*      BattConfigDisPerIna{k_ina219,1} = BattConfigDis1; */
  /*  end */
  /*  for k_ina219 = 1:Nina219 %Chr Configuration */
  /*      BattConfigChr{k_ina219,1} = BattConfigChr1 + (k_ina219-1)*N_bat;%Parallel */
  /*      BattConfigChrPerIna{k_ina219,1} = BattConfigChr1; */
  /*  end */
  /*  for k_ina219 = 1:Nina219 */
  /*      BattConfigAct{k_ina219,1} = true(N_bat,1); */
  /*  end */
  emxInit_boolean_T(&BattConfigAct, 2);
  i = BattConfigAct->size[0] * BattConfigAct->size[1];
  BattConfigAct->size[0] = prm->brd.N_bat;
  BattConfigAct->size[1] = (int)prm->brd.Nina219;
  emxEnsureCapacity_boolean_T(BattConfigAct, i);
  BattConfigAct_data = BattConfigAct->data;
  for (i = 0; i < loop_ub_tmp; i++) {
    BattConfigAct_data[i] = true;
  }

  pause(1.0);

  /* sum(seqTstV.grp>0); */
  outStruct->VpassFlag = true;
  outStruct->VresetFlag = true;
  emxInit_real_T(&VmV, 1);
  emxInit_real_T(&Vdebug, 2);
  emxInit_real_T(&Vdebug1N, 2);
  emxInit_real_T(&VbusTest2, 4);
  emxInit_real_T(&IshuntTest2, 4);
  emxInit_real_T(&ImKp184Test2, 4);
  emxInit_real_T(&Iacs758_cal, 4);
  emxInit_real_T(&Rtot, 2);
  emxInit_real_T(&pIshunt, 3);
  emxInit_real_T(&RtotHelpPoly, 3);
  emxInit_real_T(&Iacs758_cal0, 2);
  emxInit_real_T(&ImKp184Test2_0, 2);
  emxInit_real_T(&ImKp184Test2_bat_0, 2);
  emxInit_uint8_T(&Pac2Vid0All, 3);
  emxInit_real_T(&Vbrd, 5);
  emxInit_real_T(&Ibrd, 5);
  emxInit_real_T(&meanIbrd, 4);
  emxInit_real_T(&VbitInsMeas, 4);
  emxInit_real32_T(&a__36, 1);
  emxInit_real32_T(&a__37, 2);
  emxInit_real_T(&a__38, 2);
  emxInit_real_T(&a__39, 2);
  emxInit_real_T(&a__40, 2);
  emxInit_real_T(&a__42, 2);
  emxInit_real_T(&a__45, 3);
  emxInit_real_T(&Vbat, 2);
  emxInit_real_T(&IbatMat, 3);
  emxInit_real_T(&VbatMat, 3);
  emxInit_real_T(&VbatMat0, 3);
  emxInit_real_T(&tV1, 2);
  emxInit_real_T(&tV, 3);
  emxInit_real_T(&SelDual, 2);
  emxInit_real_T(&SelDualHelp, 2);
  emxInit_real_T(&tLastToggle, 2);
  emxInit_boolean_T(&switchFlag, 2);
  emxInit_boolean_T(&switchFlagHelp, 2);
  emxInit_boolean_T(&changeConfigFlag, 2);
  emxInit_real_T(&BattConfigPerIna, 3);
  emxInit_real32_T(&a__64, 1);
  emxInit_real_T(&a__65, 1);
  emxInit_real_T(&a__66, 1);
  emxInit_real32_T(&a__67, 2);
  emxInit_real_T(&a__68, 2);
  emxInit_real_T(&a__69, 2);
  emxInit_real_T(&a__70, 1);
  emxInit_real_T(&a__71, 2);
  emxInit_real_T(&a__72, 1);
  emxInit_real_T(&a__73, 1);
  emxInit_real_T(&BattConfigPerInaHelp, 3);
  emxInit_real_T(&SelDual_nm1, 2);
  emxInit_uint8_T(&bitCnfg, 3);
  emxInit_real_T(&r, 1);
  emxInit_real_T(&r1, 3);
  emxInit_real_T(&tvv, 1);
  emxInit_real_T(&VbusTest0, 1);
  emxInit_uint8_T(&Pac2Vid31, 2);
  emxInit_real_T(&VecIna219, 2);
  VecIna219_data = VecIna219->data;
  emxInit_real_T(&r2, 2);
  emxInit_int8_T(&b_Vbat);
  emxInit_real_T(&b_I, 1);
  emxInit_int32_T(&r3, 1);
  emxInit_uint16_T(&y, 2);
  y_data = y->data;
  emxInit_real_T(&x, 4);
  emxInit_boolean_T(&b_x, 2);
  emxInit_real_T(&b_VbusTest2, 4);
  emxInit_uint8_T(&b_bitCnfg, 3);
  emxInit_real_T(&b_ImKp184Test2_bat_0, 2);
  emxInit_real32_T(&b_Rwire, 2);
  emxInit_boolean_T(&b_changeConfigFlag, 2);
  emxInit_real_T(&b_BattConfigPerInaHelp, 3);
  emxInit_real_T(&r4, 2);
  emxInit_char_T(&r5);
  emxInit_char_T(&r6);
  emxInit_char_T(&k_bat);
  emxInit_char_T(&b_VbusTest_data);
  emxInit_char_T(&b_k_bat);
  emxInit_char_T(&r7);
  emxInit_char_T(&b_VmKp184Test_data);
  emxInit_char_T(&vSumMax);
  emxInit_char_T(&r8);
  emxInit_char_T(&b_prm);
  emxInit_char_T(&r9);
  emxInit_char_T(&r10);
  emxInit_char_T(&r11);
  emxInit_char_T(&c_prm);
  emxInit_char_T(&r12);
  emxInit_char_T(&k_Ttest);
  emxInit_char_T(&c_VbusTest2);
  emxInit_char_T(&b_k_Ttest);
  emxInit_char_T(&b_IshuntTest2);
  emxInit_char_T(&b_vSumMax);
  emxInit_char_T(&d_prm);
  emxInit_char_T(&r13);
  emxInit_char_T(&r14);
  emxInit_char_T(&meanIbrd0);
  emxInit_char_T(&r15);
  emxInit_char_T(&r16);
  emxInit_char_T(&r17);
  emxInit_char_T(&dv0);
  emxInit_char_T(&r18);
  emxInit_char_T(&r19);
  emxInit_char_T(&Vcharge0);
  emxInit_char_T(&b_Vcharge0);
  emxInit_char_T(&IchargeAct);
  emxInit_char_T(&b_IchargeAct);
  emxInit_char_T(&r20);
  emxInit_char_T(&r21);
  emxInit_char_T(&r22);
  emxInit_char_T(&r23);
  emxInit_char_T(&r24);
  emxInit_char_T(&r25);
  emxInit_char_T(&c_IchargeAct);
  emxInit_char_T(&d_IchargeAct);
  emxInit_char_T(&e_IchargeAct);
  emxInit_char_T(&r26);
  emxInit_char_T(&f_IchargeAct);
  emxInit_char_T(&r27);
  emxInit_char_T(&c_Vcharge0);
  emxInit_char_T(&r28);
  emxInit_char_T(&r29);
  emxInit_char_T(&d_Vcharge0);
  emxInit_char_T(&r30);
  guard1 = false;
  guard2 = false;
  guard3 = false;
  guard4 = false;
  if (prm->seq.tst.v.isTest) {
    tic();

    /*  NTtest = 2; */
    /*  if prm.ins.swm */
    /*      Control_SwChDisLoad(prm.ser.s_swm,seqTstV.swm); */
    /*  end */
    VdebugVec_size_idx_0_tmp = (int)prm->brd.Nina219;
    k_ina219_ = prm->brd.N_bat;
    b_loop_ub_tmp = loop_ub_tmp << 1;
    if (b_loop_ub_tmp - 1 >= 0) {
      memset(&VdebugVec_data[0], 0, (unsigned int)b_loop_ub_tmp * sizeof(double));
    }

    if (prm->brd.Nina219 < 1.0) {
      VecIna219->size[0] = 1;
      VecIna219->size[1] = 0;
    } else {
      i = VecIna219->size[0] * VecIna219->size[1];
      VecIna219->size[0] = 1;
      VecIna219->size[1] = (int)(prm->brd.Nina219 - 1.0) + 1;
      emxEnsureCapacity_real_T(VecIna219, i);
      VecIna219_data = VecIna219->data;
      loop_ub = (int)(prm->brd.Nina219 - 1.0);
      for (i = 0; i <= loop_ub; i++) {
        VecIna219_data[i] = (double)i + 1.0;
      }
    }

    if (b_loop_ub_tmp - 1 >= 0) {
      memset(&kEst_data[0], 0, (unsigned int)b_loop_ub_tmp * sizeof(double));
    }

    /*  [~,~,~,~]          = ControlKp184(s_kp184,'Read',[]); */
    i = prm->seq.tst.v.Nst;
    if (prm->seq.tst.v.Nst - 1 >= 0) {
      end = ipos_size - 1;
    }

    for (k_tstState = 0; k_tstState < i; k_tstState++) {
      pause(0.1);

      /* 1:length(uGroups) */
      trueCount = 0;
      loop_ub = 0;

      /* VecIna219_k = find(k_groups == uGroupId); */
      state_k = 0;
      for (b_i = 0; b_i <= end; b_i++) {
        k_groups_tmp = prm->seq.tst.v.grp[k_tstState];
        i1 = ipos_data[b_i];
        if (k_groups_tmp == i1) {
          trueCount++;
          c_tmp_data[loop_ub] = (signed char)b_i;
          loop_ub++;
        }

        if (k_groups_tmp != i1) {
          state_k++;
        }
      }

      for (k0 = 0; k0 < state_k; k0++) {
        /* k_ina219not = VecIna219_notk %diconnect group that not tested */
        switch (ProjectFlag) {
         case 2:
          /* ESP32 */
          break;

         case 3:
          /* ESP32 ser */
          /* UNTITLED Summary of this function goes here */
          /*    Detailed explanation goes here */
          /*  if coder.target('MATLAB') */
          /*  read current */
          /*   chip1     |   chip2     |   chip3 */
          /* %comN2 Port4 chip1,comN1 Port11 chip1 */
          /* %comP1 Port18 chip1 ,comP2 Port18 chip3 */
          /*  */
          /*  for k_row = 1:N_row */
          /*      out123(k_row,:) = readSPI(SPI,PortSpiRow(k_row,1),N_chip); */
          /*      oldPortSpiRow(k_row,1:2:end) = PortSpiRow(k_row,1:2:end); */
          /*      oldPortSpiRow(k_row,2:2:end) = out123(k_row,2:2:end); */
          /*  end */
          /* mask current */
          /* write current mask */
          /*  for k_row = 1:N_row */
          /*      write(SPI,oldPortSpiRowMask(k_row,:)); */
          /*  end */
          pause(0.001);

          /* mask new */
          /* write mask new */
          /*  for k_row = 1:N_row */
          /*      write(SPI0,PortSpiRowMask(k_row,:)); */
          /*  end */
          /* write new */
          /*  for k_row = 1:N_row */
          /*      write(SPI0,PortSpiRow(k_row,:)); */
          /*  end */
          /*  Rmsg = SPI_writeread(SPI0, PortSpiRow); */
          /*  ReadPortSpiRow = RmsgData([[2:4] , 1] ,:); */
          /*  ReadPortSpiRow(:,1:2:end) = ReadPortSpiRow(:,1:2:end);% - 128; */
          /* mask current */
          /* %chip1 Port6 Alert1, chip1 port 7 Alert3, chip1 port 8 Alert8, chip1 port 9 Alert6 */
          /* %chip1 Port12 Alert7, chip1 Port13 Alert5, chip 1 port19 NC */
          /* %chip1 Port20-23 NC */
          /* chip1 Port28 Alert1, chip1 Port29 Alert2 */
          /*  if sum(sum(sum(abs(double(PortSpiRow)-double(ReadPortSpiRow)))))>0 */
          /*  end */
          break;
        }

        pause(0.1);
      }

      if (trueCount - 1 >= 0) {
        b_loop_ub = trueCount;
        i2 = N_bat_tmp_tmp_tmp;
        if (N_bat_tmp_tmp_tmp < 1) {
          Nina219[1] = 0;
          y->size[0] = 1;
          y->size[1] = 0;
        } else {
          Nina219[1] = N_bat_tmp_tmp_tmp;
          i1 = y->size[0] * y->size[1];
          y->size[0] = 1;
          y->size[1] = N_bat_tmp_tmp_tmp;
          emxEnsureCapacity_uint16_T(y, i1);
          y_data = y->data;
          loop_ub = N_bat_tmp_tmp_tmp - 1;
          for (i1 = 0; i1 <= loop_ub; i1++) {
            y_data[i1] = (unsigned short)((unsigned int)i1 + 1U);
          }
        }

        k_t0 = Nina219[1];
      }

      for (k0 = 0; k0 < trueCount; k0++) {
        /* k_ina219 = VecIna219_k */
        V_chr_RI = VecIna219_data[c_tmp_data[k0]];
        for (i1 = 0; i1 < b_loop_ub; i1++) {
          b_VecIna219_data[i1] = (VecIna219_data[c_tmp_data[i1]] != V_chr_RI);
        }

        id2Bypass_size = c_eml_find(b_VecIna219_data, trueCount, id2Bypass_data);
        for (b_i = 0; b_i < id2Bypass_size; b_i++) {
          /* k_bypass = VecIna219_k(id2Bypass) */
          /*  ina219StateAll(:,k_bypass)   = ina219StateBypass; */
          /*  BattConfig{k_bypass}       = prm.brd.spi.bypass;%-2; */
          /*  BattConfigPerIna{k_bypass} = prm.brd.spi.bypass;%-2; */
          d = VecIna219_data[c_tmp_data[id2Bypass_data[b_i] - 1]];
          if (d < 2.147483648E+9) {
            i1 = (int)d;
          } else {
            i1 = MAX_int32_T;
          }

          b_sprintf(i1, r5);
          switch (ProjectFlag) {
           case 0:
            /* ina219 */
            break;

           case 1:
            /* EVBOTS_v1 */
            break;

           case 2:
            /* ESP32 */
            break;

           case 3:
            /* ESP32 ser */
            /* UNTITLED Summary of this function goes here */
            /*    Detailed explanation goes here */
            /*  if coder.target('MATLAB') */
            /*  read current */
            /*   chip1     |   chip2     |   chip3 */
            /* %comN2 Port4 chip1,comN1 Port11 chip1 */
            /* %comP1 Port18 chip1 ,comP2 Port18 chip3 */
            /*  */
            /*  for k_row = 1:N_row */
            /*      out123(k_row,:) = readSPI(SPI,PortSpiRow(k_row,1),N_chip); */
            /*      oldPortSpiRow(k_row,1:2:end) = PortSpiRow(k_row,1:2:end); */
            /*      oldPortSpiRow(k_row,2:2:end) = out123(k_row,2:2:end); */
            /*  end */
            /* mask current */
            /* write current mask */
            /*  for k_row = 1:N_row */
            /*      write(SPI,oldPortSpiRowMask(k_row,:)); */
            /*  end */
            pause(0.001);

            /* mask new */
            /* write mask new */
            /*  for k_row = 1:N_row */
            /*      write(SPI0,PortSpiRowMask(k_row,:)); */
            /*  end */
            /* write new */
            /*  for k_row = 1:N_row */
            /*      write(SPI0,PortSpiRow(k_row,:)); */
            /*  end */
            /*  Rmsg = SPI_writeread(SPI0, PortSpiRow); */
            /*  ReadPortSpiRow = RmsgData([[2:4] , 1] ,:); */
            /*  ReadPortSpiRow(:,1:2:end) = ReadPortSpiRow(:,1:2:end);% - 128; */
            /* mask current */
            /* %chip1 Port6 Alert1, chip1 port 7 Alert3, chip1 port 8 Alert8, chip1 port 9 Alert6 */
            /* %chip1 Port12 Alert7, chip1 Port13 Alert5, chip 1 port19 NC */
            /* %chip1 Port20-23 NC */
            /* chip1 Port28 Alert1, chip1 Port29 Alert2 */
            /*  if sum(sum(sum(abs(double(PortSpiRow)-double(ReadPortSpiRow)))))>0 */
            /*  end */
            break;
          }

          pause(0.1);
        }

        /*  for k_bypass = VecIna219(id2Bypass) */
        if (i2 - 1 >= 0) {
          if (V_chr_RI < 2.147483648E+9) {
            i3 = (int)V_chr_RI;
            c_k_Ttest = (int)V_chr_RI;
          } else {
            i3 = MAX_int32_T;
            c_k_Ttest = MAX_int32_T;
          }
        }

        for (c_k_bat = 0; c_k_bat < i2; c_k_bat++) {
          if (c_k_bat + 1 < 256) {
            NstTst_tmp_tmp = (unsigned char)((double)c_k_bat + 1.0);
          } else {
            NstTst_tmp_tmp = MAX_uint8_T;
          }

          padArrUint8(NstTst_tmp_tmp, prm->Nmax.NbatMax, prm->Nmax.NbatMax,
                      Pac2Vid31);
          CalcPortSpiPrm(prm->brd.spi.disconnect, prm->brd.spi.bypass,
                         prm->brd.spi.PortSpiRow_esp, prm->brd.spi.SwitchMat_esp,
                         prm->brd.spi.PortSpiRow_esp2,
                         prm->brd.spi.SwitchMat_esp2, prm->brd.spi.Pac2Vid,
                         prm->brd.spi.Pac2Vid2, Pac2Vid31, disConAll, a__1_data,
                         id2Bypass_data, a__2);

          /*                  ina219State = findSwitch(SwitchCell,k_bat);%8+(k_bat-1); */
          /*  BattConfig{k_ina219}       = k_bat + (k_ina219-1)*N_bat; */
          /*  BattConfigPerIna{k_ina219} = k_bat; */
          /*  ina219StateAll(:,k_ina219)   = ina219State; */
          b_sprintf(i3, r6);
          i1 = id2Bypass_data[0];
          for (b_loop_ub_tmp = 0; b_loop_ub_tmp < i1; b_loop_ub_tmp++) {
            d = rt_roundd_snf(a__1_data[b_loop_ub_tmp]);
            if (d < 2.147483648E+9) {
              if (d >= -2.147483648E+9) {
                id2Bypass_size = (int)d;
              } else {
                id2Bypass_size = MIN_int32_T;
              }
            } else if (d >= 2.147483648E+9) {
              id2Bypass_size = MAX_int32_T;
            } else {
              id2Bypass_size = 0;
            }

            printf("%d \n", id2Bypass_size);
            fflush(stdout);
          }

          switch (ProjectFlag) {
           case 2:
            /* ESP32 */
            break;

           case 3:
            /* ESP32 ser */
            /* UNTITLED Summary of this function goes here */
            /*    Detailed explanation goes here */
            /*  if coder.target('MATLAB') */
            /*  read current */
            /*   chip1     |   chip2     |   chip3 */
            /* %comN2 Port4 chip1,comN1 Port11 chip1 */
            /* %comP1 Port18 chip1 ,comP2 Port18 chip3 */
            /*  */
            /*  for k_row = 1:N_row */
            /*      out123(k_row,:) = readSPI(SPI,PortSpiRow(k_row,1),N_chip); */
            /*      oldPortSpiRow(k_row,1:2:end) = PortSpiRow(k_row,1:2:end); */
            /*      oldPortSpiRow(k_row,2:2:end) = out123(k_row,2:2:end); */
            /*  end */
            /* mask current */
            /* write current mask */
            /*  for k_row = 1:N_row */
            /*      write(SPI,oldPortSpiRowMask(k_row,:)); */
            /*  end */
            pause(0.001);

            /* mask new */
            /* write mask new */
            /*  for k_row = 1:N_row */
            /*      write(SPI0,PortSpiRowMask(k_row,:)); */
            /*  end */
            /* write new */
            /*  for k_row = 1:N_row */
            /*      write(SPI0,PortSpiRow(k_row,:)); */
            /*  end */
            /*  Rmsg = SPI_writeread(SPI0, PortSpiRow); */
            /*  ReadPortSpiRow = RmsgData([[2:4] , 1] ,:); */
            /*  ReadPortSpiRow(:,1:2:end) = ReadPortSpiRow(:,1:2:end);% - 128; */
            /* mask current */
            /* %chip1 Port6 Alert1, chip1 port 7 Alert3, chip1 port 8 Alert8, chip1 port 9 Alert6 */
            /* %chip1 Port12 Alert7, chip1 Port13 Alert5, chip 1 port19 NC */
            /* %chip1 Port20-23 NC */
            /* chip1 Port28 Alert1, chip1 Port29 Alert2 */
            /*  if sum(sum(sum(abs(double(PortSpiRow)-double(ReadPortSpiRow)))))>0 */
            /*  end */
            break;
          }

          pause(0.5);
          switch (ProjectFlag) {
           case 2:
            /* ESP32 */
            i1 = VmV->size[0];
            VmV->size[0] = 16;
            emxEnsureCapacity_real_T(VmV, i1);
            VmV_data = VmV->data;
            for (i1 = 0; i1 < 16; i1++) {
              VmV_data[i1] = 0.0;
            }

            i1 = Vdebug->size[0] * Vdebug->size[1];
            Vdebug->size[0] = 16;
            Vdebug->size[1] = 2;
            emxEnsureCapacity_real_T(Vdebug, i1);
            Vdebug_data = Vdebug->data;
            for (i1 = 0; i1 < 32; i1++) {
              Vdebug_data[i1] = 0.0;
            }

            /*  */
            break;

           case 3:
            /* ESP32 ser */
            for (i1 = 0; i1 < 16; i1++) {
              g_prm[i1] = prm->brd.pac.VIpacId[((int)V_chr_RI + (i1 << 1)) - 1];
            }

            c_readI2cVIfastTicTocV5_Rratio_(g_prm, N_bat1, VmV, b_I, b_tvv, tii,
              (double *)&errI2C_data, errI2C_size, Vdebug);
            Vdebug_data = Vdebug->data;
            VmV_data = VmV->data;

            /*  */
            break;

           default:
            i1 = VmV->size[0];
            VmV->size[0] = N_bat_tmp_tmp_tmp;
            emxEnsureCapacity_real_T(VmV, i1);
            VmV_data = VmV->data;
            i1 = Vdebug->size[0] * Vdebug->size[1];
            Vdebug->size[0] = N_bat_tmp_tmp_tmp;
            Vdebug->size[1] = 1;
            emxEnsureCapacity_real_T(Vdebug, i1);
            Vdebug_data = Vdebug->data;
            for (i1 = 0; i1 < N_bat_tmp_tmp_tmp; i1++) {
              VmV_data[i1] = 0.0;
              Vdebug_data[i1] = 0.0;
            }
            break;
          }

          VbusTest_data[((int)V_chr_RI + (int)Nina219_tmp_tmp * c_k_bat) - 1] =
            VmV_data[c_k_bat];
          id2Bypass_size = (int)V_chr_RI + VdebugVec_size_idx_0_tmp * c_k_bat;
          VdebugVec_data[id2Bypass_size - 1] = Vdebug_data[c_k_bat];
          VdebugVec_data[(id2Bypass_size + VdebugVec_size_idx_0_tmp * k_ina219_)
            - 1] = Vdebug_data[c_k_bat + Vdebug->size[0]];
          c_sprintf(c_k_Ttest, c_k_bat + 1, k_bat);
          d_sprintf(VbusTest_data[((int)V_chr_RI + (int)Nina219_tmp_tmp *
                     c_k_bat) - 1], b_VbusTest_data);
          pause(0.1);
          if (prm->seq.tst.v.ins[k_tstState] == 1) {
            /* kp184 */
            ControlKp184((double *)&a__3_data, a__3_size, (double *)&b_meanIbrd0,
                         a__4_size, (double *)&N0, tmp_size, (double *)
                         &errI2C_data, errI2C_size);
            VmKp184Test_data[((int)V_chr_RI + (int)Nina219_tmp_tmp * c_k_bat) -
              1] = N0;
            if (V_chr_RI < 2.147483648E+9) {
              i1 = (int)V_chr_RI;
            } else {
              i1 = MAX_int32_T;
            }

            c_sprintf(i1, c_k_bat + 1, b_k_bat);
            d_sprintf(VmKp184Test_data[((int)V_chr_RI + (int)Nina219_tmp_tmp *
                       c_k_bat) - 1], b_VmKp184Test_data);
            pause(0.1);
          }
        }

        /*  for k_bat = 1:N_bat */
        for (i1 = 0; i1 < k_t0; i1++) {
          u = y_data[i1];
          if (u > 255) {
            u = 255U;
          }

          b_y_data[i1] = (unsigned char)u;
        }

        c_padArrUint8(b_y_data, Nina219[1], prm->Nmax.NbatMax, prm->Nmax.NbatMax,
                      Pac2Vid31);
        CalcPortSpiPrm(prm->brd.spi.disconnect, prm->brd.spi.bypass,
                       prm->brd.spi.PortSpiRow_esp, prm->brd.spi.SwitchMat_esp,
                       prm->brd.spi.PortSpiRow_esp2, prm->brd.spi.SwitchMat_esp2,
                       prm->brd.spi.Pac2Vid, prm->brd.spi.Pac2Vid2, Pac2Vid31,
                       disConAll, a__1_data, id2Bypass_data, a__2);
        switch (ProjectFlag) {
         case 2:
          /* ESP32 */
          break;

         case 3:
          /* ESP32 ser */
          /* UNTITLED Summary of this function goes here */
          /*    Detailed explanation goes here */
          /*  if coder.target('MATLAB') */
          /*  read current */
          /*   chip1     |   chip2     |   chip3 */
          /* %comN2 Port4 chip1,comN1 Port11 chip1 */
          /* %comP1 Port18 chip1 ,comP2 Port18 chip3 */
          /*  */
          /*  for k_row = 1:N_row */
          /*      out123(k_row,:) = readSPI(SPI,PortSpiRow(k_row,1),N_chip); */
          /*      oldPortSpiRow(k_row,1:2:end) = PortSpiRow(k_row,1:2:end); */
          /*      oldPortSpiRow(k_row,2:2:end) = out123(k_row,2:2:end); */
          /*  end */
          /* mask current */
          /* write current mask */
          /*  for k_row = 1:N_row */
          /*      write(SPI,oldPortSpiRowMask(k_row,:)); */
          /*  end */
          pause(0.001);

          /* mask new */
          /* write mask new */
          /*  for k_row = 1:N_row */
          /*      write(SPI0,PortSpiRowMask(k_row,:)); */
          /*  end */
          /* write new */
          /*  for k_row = 1:N_row */
          /*      write(SPI0,PortSpiRow(k_row,:)); */
          /*  end */
          /*  Rmsg = SPI_writeread(SPI0, PortSpiRow); */
          /*  ReadPortSpiRow = RmsgData([[2:4] , 1] ,:); */
          /*  ReadPortSpiRow(:,1:2:end) = ReadPortSpiRow(:,1:2:end);% - 128; */
          /* mask current */
          /* %chip1 Port6 Alert1, chip1 port 7 Alert3, chip1 port 8 Alert8, chip1 port 9 Alert6 */
          /* %chip1 Port12 Alert7, chip1 Port13 Alert5, chip 1 port19 NC */
          /* %chip1 Port20-23 NC */
          /* chip1 Port28 Alert1, chip1 Port29 Alert2 */
          /*  if sum(sum(sum(abs(double(PortSpiRow)-double(ReadPortSpiRow)))))>0 */
          /*  end */
          break;
        }

        pause(0.1);
        switch (ProjectFlag) {
         case 2:
          /* ESP32 */
          i1 = Vdebug->size[0] * Vdebug->size[1];
          Vdebug->size[0] = 16;
          Vdebug->size[1] = 2;
          emxEnsureCapacity_real_T(Vdebug, i1);
          Vdebug_data = Vdebug->data;
          for (i1 = 0; i1 < 32; i1++) {
            Vdebug_data[i1] = 0.0;
          }

          /*  */
          break;

         case 3:
          /* ESP32 ser */
          for (i1 = 0; i1 < 16; i1++) {
            g_prm[i1] = prm->brd.pac.VIpacId[((int)V_chr_RI + (i1 << 1)) - 1];
          }

          c_readI2cVIfastTicTocV5_Rratio_(g_prm, N_bat1, VmV, tvv, b_tvv, tii,
            (double *)&a__3_data, a__3_size, Vdebug);

          /*  */
          break;

         default:
          i1 = Vdebug->size[0] * Vdebug->size[1];
          Vdebug->size[0] = N_bat_tmp_tmp_tmp;
          Vdebug->size[1] = 1;
          emxEnsureCapacity_real_T(Vdebug, i1);
          Vdebug_data = Vdebug->data;
          for (i1 = 0; i1 < N_bat_tmp_tmp_tmp; i1++) {
            Vdebug_data[i1] = 0.0;
          }
          break;
        }

        if (prm->seq.tst.v.ins[k_tstState] == 1) {
          /* kp184 */
          ControlKp184((double *)&a__3_data, a__3_size, (double *)&b_meanIbrd0,
                       a__4_size, (double *)&errI2C_data, errI2C_size, (double *)
                       &a__16_data, a__16_size);
        }

        /*  [SpiPortRow0,ina219State,Pac2Vid0] = CalcPortSpi(SwitchMat_esp,SwitchMat_esp2,PortSpiRow_esp,PortSpiRow_esp2,Pac2Vid,Pac2Vid2,[1+N_bat/2,N_bat]'); */
        switch (ProjectFlag) {
         case 2:
          /* ESP32 */
          break;

         case 3:
          /* ESP32 ser */
          /* UNTITLED Summary of this function goes here */
          /*    Detailed explanation goes here */
          /*  if coder.target('MATLAB') */
          /*  read current */
          /*   chip1     |   chip2     |   chip3 */
          /* %comN2 Port4 chip1,comN1 Port11 chip1 */
          /* %comP1 Port18 chip1 ,comP2 Port18 chip3 */
          /*  */
          /*  for k_row = 1:N_row */
          /*      out123(k_row,:) = readSPI(SPI,PortSpiRow(k_row,1),N_chip); */
          /*      oldPortSpiRow(k_row,1:2:end) = PortSpiRow(k_row,1:2:end); */
          /*      oldPortSpiRow(k_row,2:2:end) = out123(k_row,2:2:end); */
          /*  end */
          /* mask current */
          /* write current mask */
          /*  for k_row = 1:N_row */
          /*      write(SPI,oldPortSpiRowMask(k_row,:)); */
          /*  end */
          pause(0.001);

          /* mask new */
          /* write mask new */
          /*  for k_row = 1:N_row */
          /*      write(SPI0,PortSpiRowMask(k_row,:)); */
          /*  end */
          /* write new */
          /*  for k_row = 1:N_row */
          /*      write(SPI0,PortSpiRow(k_row,:)); */
          /*  end */
          /*  Rmsg = SPI_writeread(SPI0, PortSpiRow); */
          /*  ReadPortSpiRow = RmsgData([[2:4] , 1] ,:); */
          /*  ReadPortSpiRow(:,1:2:end) = ReadPortSpiRow(:,1:2:end);% - 128; */
          /* mask current */
          /* %chip1 Port6 Alert1, chip1 port 7 Alert3, chip1 port 8 Alert8, chip1 port 9 Alert6 */
          /* %chip1 Port12 Alert7, chip1 Port13 Alert5, chip 1 port19 NC */
          /* %chip1 Port20-23 NC */
          /* chip1 Port28 Alert1, chip1 Port29 Alert2 */
          /*  if sum(sum(sum(abs(double(PortSpiRow)-double(ReadPortSpiRow)))))>0 */
          /*  end */
          break;
        }

        pause(0.1);
        switch (ProjectFlag) {
         case 2:
          /*  */
          /* ESP32 */
          break;

         case 3:
          /* ESP32 ser */
          for (i1 = 0; i1 < 16; i1++) {
            g_prm[i1] = prm->brd.pac.VIpacId[((int)V_chr_RI + (i1 << 1)) - 1];
          }

          c_readI2cVIfastTicTocV5_Rratio_(g_prm, N_bat1, VmV, tvv, b_tvv, tii,
            (double *)&a__3_data, a__3_size, Vdebug1N);

          /*  */
          break;
        }

        if (prm->seq.tst.v.isPrm) {
          VdebugVec_size[0] = 1;
          VdebugVec_size[1] = k_ina219_;
          VdebugVec_size[2] = 2;
          for (i1 = 0; i1 < 2; i1++) {
            for (id2Bypass_size = 0; id2Bypass_size < k_ina219_; id2Bypass_size
                 ++) {
              b_VdebugVec_data[id2Bypass_size + VdebugVec_size[1] * i1] =
                VdebugVec_data[(((int)V_chr_RI + VdebugVec_size_idx_0_tmp *
                                 id2Bypass_size) + VdebugVec_size_idx_0_tmp *
                                k_ina219_ * i1) - 1];
            }
          }

          for (i1 = 0; i1 < 16; i1++) {
            g_prm[i1] = prm->brd.pac.VIpacId[((int)V_chr_RI + (i1 << 1)) - 1];
          }

          for (i1 = 0; i1 < N_bat_tmp_tmp_tmp; i1++) {
            c_VmKp184Test_data[i1] = VmKp184Test_data[((int)V_chr_RI + (int)
              Nina219_tmp_tmp * i1) - 1];
          }

          c_VdebugVec_data.data = &b_VdebugVec_data[0];
          c_VdebugVec_data.size = &VdebugVec_size[0];
          c_VdebugVec_data.allocatedSize = 64;
          c_VdebugVec_data.numDimensions = 3;
          c_VdebugVec_data.canFreeData = false;
          squeeze(&c_VdebugVec_data, r4);
          CalcK(Vdebug, (double *)r4->data, r4->size, g_prm, &a__2[0],
                c_VmKp184Test_data, N_bat_tmp_tmp_tmp, r2);
          IshuntTest2_data = r2->data;
          for (i1 = 0; i1 < 2; i1++) {
            for (id2Bypass_size = 0; id2Bypass_size < k_ina219_; id2Bypass_size
                 ++) {
              kEst_data[(((int)V_chr_RI + VdebugVec_size_idx_0_tmp *
                          id2Bypass_size) + VdebugVec_size_idx_0_tmp * k_ina219_
                         * i1) - 1] = IshuntTest2_data[id2Bypass_size + r2->
                size[0] * i1];
            }
          }
        } else {
          for (i1 = 0; i1 < N_bat_tmp_tmp_tmp; i1++) {
            VmKp184Test_data[((int)V_chr_RI + (int)Nina219_tmp_tmp * i1) - 1] =
              VbusTest_data[((int)V_chr_RI + (int)Nina219_tmp_tmp * i1) - 1];
          }
        }

        /*  Kalman */
        if (kalmanFlag) {
          for (c_k_bat = 0; c_k_bat < N_bat_tmp_tmp_tmp; c_k_bat++) {
            for (i1 = 0; i1 < 31; i1++) {
              fv[i1] = prm->klm.BatParamsCell[(int)V_chr_RI - 1]
                .BatStateOrg[c_k_bat + (i1 << 4)];
            }

            for (i1 = 0; i1 < 38; i1++) {
              h_prm[i1] = prm->klm.BatParamsCell[(int)V_chr_RI - 1]
                .BatParams[c_k_bat + (i1 << 4)];
            }

            CalcItByVout(h_prm, fv, Ta, VmKp184Test_data[((int)V_chr_RI + (int)
              Nina219_tmp_tmp * c_k_bat) - 1]);
            for (i1 = 0; i1 < 31; i1++) {
              prm->klm.BatParamsCell[(int)V_chr_RI - 1].BatState[c_k_bat + (i1 <<
                4)] = fv[i1];
            }
          }

          VmKp184Test_size[0] = 1;
          VmKp184Test_size[1] = N_bat_tmp_tmp_tmp;
          for (i1 = 0; i1 < N_bat_tmp_tmp_tmp; i1++) {
            c_VmKp184Test_data[i1] = VmKp184Test_data[((int)V_chr_RI + (int)
              Nina219_tmp_tmp * i1) - 1];
          }

          k_groups_tmp = c_tmp_data[k0];
          memcpy(&e_prm[0], &(*(float (*)[16])&prm->klm.BatParamsCell[(int)
                              VecIna219_data[k_groups_tmp] - 1].BatState[288])[0],
                 16U * sizeof(float));
          memcpy(&f_prm[0], &prm->klm.BatParamsCell[(int)
                 VecIna219_data[k_groups_tmp] - 1].BatState[0], 496U * sizeof
                 (float));
          InitKalman(e_prm, f_prm, prm->klm.BatParamsCell[(int)
                     VecIna219_data[k_groups_tmp] - 1].BatParams,
                     &prm->klm.BatParamsCell[(int)VecIna219_data[k_groups_tmp] -
                     1].BatState[16], Ta, mean(c_VmKp184Test_data,
                      VmKp184Test_size), &prm->klm.b_struct[(int)V_chr_RI - 1]);
        }
      }

      /*  for k_ina219 = VecIna219_k */
      /*  for k_groups = 1:length(uGroups) */
    }

    /* for k_tstState = 1:N_stTst */
    if (prm->seq.tst.v.isPrm) {
      if (prm->brd.Nina219 < 1.0) {
        loop_ub = 0;
      } else {
        loop_ub = (int)prm->brd.Nina219;
      }

      if (prm->brd.N_bat < 1) {
        id2Bypass_size = 0;
      } else {
        id2Bypass_size = prm->brd.N_bat;
      }

      for (i = 0; i < 2; i++) {
        for (i1 = 0; i1 < id2Bypass_size; i1++) {
          for (i2 = 0; i2 < loop_ub; i2++) {
            prm->brd.pac.Rval[(i2 + (i1 << 1)) + (i << 5)] = (float)kEst_data
              [(i2 + loop_ub * i1) + loop_ub * id2Bypass_size * i];
          }
        }
      }
    }

    i = (int)prm->brd.Nina219;
    for (k_ina219_ = 0; k_ina219_ < i; k_ina219_++) {
      switch (ProjectFlag) {
       case 2:
        /* esp32 */
        break;

       case 3:
        /* esp32 ser */
        /*  writePortToSpi4RowMask_ser(disConAll,Esp32_v1(k_ina219),false); */
        /* UNTITLED Summary of this function goes here */
        /*    Detailed explanation goes here */
        /*  if coder.target('MATLAB') */
        /*  read current */
        /*   chip1     |   chip2     |   chip3 */
        /* %comN2 Port4 chip1,comN1 Port11 chip1 */
        /* %comP1 Port18 chip1 ,comP2 Port18 chip3 */
        /*  */
        /*  for k_row = 1:N_row */
        /*      out123(k_row,:) = readSPI(SPI,PortSpiRow(k_row,1),N_chip); */
        /*      oldPortSpiRow(k_row,1:2:end) = PortSpiRow(k_row,1:2:end); */
        /*      oldPortSpiRow(k_row,2:2:end) = out123(k_row,2:2:end); */
        /*  end */
        /* mask current */
        /* write current mask */
        /*  for k_row = 1:N_row */
        /*      write(SPI,oldPortSpiRowMask(k_row,:)); */
        /*  end */
        pause(0.001);

        /* mask new */
        /* write mask new */
        /*  for k_row = 1:N_row */
        /*      write(SPI0,PortSpiRowMask(k_row,:)); */
        /*  end */
        /* write new */
        /*  for k_row = 1:N_row */
        /*      write(SPI0,PortSpiRow(k_row,:)); */
        /*  end */
        /*  Rmsg = SPI_writeread(SPI0, PortSpiRow); */
        /*  ReadPortSpiRow = RmsgData([[2:4] , 1] ,:); */
        /*  ReadPortSpiRow(:,1:2:end) = ReadPortSpiRow(:,1:2:end);% - 128; */
        /* mask current */
        /* %chip1 Port6 Alert1, chip1 port 7 Alert3, chip1 port 8 Alert8, chip1 port 9 Alert6 */
        /* %chip1 Port12 Alert7, chip1 Port13 Alert5, chip 1 port19 NC */
        /* %chip1 Port20-23 NC */
        /* chip1 Port28 Alert1, chip1 Port29 Alert2 */
        /*  if sum(sum(sum(abs(double(PortSpiRow)-double(ReadPortSpiRow)))))>0 */
        /*  end */
        break;
      }
    }

    if (prm->run.testVreset) {
      VbusTest_size = loop_ub_tmp;
      for (i = 0; i < loop_ub_tmp; i++) {
        c_VbusTest_data[i] = (VbusTest_data[i] > prm->bat.VresetMax);
      }

      d_VbusTest_data.data = &c_VbusTest_data[0];
      d_VbusTest_data.size = &VbusTest_size;
      d_VbusTest_data.allocatedSize = 512;
      d_VbusTest_data.numDimensions = 1;
      d_VbusTest_data.canFreeData = false;
      if (any(&d_VbusTest_data)) {
        outStruct->VresetFlag = false;
        if (prm->brd.Nina219 < 1.0) {
          loop_ub = 0;
        } else {
          loop_ub = (int)prm->brd.Nina219;
        }

        b_loop_ub = outStruct->VbusTest->size[1];
        for (i = 0; i < b_loop_ub; i++) {
          for (i1 = 0; i1 < loop_ub; i1++) {
            outStruct->VbusTest->data[i1 + outStruct->VbusTest->size[0] * i] =
              VbusTest_data[i1 + loop_ub * i];
          }
        }

        b_loop_ub = outStruct->VmKp184Test->size[1];
        for (i = 0; i < b_loop_ub; i++) {
          for (i1 = 0; i1 < loop_ub; i1++) {
            outStruct->VmKp184Test->data[i1 + outStruct->VmKp184Test->size[0] *
              i] = VmKp184Test_data[i1 + loop_ub * i];
          }
        }
      } else {
        VbusTest_size = loop_ub_tmp;
        for (i = 0; i < loop_ub_tmp; i++) {
          c_VbusTest_data[i] = (VbusTest_data[i] < prm->bat.VresetMax);
        }

        if (all(c_VbusTest_data, VbusTest_size)) {
          outStruct->VresetFlag = true;
          if (prm->brd.Nina219 < 1.0) {
            loop_ub = 0;
          } else {
            loop_ub = (int)prm->brd.Nina219;
          }

          b_loop_ub = outStruct->VbusTest->size[1];
          for (i = 0; i < b_loop_ub; i++) {
            for (i1 = 0; i1 < loop_ub; i1++) {
              outStruct->VbusTest->data[i1 + outStruct->VbusTest->size[0] * i] =
                VbusTest_data[i1 + loop_ub * i];
            }
          }

          b_loop_ub = outStruct->VmKp184Test->size[1];
          for (i = 0; i < b_loop_ub; i++) {
            for (i1 = 0; i1 < loop_ub; i1++) {
              outStruct->VmKp184Test->data[i1 + outStruct->VmKp184Test->size[0] *
                i] = VmKp184Test_data[i1 + loop_ub * i];
            }
          }
        } else {
          outStruct->VresetFlag = false;
          if (prm->brd.Nina219 < 1.0) {
            loop_ub = 0;
          } else {
            loop_ub = (int)prm->brd.Nina219;
          }

          b_loop_ub = outStruct->VbusTest->size[1];
          for (i = 0; i < b_loop_ub; i++) {
            for (i1 = 0; i1 < loop_ub; i1++) {
              outStruct->VbusTest->data[i1 + outStruct->VbusTest->size[0] * i] =
                VbusTest_data[i1 + loop_ub * i];
            }
          }

          b_loop_ub = outStruct->VmKp184Test->size[1];
          for (i = 0; i < b_loop_ub; i++) {
            for (i1 = 0; i1 < loop_ub; i1++) {
              outStruct->VmKp184Test->data[i1 + outStruct->VmKp184Test->size[0] *
                i] = VmKp184Test_data[i1 + loop_ub * i];
            }
          }
        }
      }
    } else {
      VbusTest_size = loop_ub_tmp;
      for (i = 0; i < loop_ub_tmp; i++) {
        c_VbusTest_data[i] = (VbusTest_data[i] > prm->bat.Vmax);
      }

      d_VbusTest_data.data = &c_VbusTest_data[0];
      d_VbusTest_data.size = &VbusTest_size;
      d_VbusTest_data.allocatedSize = 512;
      d_VbusTest_data.numDimensions = 1;
      d_VbusTest_data.canFreeData = false;
      if (any(&d_VbusTest_data)) {
        guard2 = true;
      } else {
        VbusTest_size = loop_ub_tmp;
        for (i = 0; i < loop_ub_tmp; i++) {
          c_VbusTest_data[i] = (VbusTest_data[i] < prm->bat.Vmin);
        }

        e_VbusTest_data.data = &c_VbusTest_data[0];
        e_VbusTest_data.size = &VbusTest_size;
        e_VbusTest_data.allocatedSize = 512;
        e_VbusTest_data.numDimensions = 1;
        e_VbusTest_data.canFreeData = false;
        if (any(&e_VbusTest_data)) {
          guard2 = true;
        } else {
          VbusTest_size = loop_ub_tmp;
          for (i = 0; i < loop_ub_tmp; i++) {
            c_VbusTest_data[i] = (VmKp184Test_data[i] > prm->bat.Vmax);
          }

          f_VbusTest_data.data = &c_VbusTest_data[0];
          f_VbusTest_data.size = &VbusTest_size;
          f_VbusTest_data.allocatedSize = 512;
          f_VbusTest_data.numDimensions = 1;
          f_VbusTest_data.canFreeData = false;
          if (any(&f_VbusTest_data)) {
            guard2 = true;
          } else {
            VbusTest_size = loop_ub_tmp;
            for (i = 0; i < loop_ub_tmp; i++) {
              c_VbusTest_data[i] = (VmKp184Test_data[i] < prm->bat.Vmin);
            }

            g_VbusTest_data.data = &c_VbusTest_data[0];
            g_VbusTest_data.size = &VbusTest_size;
            g_VbusTest_data.allocatedSize = 512;
            g_VbusTest_data.numDimensions = 1;
            g_VbusTest_data.canFreeData = false;
            if (any(&g_VbusTest_data)) {
              guard2 = true;
            } else {
              guard4 = true;
            }
          }
        }
      }
    }
  } else {
    /*  Kalman NA */
    if (prm->klm.kalmanFlag) {
      if ((int)prm->brd.Nina219 - 1 >= 0) {
        b_loop_ub = loop_ub_tmp;
        k_t0 = prm->brd.N_bat;
      }

      if (VbusTest_size_idx_0 - 1 >= 0) {
        VmKp184Test_size_idx_0 = (int)Nina219_tmp_tmp;
        VmKp184Test_size_idx_1 = N_bat_tmp_tmp_tmp;
        VmKp184Test_size[0] = 1;
        VmKp184Test_size[1] = N_bat_tmp_tmp_tmp;
        for (i = 0; i < b_loop_ub; i++) {
          VmKp184Test_data[i] = 4.0;
        }

        for (i = 0; i < k_t0; i++) {
          c_VmKp184Test_data[i] = 4.0;
        }
      }

      for (k_ina219_ = 0; k_ina219_ < VbusTest_size_idx_0; k_ina219_++) {
        /*  for k_bat = 1:N_bat */
        /*      BatParamsCell{k_ina219}.BatState(k_bat,:) = 0;%CalcItByVout(BatParamsCell{k_ina219}.BatParams(k_bat,:),BatParamsCell{k_ina219}.BatStateOrg(k_bat,:),Ta,1/3600,0,0,0,mean(squeeze(VmKp184Test(k_ina219,k_bat)))); */
        /*  end */
        memcpy(&e_prm[0], &(*(float (*)[16])&prm->klm.BatParamsCell[k_ina219_].
                            BatState[288])[0], 16U * sizeof(float));
        memcpy(&f_prm[0], &prm->klm.BatParamsCell[k_ina219_].BatState[0], 496U *
               sizeof(float));
        InitKalman(e_prm, f_prm, prm->klm.BatParamsCell[k_ina219_].BatParams,
                   &prm->klm.BatParamsCell[k_ina219_].BatState[16], Ta, mean
                   (c_VmKp184Test_data, VmKp184Test_size), &prm->
                   klm.b_struct[k_ina219_]);

        /* KalmanStruct(k_ina219_,1) = InitKalman(BatParamsCellKalmanStruct.BatState(:,19),BatParamsCell(k_ina219_,1).BatState,BatParamsCell(k_ina219_,1).BatParams,BatParamsCell(k_ina219_,1).BatState(:,2),Ta,mean(squeeze(VmKp184Test(k_ina219_,:)),2)); */
      }
    }

    guard4 = true;
  }

  if (guard4) {
    /* if vTestFlag */
    NstTst_tmp_tmp = prm->seq.tst.i.Nst;

    /* length(seqTstI.grp); */
    if (prm->seq.tst.i.isTest) {
      /*  ItestSwitch = [1:N_bat]'; */
      /*  [SpiPortRow0,ina219State,Pac2Vid0]  = CalcPortSpiPrm(prm.brd.spi,ItestSwitch); */
      /*  I2switchId = ina219State; */
      tic();

      /* 10; */
      /* linspace(0,4.9,NTtest); */
      i = VbusTest2->size[0] * VbusTest2->size[1] * VbusTest2->size[2] *
        VbusTest2->size[3];
      VbusTest2->size[0] = (int)prm->brd.Nina219;
      VbusTest2->size[1] = prm->brd.N_bat;
      VbusTest2->size[2] = prm->seq.tst.i.NTtest;
      k_t0 = prm->seq.tst.i.Nst;
      VbusTest2->size[3] = prm->seq.tst.i.Nst;
      emxEnsureCapacity_real_T(VbusTest2, i);
      VbusTest2_data = VbusTest2->data;
      id2Bypass_size = (int)prm->brd.Nina219 * prm->brd.N_bat;
      b_loop_ub_tmp = id2Bypass_size * prm->seq.tst.i.NTtest *
        prm->seq.tst.i.Nst;
      for (i = 0; i < b_loop_ub_tmp; i++) {
        VbusTest2_data[i] = 0.0;
      }

      i = IshuntTest2->size[0] * IshuntTest2->size[1] * IshuntTest2->size[2] *
        IshuntTest2->size[3];
      IshuntTest2->size[0] = (int)prm->brd.Nina219;
      IshuntTest2->size[1] = prm->brd.N_bat;
      IshuntTest2->size[2] = prm->seq.tst.i.NTtest;
      IshuntTest2->size[3] = prm->seq.tst.i.Nst;
      emxEnsureCapacity_real_T(IshuntTest2, i);
      IshuntTest2_data = IshuntTest2->data;
      for (i = 0; i < b_loop_ub_tmp; i++) {
        IshuntTest2_data[i] = 0.0;
      }

      i = ImKp184Test2->size[0] * ImKp184Test2->size[1] * ImKp184Test2->size[2] *
        ImKp184Test2->size[3];
      ImKp184Test2->size[0] = (int)prm->brd.Nina219;
      ImKp184Test2->size[1] = prm->brd.N_bat;
      ImKp184Test2->size[2] = prm->seq.tst.i.NTtest;
      ImKp184Test2->size[3] = prm->seq.tst.i.Nst;
      emxEnsureCapacity_real_T(ImKp184Test2, i);
      ImKp184Test2_data = ImKp184Test2->data;
      for (i = 0; i < b_loop_ub_tmp; i++) {
        ImKp184Test2_data[i] = 0.0;
      }

      i = Iacs758_cal->size[0] * Iacs758_cal->size[1] * Iacs758_cal->size[2] *
        Iacs758_cal->size[3];
      Iacs758_cal->size[0] = (int)prm->brd.Nina219;
      Iacs758_cal->size[1] = 1;
      Iacs758_cal->size[2] = prm->seq.tst.i.NTtest;
      Iacs758_cal->size[3] = prm->seq.tst.i.Nst;
      emxEnsureCapacity_real_T(Iacs758_cal, i);
      Iacs758_cal_data = Iacs758_cal->data;
      loop_ub = (int)prm->brd.Nina219 * prm->seq.tst.i.NTtest *
        prm->seq.tst.i.Nst;
      for (i = 0; i < loop_ub; i++) {
        Iacs758_cal_data[i] = 0.0;
      }

      i = Rtot->size[0] * Rtot->size[1];
      Rtot->size[0] = (int)prm->brd.Nina219;
      Rtot->size[1] = prm->brd.N_bat;
      emxEnsureCapacity_real_T(Rtot, i);
      Rtot_data = Rtot->data;
      for (i = 0; i < id2Bypass_size; i++) {
        Rtot_data[i] = 0.0;
      }

      i = Rwire->size[0] * Rwire->size[1];
      Rwire->size[0] = (int)prm->brd.Nina219;
      Rwire->size[1] = prm->brd.N_bat;
      emxEnsureCapacity_real32_T(Rwire, i);
      Rwire_data = Rwire->data;
      for (i = 0; i < id2Bypass_size; i++) {
        Rwire_data[i] = 0.0F;
      }

      if (prm->brd.Nina219 < 1.0) {
        VecIna219->size[0] = 1;
        VecIna219->size[1] = 0;
      } else {
        i = VecIna219->size[0] * VecIna219->size[1];
        VecIna219->size[0] = 1;
        VecIna219->size[1] = (int)(prm->brd.Nina219 - 1.0) + 1;
        emxEnsureCapacity_real_T(VecIna219, i);
        VecIna219_data = VecIna219->data;
        loop_ub = (int)(prm->brd.Nina219 - 1.0);
        for (i = 0; i <= loop_ub; i++) {
          VecIna219_data[i] = (double)i + 1.0;
        }
      }

      for (k_tstState = 0; k_tstState < k_t0; k_tstState++) {
        pause(0.1);
        if (Nina219_tmp_tmp < 1.0) {
          i = 0;
        } else {
          i = (int)Nina219_tmp_tmp;
        }

        if (i - 1 >= 0) {
          b_k_tstState = k_tstState + 1;
          b_end = ipos_size - 1;
        }

        for (k0 = 0; k0 < i; k0++) {
          VdebugVec_size_idx_0_tmp = k_tstState << 3;
          k_groups_tmp = prm->seq.tst.i.grp[k0 + VdebugVec_size_idx_0_tmp];

          /* 1:length(uGroups) %seqTstI.grp(k_tstState)%1:length(uGroups) */
          /* seqTstI.ItestSwitch{k_groups,k_tstState};%[1:N_bat]'; */
          i1 = (k_groups_tmp - 1) << 8;
          b_CalcPortSpiPrm(prm->brd.spi.disconnect, prm->brd.spi.bypass,
                           prm->brd.spi.PortSpiRow_esp,
                           prm->brd.spi.SwitchMat_esp,
                           prm->brd.spi.PortSpiRow_esp2,
                           prm->brd.spi.SwitchMat_esp2, prm->brd.spi.Pac2Vid,
                           prm->brd.spi.Pac2Vid2, &prm->seq.tst.i.ItestSwitch[i1
                           + ((b_k_tstState - 1) << 11)], disConAll, a__1_data,
                           id2Bypass_data, a__2);

          /*  I2switchId = ina219State; */
          /*  calc V */
          /* squeeze(Pac2Vid0All(k_ina219,:,:));%Pac2Vid3(:,1); */
          trueCount = 0;
          for (b_i = 0; b_i < 256; b_i++) {
            if (a__2[b_i] > 0) {
              trueCount++;
            }
          }

          VdebugVec_size_idx_0_tmp = (k_groups_tmp + VdebugVec_size_idx_0_tmp) -
            1;
          c_vSumMax = prm->bat.Vd + ((prm->bat.CutOffChrV[0] + 0.1F) + maximum
            (prm->seq.tst.i.i_in_test) * prm->
            seq.tst.i.Rin[VdebugVec_size_idx_0_tmp]) * (float)trueCount;
          state_k = 0;
          loop_ub = 0;

          /* VecIna219_k = find(k_groups == uGroupId); */
          trueCount = 0;
          for (b_i = 0; b_i <= b_end; b_i++) {
            i2 = ipos_data[b_i];
            if (k_groups_tmp == i2) {
              state_k++;
              f_tmp_data[loop_ub] = (signed char)b_i;
              loop_ub++;
            }

            if (k_groups_tmp != i2) {
              trueCount++;
            }
          }

          for (b_i = 0; b_i < trueCount; b_i++) {
            /* diconnect group that not tested */
            switch (ProjectFlag) {
             case 2:
              /* ESP32 */
              break;

             case 3:
              /* ESP32 ser */
              /* UNTITLED Summary of this function goes here */
              /*    Detailed explanation goes here */
              /*  if coder.target('MATLAB') */
              /*  read current */
              /*   chip1     |   chip2     |   chip3 */
              /* %comN2 Port4 chip1,comN1 Port11 chip1 */
              /* %comP1 Port18 chip1 ,comP2 Port18 chip3 */
              /*  */
              /*  for k_row = 1:N_row */
              /*      out123(k_row,:) = readSPI(SPI,PortSpiRow(k_row,1),N_chip); */
              /*      oldPortSpiRow(k_row,1:2:end) = PortSpiRow(k_row,1:2:end); */
              /*      oldPortSpiRow(k_row,2:2:end) = out123(k_row,2:2:end); */
              /*  end */
              /* mask current */
              /* write current mask */
              /*  for k_row = 1:N_row */
              /*      write(SPI,oldPortSpiRowMask(k_row,:)); */
              /*  end */
              pause(0.001);

              /* mask new */
              /* write mask new */
              /*  for k_row = 1:N_row */
              /*      write(SPI0,PortSpiRowMask(k_row,:)); */
              /*  end */
              /* write new */
              /*  for k_row = 1:N_row */
              /*      write(SPI0,PortSpiRow(k_row,:)); */
              /*  end */
              /*  Rmsg = SPI_writeread(SPI0, PortSpiRow); */
              /*  ReadPortSpiRow = RmsgData([[2:4] , 1] ,:); */
              /*  ReadPortSpiRow(:,1:2:end) = ReadPortSpiRow(:,1:2:end);% - 128; */
              /* mask current */
              /* %chip1 Port6 Alert1, chip1 port 7 Alert3, chip1 port 8 Alert8, chip1 port 9 Alert6 */
              /* %chip1 Port12 Alert7, chip1 Port13 Alert5, chip 1 port19 NC */
              /* %chip1 Port20-23 NC */
              /* chip1 Port28 Alert1, chip1 Port29 Alert2 */
              /*  if sum(sum(sum(abs(double(PortSpiRow)-double(ReadPortSpiRow)))))>0 */
              /*  end */
              break;
            }

            pause(0.1);
          }

          for (b_i = 0; b_i < state_k; b_i++) {
            V_chr_RI = VecIna219_data[f_tmp_data[b_i]];
            if (prm->seq.tst.i.BrdBeforePSflag[VdebugVec_size_idx_0_tmp] != 0) {
              for (i2 = 0; i2 < state_k; i2++) {
                b_VecIna219_data[i2] = (VecIna219_data[f_tmp_data[i2]] !=
                  V_chr_RI);
              }

              id2Bypass_size = c_eml_find(b_VecIna219_data, state_k,
                id2Bypass_data);
              for (k_ina219_ = 0; k_ina219_ < id2Bypass_size; k_ina219_++) {
                /* k_bypass = VecIna219_k(id2Bypass) */
                /*  ina219StateAll(:,k_bypass)   = ina219StateBypass; */
                /*  BattConfig{k_bypass}       = prm.brd.spi.bypass;%-2; */
                /*  BattConfigPerIna{k_bypass} = prm.brd.spi.bypass;%-2; */
                d = VecIna219_data[f_tmp_data[id2Bypass_data[k_ina219_] - 1]];
                if (d < 2.147483648E+9) {
                  i2 = (int)d;
                } else {
                  i2 = MAX_int32_T;
                }

                b_sprintf(i2, r8);
                switch (ProjectFlag) {
                 case 2:
                  /* ESP32 */
                  /* TODO bypass */
                  break;

                 case 3:
                  /* ESP32 ser */
                  /* UNTITLED Summary of this function goes here */
                  /*    Detailed explanation goes here */
                  /*  if coder.target('MATLAB') */
                  /*  read current */
                  /*   chip1     |   chip2     |   chip3 */
                  /* %comN2 Port4 chip1,comN1 Port11 chip1 */
                  /* %comP1 Port18 chip1 ,comP2 Port18 chip3 */
                  /*  */
                  /*  for k_row = 1:N_row */
                  /*      out123(k_row,:) = readSPI(SPI,PortSpiRow(k_row,1),N_chip); */
                  /*      oldPortSpiRow(k_row,1:2:end) = PortSpiRow(k_row,1:2:end); */
                  /*      oldPortSpiRow(k_row,2:2:end) = out123(k_row,2:2:end); */
                  /*  end */
                  /* mask current */
                  /* write current mask */
                  /*  for k_row = 1:N_row */
                  /*      write(SPI,oldPortSpiRowMask(k_row,:)); */
                  /*  end */
                  pause(0.001);

                  /* mask new */
                  /* write mask new */
                  /*  for k_row = 1:N_row */
                  /*      write(SPI0,PortSpiRowMask(k_row,:)); */
                  /*  end */
                  /* write new */
                  /*  for k_row = 1:N_row */
                  /*      write(SPI0,PortSpiRow(k_row,:)); */
                  /*  end */
                  /*  Rmsg = SPI_writeread(SPI0, PortSpiRow); */
                  /*  ReadPortSpiRow = RmsgData([[2:4] , 1] ,:); */
                  /*  ReadPortSpiRow(:,1:2:end) = ReadPortSpiRow(:,1:2:end);% - 128; */
                  /* mask current */
                  /* %chip1 Port6 Alert1, chip1 port 7 Alert3, chip1 port 8 Alert8, chip1 port 9 Alert6 */
                  /* %chip1 Port12 Alert7, chip1 Port13 Alert5, chip 1 port19 NC */
                  /* %chip1 Port20-23 NC */
                  /* chip1 Port28 Alert1, chip1 Port29 Alert2 */
                  /*  if sum(sum(sum(abs(double(PortSpiRow)-double(ReadPortSpiRow)))))>0 */
                  /*  end */
                  /* TODO bypass */
                  break;
                }

                pause(0.1);
              }

              b_CalcPortSpiPrm(prm->brd.spi.disconnect, prm->brd.spi.bypass,
                               prm->brd.spi.PortSpiRow_esp,
                               prm->brd.spi.SwitchMat_esp,
                               prm->brd.spi.PortSpiRow_esp2,
                               prm->brd.spi.SwitchMat_esp2, prm->brd.spi.Pac2Vid,
                               prm->brd.spi.Pac2Vid2,
                               &prm->seq.tst.i.ItestSwitch[i1 + (k_tstState <<
                11)], disConAll, a__1_data, id2Bypass_data, a__2);

              /*  ina219StateAll(:,k_ina219)   = ina219State; */
              /*  BattConfig{k_ina219}       = iTestSwitchCell_1+(k_ina219-1)*N_bat; */
              /*  BattConfigPerIna{k_ina219} = iTestSwitchCell_1; */
            }

            if (V_chr_RI < 2.147483648E+9) {
              i2 = (int)V_chr_RI;
            } else {
              i2 = MAX_int32_T;
            }

            b_sprintf(i2, r7);
            i2 = id2Bypass_data[0];
            for (b_loop_ub_tmp = 0; b_loop_ub_tmp < i2; b_loop_ub_tmp++) {
              d = rt_roundd_snf(a__1_data[b_loop_ub_tmp]);
              if (d < 2.147483648E+9) {
                if (d >= -2.147483648E+9) {
                  i3 = (int)d;
                } else {
                  i3 = MIN_int32_T;
                }
              } else if (d >= 2.147483648E+9) {
                i3 = MAX_int32_T;
              } else {
                i3 = 0;
              }

              printf("%d \n", i3);
              fflush(stdout);
            }

            switch (ProjectFlag) {
             case 2:
              /* esp32 */
              break;

             case 3:
              /* esp32 ser */
              /* UNTITLED Summary of this function goes here */
              /*    Detailed explanation goes here */
              /*  if coder.target('MATLAB') */
              /*  read current */
              /*   chip1     |   chip2     |   chip3 */
              /* %comN2 Port4 chip1,comN1 Port11 chip1 */
              /* %comP1 Port18 chip1 ,comP2 Port18 chip3 */
              /*  */
              /*  for k_row = 1:N_row */
              /*      out123(k_row,:) = readSPI(SPI,PortSpiRow(k_row,1),N_chip); */
              /*      oldPortSpiRow(k_row,1:2:end) = PortSpiRow(k_row,1:2:end); */
              /*      oldPortSpiRow(k_row,2:2:end) = out123(k_row,2:2:end); */
              /*  end */
              /* mask current */
              /* write current mask */
              /*  for k_row = 1:N_row */
              /*      write(SPI,oldPortSpiRowMask(k_row,:)); */
              /*  end */
              pause(0.001);

              /* mask new */
              /* write mask new */
              /*  for k_row = 1:N_row */
              /*      write(SPI0,PortSpiRowMask(k_row,:)); */
              /*  end */
              /* write new */
              /*  for k_row = 1:N_row */
              /*      write(SPI0,PortSpiRow(k_row,:)); */
              /*  end */
              /*  Rmsg = SPI_writeread(SPI0, PortSpiRow); */
              /*  ReadPortSpiRow = RmsgData([[2:4] , 1] ,:); */
              /*  ReadPortSpiRow(:,1:2:end) = ReadPortSpiRow(:,1:2:end);% - 128; */
              /* mask current */
              /* %chip1 Port6 Alert1, chip1 port 7 Alert3, chip1 port 8 Alert8, chip1 port 9 Alert6 */
              /* %chip1 Port12 Alert7, chip1 Port13 Alert5, chip 1 port19 NC */
              /* %chip1 Port20-23 NC */
              /* chip1 Port28 Alert1, chip1 Port29 Alert2 */
              /*  if sum(sum(sum(abs(double(PortSpiRow)-double(ReadPortSpiRow)))))>0 */
              /*  end */
              break;
            }

            /*  end */
            i2 = prm->seq.tst.i.NTtest;
            if (i2 - 1 >= 0) {
              if (V_chr_RI < 2.147483648E+9) {
                k_groups = (int)V_chr_RI;
                i4 = (int)V_chr_RI;
              } else {
                k_groups = MAX_int32_T;
                i4 = MAX_int32_T;
              }
            }

            for (c_k_Ttest = 0; c_k_Ttest < i2; c_k_Ttest++) {
              /*  for k_ina219 = 1:Nina219 */
              switch (prm->seq.tst.i.ins[(k_groups_tmp + (k_tstState << 3)) - 1])
              {
               case 1:
                /* kp184 */
                /* CC */
                pause(0.05);
                pause(0.05);
                pause(1.0);
                break;

               case 2:
                /* ka6005p */
                e_sprintf(c_vSumMax, vSumMax);

                /* ['VSET',num2str(ch),':',num2str(Val)];%[V] */
                pause(0.05);
                f_sprintf(rt_roundf_snf(prm->seq.tst.i.i_in_test[c_k_Ttest]),
                          b_prm);

                /* ['ISET',num2str(ch),':',num2str(Val)];%[A] */
                pause(0.05);
                pause(1.0);
                break;

               case 3:
                /* juntek+ACDC */
                /* Imax); */
                i_prm[0] = prm->seq.tst.i.i_in_test[c_k_Ttest];
                i_prm[1] = ImaxAcDC;
                f = rt_roundf_snf(rt_roundf_snf(1000.0F * minimum(i_prm)) /
                                  1000.0F * 1000.0F);
                if (f < 2.14748365E+9F) {
                  if (f >= -2.14748365E+9F) {
                    i3 = (int)f;
                  } else {
                    i3 = MIN_int32_T;
                  }
                } else if (f >= 2.14748365E+9F) {
                  i3 = MAX_int32_T;
                } else {
                  i3 = 0;
                }

                g_sprintf(i3, r9);

                /* [':',juntek_address,'w11=',num2str(uint32(Val*1000),'%04d'),',',newline];%['VSET',num2str(ch),':',num2str(Val)];%[V] */
                /*  disp(['Juntek: I=',num2str(IchargeBit)]); */
                pause(0.1);
                f = rt_roundf_snf(c_vSumMax * 100.0F);
                if (f < 2.14748365E+9F) {
                  if (f >= -2.14748365E+9F) {
                    i3 = (int)f;
                  } else {
                    i3 = MIN_int32_T;
                  }
                } else if (f >= 2.14748365E+9F) {
                  i3 = MAX_int32_T;
                } else {
                  i3 = 0;
                }

                g_sprintf(i3, r11);

                /* [':',juntek_address,'w10=',num2str(uint32(Val*100),'%04d'),',',newline];%['VSET',num2str(ch),':',num2str(Val)];%[V] */
                /*  disp(['Juntek VSET=',num2str(vSumMax)]); */
                pause(0.1);
                pause(1.0);

                /*  disp('juntek ON'); */
                break;

               case 5:
                /* switch out to load */
                pause(0.05);
                break;
              }

              /*              [~,~,~,~]  = ControlKp184(s_kp184,'Read',[]); */
              pause(0.5);
              if (prm->seq.tst.i.BrdBeforePSflag[VdebugVec_size_idx_0_tmp] == 0)
              {
                for (i3 = 0; i3 < state_k; i3++) {
                  b_VecIna219_data[i3] = (VecIna219_data[f_tmp_data[i3]] !=
                    V_chr_RI);
                }

                id2Bypass_size = c_eml_find(b_VecIna219_data, state_k,
                  id2Bypass_data);
                for (k_ina219_ = 0; k_ina219_ < id2Bypass_size; k_ina219_++) {
                  /*  ina219StateAll(:,k_bypass)   = ina219StateBypass; */
                  /*  BattConfig{k_bypass}       = prm.brd.spi.bypass;%-2; */
                  /*  BattConfigPerIna{k_bypass} = prm.brd.spi.bypass;%-2; */
                  d = VecIna219_data[f_tmp_data[id2Bypass_data[k_ina219_] - 1]];
                  if (d < 2.147483648E+9) {
                    i3 = (int)d;
                  } else {
                    i3 = MAX_int32_T;
                  }

                  b_sprintf(i3, r10);
                  switch (ProjectFlag) {
                   case 2:
                    /* ESP32 */
                    /* TODO bypass */
                    break;

                   case 3:
                    /* ESP32 ser */
                    /* UNTITLED Summary of this function goes here */
                    /*    Detailed explanation goes here */
                    /*  if coder.target('MATLAB') */
                    /*  read current */
                    /*   chip1     |   chip2     |   chip3 */
                    /* %comN2 Port4 chip1,comN1 Port11 chip1 */
                    /* %comP1 Port18 chip1 ,comP2 Port18 chip3 */
                    /*  */
                    /*  for k_row = 1:N_row */
                    /*      out123(k_row,:) = readSPI(SPI,PortSpiRow(k_row,1),N_chip); */
                    /*      oldPortSpiRow(k_row,1:2:end) = PortSpiRow(k_row,1:2:end); */
                    /*      oldPortSpiRow(k_row,2:2:end) = out123(k_row,2:2:end); */
                    /*  end */
                    /* mask current */
                    /* write current mask */
                    /*  for k_row = 1:N_row */
                    /*      write(SPI,oldPortSpiRowMask(k_row,:)); */
                    /*  end */
                    pause(0.001);

                    /* mask new */
                    /* write mask new */
                    /*  for k_row = 1:N_row */
                    /*      write(SPI0,PortSpiRowMask(k_row,:)); */
                    /*  end */
                    /* write new */
                    /*  for k_row = 1:N_row */
                    /*      write(SPI0,PortSpiRow(k_row,:)); */
                    /*  end */
                    /*  Rmsg = SPI_writeread(SPI0, PortSpiRow); */
                    /*  ReadPortSpiRow = RmsgData([[2:4] , 1] ,:); */
                    /*  ReadPortSpiRow(:,1:2:end) = ReadPortSpiRow(:,1:2:end);% - 128; */
                    /* mask current */
                    /* %chip1 Port6 Alert1, chip1 port 7 Alert3, chip1 port 8 Alert8, chip1 port 9 Alert6 */
                    /* %chip1 Port12 Alert7, chip1 Port13 Alert5, chip 1 port19 NC */
                    /* %chip1 Port20-23 NC */
                    /* chip1 Port28 Alert1, chip1 Port29 Alert2 */
                    /*  if sum(sum(sum(abs(double(PortSpiRow)-double(ReadPortSpiRow)))))>0 */
                    /*  end */
                    /* TODO bypass */
                    break;
                  }

                  pause(0.1);
                }

                b_CalcPortSpiPrm(prm->brd.spi.disconnect, prm->brd.spi.bypass,
                                 prm->brd.spi.PortSpiRow_esp,
                                 prm->brd.spi.SwitchMat_esp,
                                 prm->brd.spi.PortSpiRow_esp2,
                                 prm->brd.spi.SwitchMat_esp2,
                                 prm->brd.spi.Pac2Vid, prm->brd.spi.Pac2Vid2,
                                 &prm->seq.tst.i.ItestSwitch[i1 + (k_tstState <<
                  11)], disConAll, a__1_data, id2Bypass_data, a__2);

                /*  ina219StateAll(:,k_ina219)   = ina219State; */
                /*  BattConfig{k_ina219}       = iTestSwitchCell_1+(k_ina219-1)*N_bat; */
                /*  BattConfigPerIna{k_ina219} = iTestSwitchCell_1; */
              }

              pause(0.5);
              switch (ProjectFlag) {
               case 2:
                /* esp32 */
                i3 = VmV->size[0];
                VmV->size[0] = 16;
                emxEnsureCapacity_real_T(VmV, i3);
                VmV_data = VmV->data;
                i3 = b_I->size[0];
                b_I->size[0] = 16;
                emxEnsureCapacity_real_T(b_I, i3);
                IbatMat_data = b_I->data;
                i3 = tvv->size[0];
                tvv->size[0] = 16;
                emxEnsureCapacity_real_T(tvv, i3);
                Vdebug_data = tvv->data;
                for (i3 = 0; i3 < 16; i3++) {
                  VmV_data[i3] = 0.0;
                  IbatMat_data[i3] = 0.0;
                  Vdebug_data[i3] = 0.0;
                }
                break;

               case 3:
                /* esp32 ser */
                for (i3 = 0; i3 < 16; i3++) {
                  g_prm[i3] = prm->brd.pac.VIpacId[((int)V_chr_RI + (i3 << 1)) -
                    1];
                }

                c_readI2cVIfastTicTocV5_Rratio_(g_prm, N_bat1, VmV, b_I, b_tvv,
                  tii, (double *)&errI2C_data, errI2C_size, Vdebug);
                IbatMat_data = b_I->data;
                VmV_data = VmV->data;
                i3 = tvv->size[0];
                tvv->size[0] = 16;
                emxEnsureCapacity_real_T(tvv, i3);
                Vdebug_data = tvv->data;
                for (i3 = 0; i3 < 16; i3++) {
                  Vdebug_data[i3] = b_tvv[i3];
                }
                break;

               default:
                i3 = VmV->size[0];
                VmV->size[0] = N_bat_tmp_tmp_tmp;
                emxEnsureCapacity_real_T(VmV, i3);
                VmV_data = VmV->data;
                i3 = b_I->size[0];
                b_I->size[0] = N_bat_tmp_tmp_tmp;
                emxEnsureCapacity_real_T(b_I, i3);
                IbatMat_data = b_I->data;
                i3 = tvv->size[0];
                tvv->size[0] = N_bat_tmp_tmp_tmp;
                emxEnsureCapacity_real_T(tvv, i3);
                Vdebug_data = tvv->data;
                for (i3 = 0; i3 < N_bat_tmp_tmp_tmp; i3++) {
                  VmV_data[i3] = 0.0;
                  IbatMat_data[i3] = 0.0;
                  Vdebug_data[i3] = 0.0;
                }
                break;
              }

              loop_ub = VbusTest2->size[1];
              for (i3 = 0; i3 < loop_ub; i3++) {
                VbusTest2_data[((((int)V_chr_RI + VbusTest2->size[0] * i3) +
                                 VbusTest2->size[0] * VbusTest2->size[1] *
                                 c_k_Ttest) + VbusTest2->size[0] *
                                VbusTest2->size[1] * VbusTest2->size[2] *
                                k_tstState) - 1] = VmV_data[i3];
              }

              loop_ub = IshuntTest2->size[1];
              for (i3 = 0; i3 < loop_ub; i3++) {
                IshuntTest2_data[((((int)V_chr_RI + IshuntTest2->size[0] * i3) +
                                   IshuntTest2->size[0] * IshuntTest2->size[1] *
                                   c_k_Ttest) + IshuntTest2->size[0] *
                                  IshuntTest2->size[1] * IshuntTest2->size[2] *
                                  k_tstState) - 1] = IbatMat_data[i3];
              }

              switch (prm->seq.tst.i.meas[(k_groups_tmp + (k_tstState << 3)) - 1])
              {
               case 1:
                /* kp184 */
                ControlKp184((double *)&a__3_data, a__3_size, (double *)
                             &b_meanIbrd0, a__4_size, (double *)&N0, tmp_size,
                             (double *)&a__16_data, a__16_size);
                ImKp184Test2_data[(((int)V_chr_RI + ImKp184Test2->size[0] *
                                    ImKp184Test2->size[1] * c_k_Ttest) +
                                   ImKp184Test2->size[0] * ImKp184Test2->size[1]
                                   * ImKp184Test2->size[2] * k_tstState) - 1] =
                  a__16_data;
                break;

               case 2:
                /* ka6005p */
                /* read I */
                d = d_rand();
                ImKp184Test2_data[(((int)V_chr_RI + ImKp184Test2->size[0] *
                                    ImKp184Test2->size[1] * c_k_Ttest) +
                                   ImKp184Test2->size[0] * ImKp184Test2->size[1]
                                   * ImKp184Test2->size[2] * k_tstState) - 1] =
                  d * 5.0;

                /* read I */
                d_rand();
                break;

               case 3:
                /* juntek+ACDC */
                controlJuntekDPH8920(outVI, c_tmp_size);
                ImKp184Test2_data[(((int)V_chr_RI + ImKp184Test2->size[0] *
                                    ImKp184Test2->size[1] * c_k_Ttest) +
                                   ImKp184Test2->size[0] * ImKp184Test2->size[1]
                                   * ImKp184Test2->size[2] * k_tstState) - 1] =
                  outVI[0];
                b_controlJuntekDPH8920(r31.data, r31.size);
                break;

               case 5:
                /*  switch out to load */
                /* []; */
                d = d_rand() * 2.5 + 2.0;
                b_meanIbrd0 = d_rand() * 5.0;
                if (prm->ins.prm.swOut.IfromVdivR_flag) {
                  if (prm->seq.tst.i.i_in_test[c_k_Ttest] == 0.0F) {
                    ImKp184Test2_data[(((int)V_chr_RI + ImKp184Test2->size[0] *
                                        ImKp184Test2->size[1] * c_k_Ttest) +
                                       ImKp184Test2->size[0] *
                                       ImKp184Test2->size[1] *
                                       ImKp184Test2->size[2] * k_tstState) - 1] =
                      0.0;
                  } else {
                    ImKp184Test2_data[(((int)V_chr_RI + ImKp184Test2->size[0] *
                                        ImKp184Test2->size[1] * c_k_Ttest) +
                                       ImKp184Test2->size[0] *
                                       ImKp184Test2->size[1] *
                                       ImKp184Test2->size[2] * k_tstState) - 1] =
                      (float)d / prm->seq.tst.i.Rload;

                    /* seqTstI.i_in_test(k_Ttest); */
                  }
                } else {
                  ImKp184Test2_data[(((int)V_chr_RI + ImKp184Test2->size[0] *
                                      ImKp184Test2->size[1] * c_k_Ttest) +
                                     ImKp184Test2->size[0] * ImKp184Test2->size
                                     [1] * ImKp184Test2->size[2] * k_tstState) -
                    1] = b_meanIbrd0;
                }
                break;
              }

              /*  formatSpec0 = '%.3g'; */
              /*  formatSpec1 = '%.5g'; */
              /*                      disp(['V(',pad(num2str([k_ina219,k_Ttest,k_tstState]),'left',padLeft),')=',pad(num2str(VbusTest2(k_ina219,:,k_Ttest,k_tstState)),padRight,'left',padLeft),'V ',newline ,... */
              /*                          'I(',pad(num2str([k_ina219,k_Ttest,k_tstState]),'left',padLeft),')=',pad(num2str(IshuntTest2(k_ina219,:,k_Ttest,k_tstState)),padRight,'left',padLeft),'A ']); */
              h_sprintf(k_groups, c_k_Ttest + 1, k_tstState + 1, k_Ttest);
              for (c_k_bat = 0; c_k_bat < N_bat_tmp_tmp_tmp; c_k_bat++) {
                i_sprintf(VbusTest2_data[((((int)V_chr_RI + VbusTest2->size[0] *
                  c_k_bat) + VbusTest2->size[0] * VbusTest2->size[1] * c_k_Ttest)
                           + VbusTest2->size[0] * VbusTest2->size[1] *
                           VbusTest2->size[2] * k_tstState) - 1], c_VbusTest2);
              }

              h_sprintf(i4, c_k_Ttest + 1, k_tstState + 1, b_k_Ttest);
              for (c_k_bat = 0; c_k_bat < N_bat_tmp_tmp_tmp; c_k_bat++) {
                i_sprintf(IshuntTest2_data[((((int)V_chr_RI + IshuntTest2->size
                  [0] * c_k_bat) + IshuntTest2->size[0] * IshuntTest2->size[1] *
                            c_k_Ttest) + IshuntTest2->size[0] *
                           IshuntTest2->size[1] * IshuntTest2->size[2] *
                           k_tstState) - 1], b_IshuntTest2);

                /* nadav Coder */
              }

              pause(0.1);
              if (Iacs758Flag == 1) {
                if (ProjectFlag == 2) {
                  /* esp32 */
                  Iacs758_cal_data[(((int)V_chr_RI + Iacs758_cal->size[0] *
                                     c_k_Ttest) + Iacs758_cal->size[0] *
                                    Iacs758_cal->size[2] * k_tstState) - 1] =
                    0.0;
                }

                b_meanIbrd0 = Iacs758_cal_data[(((int)V_chr_RI +
                  Iacs758_cal->size[0] * c_k_Ttest) + Iacs758_cal->size[0] *
                  Iacs758_cal->size[2] * k_tstState) - 1];
                trueCount = b_I->size[0];
                i3 = b_I->size[0];
                b_I->size[0] = trueCount;
                emxEnsureCapacity_real_T(b_I, i3);
                IbatMat_data = b_I->data;
                for (i3 = 0; i3 < trueCount; i3++) {
                  IbatMat_data[i3] = b_meanIbrd0;
                }
              } else if (Iacs758Flag == 2) {
                switch (ProjectFlag) {
                 case 2:
                  /* esp32 */
                  break;

                 case 3:
                  /* esp32 */
                  /* UNTITLED3 Summary of this function goes here */
                  /*    Detailed explanation goes here */
                  Iacs758_cal_data[(((int)V_chr_RI + Iacs758_cal->size[0] *
                                     c_k_Ttest) + Iacs758_cal->size[0] *
                                    Iacs758_cal->size[2] * k_tstState) - 1] =
                    randn() * 5.0;

                  /*  else */
                  /*      V = zeros(length(VIpacId),1); */
                  /*      I = zeros(length(VIpacId),1); */
                  /*      tI = zeros(length(VIpacId),1); */
                  /*      tV = zeros(length(VIpacId),1); */
                  /*      outV2 = zeros(length(VIpacId),1); */
                  /*      err = -1; */
                  /*  end */
                  /*  I = I(10); */
                  /*  tI = tI(10); */
                  b_meanIbrd0 = Iacs758_cal_data[(((int)V_chr_RI +
                    Iacs758_cal->size[0] * c_k_Ttest) + Iacs758_cal->size[0] *
                    Iacs758_cal->size[2] * k_tstState) - 1];
                  trueCount = b_I->size[0];
                  i3 = b_I->size[0];
                  b_I->size[0] = trueCount;
                  emxEnsureCapacity_real_T(b_I, i3);
                  IbatMat_data = b_I->data;
                  for (i3 = 0; i3 < trueCount; i3++) {
                    IbatMat_data[i3] = b_meanIbrd0;
                  }
                  break;
                }
              }

              switch (prm->seq.tst.i.ins[(k_groups_tmp + (k_tstState << 3)) - 1])
              {
               case 1:
                /* kunkin kp184 */
                break;

               case 2:
                /* ka6005p */
                break;

               case 3:
                /* juntek+ACDC */
                break;

               case 5:
                /* switch out to load */
                /* CC */
                pause(0.05);
                break;
              }

              b_pause(prm->seq.tst.i.pauseOff);

              /* pause(0.1); */
              /* kalman */
              if (kalmanFlag) {
                CalcStructKalman(&prm->klm.b_struct[(int)V_chr_RI - 1], Ta, tvv,
                                 b_I, VmV);
              }
            }

            /*  switch seqTstI.ins(k_tstState) */
            /*      case 1 %kunkin kp184 */
            /*          ControlKp184(prm.ser.s_kp184,'Off'); */
            /*      case 2 %ka6005p */
            /*          ControlKa6005P(prm.ser.s_ka6005p,'Off',[]); */
            /*      case 3 %juntek+ACDC */
            /*          controlJuntekDPH8920(prm.ser.s_juntek,'OFF',[]); */
            /*  end */
            /*  pause(seqTstI.pauseOff);%pause(0.1); */
            switch (ProjectFlag) {
             case 2:
              /* esp32 */
              break;

             case 3:
              /* esp32 ser */
              /* UNTITLED Summary of this function goes here */
              /*    Detailed explanation goes here */
              /*  if coder.target('MATLAB') */
              /*  read current */
              /*   chip1     |   chip2     |   chip3 */
              /* %comN2 Port4 chip1,comN1 Port11 chip1 */
              /* %comP1 Port18 chip1 ,comP2 Port18 chip3 */
              /*  */
              /*  for k_row = 1:N_row */
              /*      out123(k_row,:) = readSPI(SPI,PortSpiRow(k_row,1),N_chip); */
              /*      oldPortSpiRow(k_row,1:2:end) = PortSpiRow(k_row,1:2:end); */
              /*      oldPortSpiRow(k_row,2:2:end) = out123(k_row,2:2:end); */
              /*  end */
              /* mask current */
              /* write current mask */
              /*  for k_row = 1:N_row */
              /*      write(SPI,oldPortSpiRowMask(k_row,:)); */
              /*  end */
              pause(0.001);

              /* mask new */
              /* write mask new */
              /*  for k_row = 1:N_row */
              /*      write(SPI0,PortSpiRowMask(k_row,:)); */
              /*  end */
              /* write new */
              /*  for k_row = 1:N_row */
              /*      write(SPI0,PortSpiRow(k_row,:)); */
              /*  end */
              /*  Rmsg = SPI_writeread(SPI0, PortSpiRow); */
              /*  ReadPortSpiRow = RmsgData([[2:4] , 1] ,:); */
              /*  ReadPortSpiRow(:,1:2:end) = ReadPortSpiRow(:,1:2:end);% - 128; */
              /* mask current */
              /* %chip1 Port6 Alert1, chip1 port 7 Alert3, chip1 port 8 Alert8, chip1 port 9 Alert6 */
              /* %chip1 Port12 Alert7, chip1 Port13 Alert5, chip 1 port19 NC */
              /* %chip1 Port20-23 NC */
              /* chip1 Port28 Alert1, chip1 Port29 Alert2 */
              /*  if sum(sum(sum(abs(double(PortSpiRow)-double(ReadPortSpiRow)))))>0 */
              /*  end */
              break;
            }

            /*  if prm.ser.kp184_Flag */
            /*      ControlKp184(prm.ser.s_kp184,'Off',[]); */
            /*      pause(0.1); */
            /*  end */
          }
        }
      }

      /*  if prm.ser.kp184_Flag */
      /*      ControlKp184(prm.ser.s_kp184,'Off',[]); */
      /*  end */
      /* TODO save */
      i = (int)prm->brd.Nina219;
      i1 = pIshunt->size[0] * pIshunt->size[1] * pIshunt->size[2];
      pIshunt->size[0] = (int)prm->brd.Nina219;
      pIshunt->size[1] = prm->brd.N_bat;
      pIshunt->size[2] = 2;
      emxEnsureCapacity_real_T(pIshunt, i1);
      pIshunt_data = pIshunt->data;
      b_loop_ub_tmp = ((int)prm->brd.Nina219 * prm->brd.N_bat) << 1;
      for (i1 = 0; i1 < b_loop_ub_tmp; i1++) {
        pIshunt_data[i1] = 0.0;
      }

      i1 = RtotHelpPoly->size[0] * RtotHelpPoly->size[1] * RtotHelpPoly->size[2];
      RtotHelpPoly->size[0] = (int)prm->brd.Nina219;
      RtotHelpPoly->size[1] = prm->brd.N_bat;
      RtotHelpPoly->size[2] = 2;
      emxEnsureCapacity_real_T(RtotHelpPoly, i1);
      RtotHelpPoly_data = RtotHelpPoly->data;
      for (i1 = 0; i1 < b_loop_ub_tmp; i1++) {
        RtotHelpPoly_data[i1] = 0.0;
      }

      i1 = pIacs758->size[0] * pIacs758->size[1] * pIacs758->size[2];
      pIacs758->size[0] = (int)prm->brd.Nina219;
      pIacs758->size[1] = 1;
      pIacs758->size[2] = 2;
      emxEnsureCapacity_real32_T(pIacs758, i1);
      pIacs758_data = pIacs758->data;
      loop_ub = (int)prm->brd.Nina219 << 1;
      for (i1 = 0; i1 < loop_ub; i1++) {
        pIacs758_data[i1] = 0.0F;
      }

      if (!prm->seq.tst.i.measRintR) {
        i1 = pIshunt->size[0] * pIshunt->size[1] * pIshunt->size[2];
        pIshunt->size[0] = (int)prm->brd.Nina219;
        pIshunt->size[1] = prm->brd.N_bat;
        pIshunt->size[2] = 2;
        emxEnsureCapacity_real_T(pIshunt, i1);
        pIshunt_data = pIshunt->data;
        if ((i - 1 >= 0) && (N_bat_tmp_tmp_tmp - 1 >= 0)) {
          c_end = NstTst_tmp_tmp - 1;
          b_tmp_size[0] = 1;
        }

        for (k_ina219_ = 0; k_ina219_ < i; k_ina219_++) {
          i1 = b_VbusTest2->size[0] * b_VbusTest2->size[1] * b_VbusTest2->size[2]
            * b_VbusTest2->size[3];
          b_VbusTest2->size[0] = 1;
          b_VbusTest2->size[1] = 1;
          b_VbusTest2->size[2] = Iacs758_cal->size[2];
          loop_ub = Iacs758_cal->size[3];
          b_VbusTest2->size[3] = Iacs758_cal->size[3];
          emxEnsureCapacity_real_T(b_VbusTest2, i1);
          VmV_data = b_VbusTest2->data;
          for (i1 = 0; i1 < loop_ub; i1++) {
            b_loop_ub = Iacs758_cal->size[2];
            for (i2 = 0; i2 < b_loop_ub; i2++) {
              VmV_data[i2 + b_VbusTest2->size[2] * i1] = Iacs758_cal_data
                [(k_ina219_ + Iacs758_cal->size[0] * i2) + Iacs758_cal->size[0] *
                Iacs758_cal->size[2] * i1];
            }
          }

          b_squeeze(b_VbusTest2, Iacs758_cal0);
          i1 = b_VbusTest2->size[0] * b_VbusTest2->size[1] * b_VbusTest2->size[2]
            * b_VbusTest2->size[3];
          b_VbusTest2->size[0] = 1;
          b_VbusTest2->size[1] = 1;
          b_VbusTest2->size[2] = ImKp184Test2->size[2];
          loop_ub = ImKp184Test2->size[3];
          b_VbusTest2->size[3] = ImKp184Test2->size[3];
          emxEnsureCapacity_real_T(b_VbusTest2, i1);
          VmV_data = b_VbusTest2->data;
          for (i1 = 0; i1 < loop_ub; i1++) {
            b_loop_ub = ImKp184Test2->size[2];
            for (i2 = 0; i2 < b_loop_ub; i2++) {
              VmV_data[i2 + b_VbusTest2->size[2] * i1] = ImKp184Test2_data
                [(k_ina219_ + ImKp184Test2->size[0] * ImKp184Test2->size[1] * i2)
                + ImKp184Test2->size[0] * ImKp184Test2->size[1] *
                ImKp184Test2->size[2] * i1];
            }
          }

          b_squeeze(b_VbusTest2, ImKp184Test2_0);
          Vdebug_data = ImKp184Test2_0->data;
          if (ImKp184Test2_0->size[1] == NstTst_tmp_tmp) {
            i1 = ImKp184Test2_bat_0->size[0] * ImKp184Test2_bat_0->size[1];
            ImKp184Test2_bat_0->size[0] = ImKp184Test2_0->size[0];
            ImKp184Test2_bat_0->size[1] = ImKp184Test2_0->size[1];
            emxEnsureCapacity_real_T(ImKp184Test2_bat_0, i1);
            VmV_data = ImKp184Test2_bat_0->data;
            loop_ub = ImKp184Test2_0->size[1];
            for (i1 = 0; i1 < loop_ub; i1++) {
              b_loop_ub = ImKp184Test2_0->size[0];
              for (i2 = 0; i2 < b_loop_ub; i2++) {
                VmV_data[i2 + ImKp184Test2_bat_0->size[0] * i1] = Vdebug_data[i2
                  + ImKp184Test2_0->size[0] * i1] * (double)
                  prm->seq.tst.i.NegIflag[(ipos_data[k_ina219_] + (i1 << 3)) - 1];
              }
            }

            i1 = ImKp184Test2_0->size[0] * ImKp184Test2_0->size[1];
            ImKp184Test2_0->size[0] = ImKp184Test2_bat_0->size[0];
            ImKp184Test2_0->size[1] = ImKp184Test2_bat_0->size[1];
            emxEnsureCapacity_real_T(ImKp184Test2_0, i1);
            Vdebug_data = ImKp184Test2_0->data;
            loop_ub = ImKp184Test2_bat_0->size[0] * ImKp184Test2_bat_0->size[1];
            for (i1 = 0; i1 < loop_ub; i1++) {
              Vdebug_data[i1] = VmV_data[i1];
            }
          } else {
            binary_expand_op_2(ImKp184Test2_0, prm, ipos_data, k_ina219_,
                               NstTst_tmp_tmp - 1);
          }

          trueCount = Iacs758_cal0->size[0] * Iacs758_cal0->size[1];
          id2Bypass_size = ImKp184Test2_0->size[0] * ImKp184Test2_0->size[1];
          c_VdebugVec_data = *Iacs758_cal0;
          b_trueCount = trueCount;
          c_VdebugVec_data.size = &b_trueCount;
          c_VdebugVec_data.numDimensions = 1;
          b_ImKp184Test2_0 = *ImKp184Test2_0;
          c_ImKp184Test2_0 = id2Bypass_size;
          b_ImKp184Test2_0.size = &c_ImKp184Test2_0;
          b_ImKp184Test2_0.numDimensions = 1;
          polyfit(&c_VdebugVec_data, &b_ImKp184Test2_0, outVI);
          pIacs758_data[k_ina219_] = (float)outVI[0];
          pIacs758_data[k_ina219_ + pIacs758->size[0]] = (float)outVI[1];

          /* polyfit(squeeze(IshuntTest2(k_ina219,k_bat,:,k_iTest)),1000*squeeze(ImKp184Test2(k_ina219,1,:,k_iTest)),1); */
          for (c_k_bat = 0; c_k_bat < N_bat_tmp_tmp_tmp; c_k_bat++) {
            for (k_tstState = 0; k_tstState < k_t0; k_tstState++) {
              ismember0_data[k_tstState] = isMember((double)c_k_bat + 1.0,
                &prm->seq.tst.i.ItestSwitch[((ipos_data[k_ina219_] - 1) << 8) +
                (k_tstState << 11)]);
            }

            trueCount = 0;
            loop_ub = 0;
            for (b_i = 0; b_i <= c_end; b_i++) {
              if (ismember0_data[b_i]) {
                trueCount++;
                h_tmp_data[loop_ub] = (signed char)b_i;
                loop_ub++;
              }
            }

            i1 = b_VbusTest2->size[0] * b_VbusTest2->size[1] * b_VbusTest2->
              size[2] * b_VbusTest2->size[3];
            b_VbusTest2->size[0] = 1;
            b_VbusTest2->size[1] = 1;
            b_VbusTest2->size[2] = IshuntTest2->size[2];
            b_VbusTest2->size[3] = trueCount;
            emxEnsureCapacity_real_T(b_VbusTest2, i1);
            VmV_data = b_VbusTest2->data;
            for (i1 = 0; i1 < trueCount; i1++) {
              loop_ub = IshuntTest2->size[2];
              for (i2 = 0; i2 < loop_ub; i2++) {
                VmV_data[i2 + b_VbusTest2->size[2] * i1] = IshuntTest2_data
                  [((k_ina219_ + IshuntTest2->size[0] * c_k_bat) +
                    IshuntTest2->size[0] * IshuntTest2->size[1] * i2) +
                  IshuntTest2->size[0] * IshuntTest2->size[1] *
                  IshuntTest2->size[2] * h_tmp_data[i1]];
              }
            }

            b_squeeze(b_VbusTest2, Iacs758_cal0);
            i1 = b_VbusTest2->size[0] * b_VbusTest2->size[1] * b_VbusTest2->
              size[2] * b_VbusTest2->size[3];
            b_VbusTest2->size[0] = 1;
            b_VbusTest2->size[1] = 1;
            b_VbusTest2->size[2] = VbusTest2->size[2];
            b_VbusTest2->size[3] = trueCount;
            emxEnsureCapacity_real_T(b_VbusTest2, i1);
            VmV_data = b_VbusTest2->data;
            for (i1 = 0; i1 < trueCount; i1++) {
              loop_ub = VbusTest2->size[2];
              for (i2 = 0; i2 < loop_ub; i2++) {
                VmV_data[i2 + b_VbusTest2->size[2] * i1] = VbusTest2_data
                  [((k_ina219_ + VbusTest2->size[0] * c_k_bat) + VbusTest2->
                    size[0] * VbusTest2->size[1] * i2) + VbusTest2->size[0] *
                  VbusTest2->size[1] * VbusTest2->size[2] * h_tmp_data[i1]];
              }
            }

            b_squeeze(b_VbusTest2, ImKp184Test2_0);
            i1 = b_VbusTest2->size[0] * b_VbusTest2->size[1] * b_VbusTest2->
              size[2] * b_VbusTest2->size[3];
            b_VbusTest2->size[0] = 1;
            b_VbusTest2->size[1] = 1;
            b_VbusTest2->size[2] = ImKp184Test2->size[2];
            b_VbusTest2->size[3] = trueCount;
            emxEnsureCapacity_real_T(b_VbusTest2, i1);
            VmV_data = b_VbusTest2->data;
            b_tmp_size[1] = trueCount;
            for (i1 = 0; i1 < trueCount; i1++) {
              loop_ub = ImKp184Test2->size[2];
              for (i2 = 0; i2 < loop_ub; i2++) {
                VmV_data[i2 + b_VbusTest2->size[2] * i1] = ImKp184Test2_data
                  [(k_ina219_ + ImKp184Test2->size[0] * ImKp184Test2->size[1] *
                    i2) + ImKp184Test2->size[0] * ImKp184Test2->size[1] *
                  ImKp184Test2->size[2] * h_tmp_data[i1]];
              }

              i_tmp_data[i1] = prm->seq.tst.i.NegIflag[(ipos_data[k_ina219_] +
                (h_tmp_data[i1] << 3)) - 1];
            }

            b_squeeze(b_VbusTest2, ImKp184Test2_bat_0);
            VmV_data = ImKp184Test2_bat_0->data;
            if (ImKp184Test2_bat_0->size[1] == trueCount) {
              i1 = b_ImKp184Test2_bat_0->size[0] * b_ImKp184Test2_bat_0->size[1];
              b_ImKp184Test2_bat_0->size[0] = ImKp184Test2_bat_0->size[0];
              b_ImKp184Test2_bat_0->size[1] = ImKp184Test2_bat_0->size[1];
              emxEnsureCapacity_real_T(b_ImKp184Test2_bat_0, i1);
              Vdebug_data = b_ImKp184Test2_bat_0->data;
              loop_ub = ImKp184Test2_bat_0->size[1];
              for (i1 = 0; i1 < loop_ub; i1++) {
                b_loop_ub = ImKp184Test2_bat_0->size[0];
                for (i2 = 0; i2 < b_loop_ub; i2++) {
                  Vdebug_data[i2 + b_ImKp184Test2_bat_0->size[0] * i1] =
                    VmV_data[i2 + ImKp184Test2_bat_0->size[0] * i1] * (double)
                    i_tmp_data[i1];
                }
              }

              i1 = ImKp184Test2_bat_0->size[0] * ImKp184Test2_bat_0->size[1];
              ImKp184Test2_bat_0->size[0] = b_ImKp184Test2_bat_0->size[0];
              ImKp184Test2_bat_0->size[1] = b_ImKp184Test2_bat_0->size[1];
              emxEnsureCapacity_real_T(ImKp184Test2_bat_0, i1);
              VmV_data = ImKp184Test2_bat_0->data;
              loop_ub = b_ImKp184Test2_bat_0->size[0] *
                b_ImKp184Test2_bat_0->size[1];
              for (i1 = 0; i1 < loop_ub; i1++) {
                VmV_data[i1] = Vdebug_data[i1];
              }
            } else {
              binary_expand_op(ImKp184Test2_bat_0, i_tmp_data, b_tmp_size);
            }

            VdebugVec_size_idx_0_tmp = ImKp184Test2_bat_0->size[0] *
              ImKp184Test2_bat_0->size[1];
            id2Bypass_size = ImKp184Test2_0->size[0] * ImKp184Test2_0->size[1];
            c_ImKp184Test2_bat_0 = *ImKp184Test2_bat_0;
            b_VecIna219 = VdebugVec_size_idx_0_tmp;
            c_ImKp184Test2_bat_0.size = &b_VecIna219;
            c_ImKp184Test2_bat_0.numDimensions = 1;
            b_ImKp184Test2_0 = *ImKp184Test2_0;
            d_ImKp184Test2_0 = id2Bypass_size;
            b_ImKp184Test2_0.size = &d_ImKp184Test2_0;
            b_ImKp184Test2_0.numDimensions = 1;
            polyfit(&c_ImKp184Test2_bat_0, &b_ImKp184Test2_0, outVI);
            RtotHelpPoly_data[k_ina219_ + RtotHelpPoly->size[0] * c_k_bat] =
              outVI[0];
            RtotHelpPoly_data[(k_ina219_ + RtotHelpPoly->size[0] * c_k_bat) +
              RtotHelpPoly->size[0] * RtotHelpPoly->size[1]] = outVI[1];
            trueCount = Iacs758_cal0->size[0] * Iacs758_cal0->size[1];
            c_VdebugVec_data = *Iacs758_cal0;
            c_trueCount = trueCount;
            c_VdebugVec_data.size = &c_trueCount;
            c_VdebugVec_data.numDimensions = 1;
            c_ImKp184Test2_bat_0 = *ImKp184Test2_bat_0;
            c_VecIna219 = VdebugVec_size_idx_0_tmp;
            c_ImKp184Test2_bat_0.size = &c_VecIna219;
            c_ImKp184Test2_bat_0.numDimensions = 1;
            polyfit(&c_VdebugVec_data, &c_ImKp184Test2_bat_0, outVI);
            pIshunt_data[k_ina219_ + pIshunt->size[0] * c_k_bat] = outVI[0];
            pIshunt_data[(k_ina219_ + pIshunt->size[0] * c_k_bat) +
              pIshunt->size[0] * pIshunt->size[1]] = outVI[1];

            /* polyfit(squeeze(IshuntTest2(k_ina219,k_bat,:,k_iTest)),1000*squeeze(ImKp184Test2(k_ina219,1,:,k_iTest)),1); */
          }

          /*  end */
          loop_ub = Rtot->size[1];
          for (i1 = 0; i1 < loop_ub; i1++) {
            Rtot_data[k_ina219_ + Rtot->size[0] * i1] =
              -RtotHelpPoly_data[k_ina219_ + RtotHelpPoly->size[0] * i1];
          }

          if (Rtot->size[1] == 16) {
            loop_ub = Rwire->size[1];
            for (i1 = 0; i1 < loop_ub; i1++) {
              Rwire_data[k_ina219_ + Rwire->size[0] * i1] = (float)
                Rtot_data[k_ina219_ + Rtot->size[0] * i1] - prm->
                bat.Rint[k_ina219_ + (i1 << 1)];
            }
          } else {
            binary_expand_op_1(Rwire, k_ina219_, Rtot, prm);
            Rwire_data = Rwire->data;
          }

          if (!prm->seq.tst.i.useRwireFlag) {
            loop_ub = Rwire->size[1];
            for (i1 = 0; i1 < loop_ub; i1++) {
              b_loop_ub = Rwire->size[0];
              for (i2 = 0; i2 < b_loop_ub; i2++) {
                Rwire_data[i2 + Rwire->size[0] * i1] = 0.0F;
              }
            }
          }
        }

        guard3 = true;
      } else if (prm->seq.tst.i.measRintR) {
        loop_ub = ImKp184Test2->size[2];
        for (i = 0; i < loop_ub; i++) {
          e_tmp_data[i] = ImKp184Test2_data[ImKp184Test2->size[0] *
            ImKp184Test2->size[1] * i];
        }

        if (prm->seq.tst.i.RintBatId < 1) {
          k_groups_tmp = 1;
        } else {
          k_groups_tmp = prm->seq.tst.i.RintBatId;
        }

        loop_ub = VbusTest2->size[2];
        for (i = 0; i < loop_ub; i++) {
          g_tmp_data[i] = VbusTest2_data[VbusTest2->size[0] * (k_groups_tmp - 1)
            + VbusTest2->size[0] * VbusTest2->size[1] * i];
        }

        id2Bypass_data[0] = 1;
        id2Bypass_data[1] = prm->seq.tst.i.NTtest;
        Nina219[0] = 1;
        Nina219[1] = prm->seq.tst.i.NTtest;
        b_polyfit(e_tmp_data, id2Bypass_data, g_tmp_data, Nina219, outVI);
        outStruct->Rint = -outVI[0];
        i = b_VbusTest2->size[0] * b_VbusTest2->size[1] * b_VbusTest2->size[2] *
          b_VbusTest2->size[3];
        b_VbusTest2->size[0] = 1;
        b_VbusTest2->size[1] = 1;
        b_VbusTest2->size[2] = VbusTest2->size[2];
        b_VbusTest2->size[3] = VbusTest2->size[3];
        emxEnsureCapacity_real_T(b_VbusTest2, i);
        VmV_data = b_VbusTest2->data;
        loop_ub = VbusTest2->size[3];
        for (i = 0; i < loop_ub; i++) {
          b_loop_ub = VbusTest2->size[2];
          for (i1 = 0; i1 < b_loop_ub; i1++) {
            VmV_data[i1 + b_VbusTest2->size[2] * i] = VbusTest2_data
              [(VbusTest2->size[0] * (prm->seq.tst.i.RintBatId - 1) +
                VbusTest2->size[0] * VbusTest2->size[1] * i1) + VbusTest2->size
              [0] * VbusTest2->size[1] * VbusTest2->size[2] * i];
          }
        }

        trueCount = VbusTest2->size[2] * VbusTest2->size[3];
        i = VbusTest0->size[0];
        VbusTest0->size[0] = trueCount;
        emxEnsureCapacity_real_T(VbusTest0, i);
        Vdebug_data = VbusTest0->data;
        for (i = 0; i < trueCount; i++) {
          Vdebug_data[i] = VmV_data[i];
        }

        outStruct->VpassFlag = true;
        VbusTest0_size = VbusTest0->size[0];
        loop_ub = VbusTest0->size[0];
        for (i = 0; i < loop_ub; i++) {
          VbusTest0_data[i] = (Vdebug_data[i] > prm->bat.Vmax);
        }

        b_VbusTest0_data.data = &VbusTest0_data[0];
        b_VbusTest0_data.size = &VbusTest0_size;
        b_VbusTest0_data.allocatedSize = 32385;
        b_VbusTest0_data.numDimensions = 1;
        b_VbusTest0_data.canFreeData = false;
        if (any(&b_VbusTest0_data)) {
          outStruct->VpassFlag = false;
        } else {
          VbusTest0_size = VbusTest0->size[0];
          loop_ub = VbusTest0->size[0];
          for (i = 0; i < loop_ub; i++) {
            VbusTest0_data[i] = (Vdebug_data[i] < prm->bat.Vmin);
          }

          c_VbusTest0_data.data = &VbusTest0_data[0];
          c_VbusTest0_data.size = &VbusTest0_size;
          c_VbusTest0_data.allocatedSize = 32385;
          c_VbusTest0_data.numDimensions = 1;
          c_VbusTest0_data.canFreeData = false;
          if (any(&c_VbusTest0_data)) {
            outStruct->VpassFlag = false;
          } else {
            j_sprintf(prm->seq.tst.i.RintBatId, c_prm);
            id2Bypass_data[0] = 1;
            id2Bypass_data[1] = prm->seq.tst.i.NTtest;
            Nina219[0] = 1;
            Nina219[1] = prm->seq.tst.i.NTtest;
            b_polyfit(e_tmp_data, id2Bypass_data, g_tmp_data, Nina219, outVI);
            d_sprintf(-outVI[0], r12);
          }
        }
      } else {
        guard3 = true;
      }
    } else {
      b_loop_ub_tmp = (int)prm->brd.Nina219;
      i = pIshunt->size[0] * pIshunt->size[1] * pIshunt->size[2];
      pIshunt->size[0] = (int)prm->brd.Nina219;
      k_t0 = prm->brd.N_bat;
      pIshunt->size[1] = prm->brd.N_bat;
      pIshunt->size[2] = 2;
      emxEnsureCapacity_real_T(pIshunt, i);
      pIshunt_data = pIshunt->data;
      loop_ub = ((int)prm->brd.Nina219 * prm->brd.N_bat) << 1;
      for (i = 0; i < loop_ub; i++) {
        pIshunt_data[i] = 0.0;
      }

      for (i = 0; i < k_t0; i++) {
        for (i1 = 0; i1 < b_loop_ub_tmp; i1++) {
          pIshunt_data[i1 + pIshunt->size[0] * i] = 1.0;
        }
      }

      /* pIshunt*0; */
      loop_ub = pIshunt->size[1];
      for (i = 0; i < loop_ub; i++) {
        b_loop_ub = pIshunt->size[0];
        for (i1 = 0; i1 < b_loop_ub; i1++) {
          pIshunt_data[(i1 + pIshunt->size[0] * i) + pIshunt->size[0] *
            pIshunt->size[1]] = 0.0;
        }
      }

      guard1 = true;
    }
  }

  if (guard3) {
    /*  if 0 */
    /*      I2switchId = [I2switchId 2];%add bypass */
    /*      pIshuntAll(:,:,:,1+end) = zeros(Nina219,Nbat,2); */
    /*      pIshunt(:,2:2:end,:) = 0;%pac even disconnected */
    for (k_ina219_ = 0; k_ina219_ < i; k_ina219_++) {
      switch (ProjectFlag) {
       case 2:
        /* esp32 */
        break;

       case 3:
        /* esp32 ser */
        /* UNTITLED Summary of this function goes here */
        /*    Detailed explanation goes here */
        /*  if coder.target('MATLAB') */
        /*  read current */
        /*   chip1     |   chip2     |   chip3 */
        /* %comN2 Port4 chip1,comN1 Port11 chip1 */
        /* %comP1 Port18 chip1 ,comP2 Port18 chip3 */
        /*  */
        /*  for k_row = 1:N_row */
        /*      out123(k_row,:) = readSPI(SPI,PortSpiRow(k_row,1),N_chip); */
        /*      oldPortSpiRow(k_row,1:2:end) = PortSpiRow(k_row,1:2:end); */
        /*      oldPortSpiRow(k_row,2:2:end) = out123(k_row,2:2:end); */
        /*  end */
        /* mask current */
        /* write current mask */
        /*  for k_row = 1:N_row */
        /*      write(SPI,oldPortSpiRowMask(k_row,:)); */
        /*  end */
        pause(0.001);

        /* mask new */
        /* write mask new */
        /*  for k_row = 1:N_row */
        /*      write(SPI0,PortSpiRowMask(k_row,:)); */
        /*  end */
        /* write new */
        /*  for k_row = 1:N_row */
        /*      write(SPI0,PortSpiRow(k_row,:)); */
        /*  end */
        /*  Rmsg = SPI_writeread(SPI0, PortSpiRow); */
        /*  ReadPortSpiRow = RmsgData([[2:4] , 1] ,:); */
        /*  ReadPortSpiRow(:,1:2:end) = ReadPortSpiRow(:,1:2:end);% - 128; */
        /* mask current */
        /* %chip1 Port6 Alert1, chip1 port 7 Alert3, chip1 port 8 Alert8, chip1 port 9 Alert6 */
        /* %chip1 Port12 Alert7, chip1 Port13 Alert5, chip 1 port19 NC */
        /* %chip1 Port20-23 NC */
        /* chip1 Port28 Alert1, chip1 Port29 Alert2 */
        /*  if sum(sum(sum(abs(double(PortSpiRow)-double(ReadPortSpiRow)))))>0 */
        /*  end */
        break;
      }
    }

    loop_ub = pIshunt->size[1];
    for (i = 0; i < loop_ub; i++) {
      b_loop_ub = pIshunt->size[0];
      for (i1 = 0; i1 < b_loop_ub; i1++) {
        pIshunt_data[i1 + pIshunt->size[0] * i] = 1.0;
      }
    }

    /* pIshunt*0; */
    loop_ub = pIshunt->size[1];
    for (i = 0; i < loop_ub; i++) {
      b_loop_ub = pIshunt->size[0];
      for (i1 = 0; i1 < b_loop_ub; i1++) {
        pIshunt_data[(i1 + pIshunt->size[0] * i) + pIshunt->size[0] *
          pIshunt->size[1]] = 0.0;
      }
    }

    /*  end */
    /*  pIshunt = squeeze(pIshunt); */
    /*  pIshunt(:,:,1) = 1;pIshunt(:,:,1) = 0; */
    guard1 = true;
  }

  if (guard2) {
    outStruct->VpassFlag = false;
    if (prm->brd.Nina219 < 1.0) {
      loop_ub = 0;
    } else {
      loop_ub = (int)prm->brd.Nina219;
    }

    b_loop_ub = outStruct->VbusTest->size[1];
    for (i = 0; i < b_loop_ub; i++) {
      for (i1 = 0; i1 < loop_ub; i1++) {
        outStruct->VbusTest->data[i1 + outStruct->VbusTest->size[0] * i] =
          VbusTest_data[i1 + loop_ub * i];
      }
    }

    b_loop_ub = outStruct->VmKp184Test->size[1];
    for (i = 0; i < b_loop_ub; i++) {
      for (i1 = 0; i1 < loop_ub; i1++) {
        outStruct->VmKp184Test->data[i1 + outStruct->VmKp184Test->size[0] * i] =
          VmKp184Test_data[i1 + loop_ub * i];
      }
    }
  }

  if (guard1 && ((!prm->seq.tst.savePrmFlag) || (!prm->seq.tst.v.isPrm) ||
                 (!prm->seq.tst.i.isTest))) {
    /*  RintTotal = zeros(Nina219,N_bat); */
    /*  for k_ina219 = 1:Nina219 */
    /*      for k_bat = 1:N_bat */
    /*          RintTotal0 = polyfit(i_in_test,squeeze(VbusTest2(k_ina219,k_bat,:,k_ina219))',1); */
    /*          RintTotal(k_ina219,k_bat) = -RintTotal0(1); */
    /*      end */
    /*  end */
    /*  BIT */
    b_loop_ub_tmp = (int)prm->brd.Nina219;
    i = Pac2Vid0All->size[0] * Pac2Vid0All->size[1] * Pac2Vid0All->size[2];
    Pac2Vid0All->size[0] = (int)prm->brd.Nina219;
    Pac2Vid0All->size[1] = prm->brd.N_bat;
    Pac2Vid0All->size[2] = prm->brd.N_bat;
    emxEnsureCapacity_uint8_T(Pac2Vid0All, i);
    Pac2Vid0All_data = Pac2Vid0All->data;
    k_t0 = (int)prm->brd.Nina219 * prm->brd.N_bat;
    c_k_Ttest = k_t0 * prm->brd.N_bat;
    for (i = 0; i < c_k_Ttest; i++) {
      Pac2Vid0All_data[i] = 0U;
    }

    if (prm->seq.bit.bit_flag) {
      /* TODO groups */
      /*  meanVbusTest = mean(VbusTest,3);%(k_ina219,k_bat,k_Ttest) */
      N_bitCnfg = BuildBitCnfg(prm->brd.N_bat1, prm->brd.N_bat2, prm->brd.N_bat,
        prm->Nmax.NbatMax, bitCnfg, bitCnfgStr);
      bitCnfg_data = bitCnfg->data;

      /*      N_bitCnfg = length(bitCnfg); */
      /*  [N_bitI,Ngroups,NstBit] = size(seqBit.IdisChr); */
      /*  N_bitIdis = length(prm.bit.Idis); */
      /*  N_bitIchr = length(prm.bit.Ichr); */
      i = Vbrd->size[0] * Vbrd->size[1] * Vbrd->size[2] * Vbrd->size[3] *
        Vbrd->size[4];
      Vbrd->size[0] = N_bitCnfg;
      Vbrd->size[1] = (int)prm->brd.Nina219;
      Vbrd->size[2] = 1;
      Vbrd->size[3] = prm->brd.N_bat;
      i1 = prm->seq.bit.Nst;
      Vbrd->size[4] = prm->seq.bit.Nst;
      emxEnsureCapacity_real_T(Vbrd, i);
      VmV_data = Vbrd->data;
      id2Bypass_size = N_bitCnfg * (int)prm->brd.Nina219;
      VdebugVec_size_idx_0_tmp = id2Bypass_size * prm->brd.N_bat *
        prm->seq.bit.Nst;
      for (i = 0; i < VdebugVec_size_idx_0_tmp; i++) {
        VmV_data[i] = 0.0;
      }

      i = Ibrd->size[0] * Ibrd->size[1] * Ibrd->size[2] * Ibrd->size[3] *
        Ibrd->size[4];
      Ibrd->size[0] = N_bitCnfg;
      Ibrd->size[1] = (int)prm->brd.Nina219;
      Ibrd->size[2] = 1;
      Ibrd->size[3] = prm->brd.N_bat;
      Ibrd->size[4] = prm->seq.bit.Nst;
      emxEnsureCapacity_real_T(Ibrd, i);
      pIshunt_data = Ibrd->data;
      for (i = 0; i < VdebugVec_size_idx_0_tmp; i++) {
        pIshunt_data[i] = 0.0;
      }

      /*  VbrdChr = zeros(N_bitCnfg,Nina219,N_bitIchr,N_bat); */
      /*  IbrdChr = zeros(N_bitCnfg,Nina219,N_bitIchr,N_bat); */
      i = meanIbrd->size[0] * meanIbrd->size[1] * meanIbrd->size[2] *
        meanIbrd->size[3];
      meanIbrd->size[0] = N_bitCnfg;
      meanIbrd->size[1] = (int)prm->brd.Nina219;
      meanIbrd->size[2] = 1;
      meanIbrd->size[3] = prm->seq.bit.Nst;
      emxEnsureCapacity_real_T(meanIbrd, i);
      RtotHelpPoly_data = meanIbrd->data;
      VdebugVec_size_idx_0_tmp = id2Bypass_size * prm->seq.bit.Nst;
      for (i = 0; i < VdebugVec_size_idx_0_tmp; i++) {
        RtotHelpPoly_data[i] = 0.0;
      }

      /*  meanIbrdChr = zeros(N_bitCnfg,Nina219,N_bitIchr); */
      i = VbitInsMeas->size[0] * VbitInsMeas->size[1] * VbitInsMeas->size[2] *
        VbitInsMeas->size[3];
      VbitInsMeas->size[0] = N_bitCnfg;
      VbitInsMeas->size[1] = (int)prm->brd.Nina219;
      VbitInsMeas->size[2] = 1;
      VbitInsMeas->size[3] = prm->seq.bit.Nst;
      emxEnsureCapacity_real_T(VbitInsMeas, i);
      VbusTest2_data = VbitInsMeas->data;
      for (i = 0; i < VdebugVec_size_idx_0_tmp; i++) {
        VbusTest2_data[i] = 0.0;
      }

      /*  VbitInsMeasChr = zeros(N_bitCnfg,Nina219,N_bitIdis); */
      /*  IbitInsMeasChr = zeros(N_bitCnfg,Nina219,N_bitIdis); */
      if (prm->brd.Nina219 < 1.0) {
        VecIna219->size[0] = 1;
        VecIna219->size[1] = 0;
      } else {
        i = VecIna219->size[0] * VecIna219->size[1];
        VecIna219->size[0] = 1;
        VecIna219->size[1] = (int)(prm->brd.Nina219 - 1.0) + 1;
        emxEnsureCapacity_real_T(VecIna219, i);
        VecIna219_data = VecIna219->data;
        loop_ub = (int)(prm->brd.Nina219 - 1.0);
        for (i = 0; i <= loop_ub; i++) {
          VecIna219_data[i] = (double)i + 1.0;
        }
      }

      u = (unsigned short)N_bitCnfg;
      if ((unsigned short)N_bitCnfg > 32767) {
        u = 32767U;
      }

      i = u;
      for (b_k_tstState = 0; b_k_tstState < i; b_k_tstState++) {
        for (b_end = 0; b_end < i1; b_end++) {
          for (k_groups = 0; k_groups < b_size; k_groups++) {
            end = ipos_size - 1;
            trueCount = 0;
            loop_ub = 0;
            for (b_i = 0; b_i <= end; b_i++) {
              if (k_groups + 1 == ipos_data[b_i]) {
                trueCount++;
                d_tmp_data[loop_ub] = (signed char)b_i;
                loop_ub++;
              }
            }

            /* VecIna219_k = find(k_groups == uGroupId); */
            for (k0 = 0; k0 < trueCount; k0++) {
              /* for k_ina219 = VecIna219_k */
              if (prm->seq.bit.BrdBeforePSflag[b_end] != 0) {
                V_chr_RI = VecIna219_data[d_tmp_data[k0]];
                for (i2 = 0; i2 < trueCount; i2++) {
                  b_VecIna219_data[i2] = (VecIna219_data[d_tmp_data[i2]] !=
                    V_chr_RI);
                }

                id2Bypass_size = c_eml_find(b_VecIna219_data, trueCount,
                  id2Bypass_data);
                for (b_i = 0; b_i < id2Bypass_size; b_i++) {
                  /* for k_bypass = VecIna219(id2Bypass) */
                  switch (ProjectFlag) {
                   case 2:
                    /* ESP32 */
                    /* TODO bypass */
                    break;

                   case 3:
                    /* ESP32 ser */
                    /* UNTITLED Summary of this function goes here */
                    /*    Detailed explanation goes here */
                    /*  if coder.target('MATLAB') */
                    /*  read current */
                    /*   chip1     |   chip2     |   chip3 */
                    /* %comN2 Port4 chip1,comN1 Port11 chip1 */
                    /* %comP1 Port18 chip1 ,comP2 Port18 chip3 */
                    /*  */
                    /*  for k_row = 1:N_row */
                    /*      out123(k_row,:) = readSPI(SPI,PortSpiRow(k_row,1),N_chip); */
                    /*      oldPortSpiRow(k_row,1:2:end) = PortSpiRow(k_row,1:2:end); */
                    /*      oldPortSpiRow(k_row,2:2:end) = out123(k_row,2:2:end); */
                    /*  end */
                    /* mask current */
                    /* write current mask */
                    /*  for k_row = 1:N_row */
                    /*      write(SPI,oldPortSpiRowMask(k_row,:)); */
                    /*  end */
                    pause(0.001);

                    /* mask new */
                    /* write mask new */
                    /*  for k_row = 1:N_row */
                    /*      write(SPI0,PortSpiRowMask(k_row,:)); */
                    /*  end */
                    /* write new */
                    /*  for k_row = 1:N_row */
                    /*      write(SPI0,PortSpiRow(k_row,:)); */
                    /*  end */
                    /*  Rmsg = SPI_writeread(SPI0, PortSpiRow); */
                    /*  ReadPortSpiRow = RmsgData([[2:4] , 1] ,:); */
                    /*  ReadPortSpiRow(:,1:2:end) = ReadPortSpiRow(:,1:2:end);% - 128; */
                    /* mask current */
                    /* %chip1 Port6 Alert1, chip1 port 7 Alert3, chip1 port 8 Alert8, chip1 port 9 Alert6 */
                    /* %chip1 Port12 Alert7, chip1 Port13 Alert5, chip 1 port19 NC */
                    /* %chip1 Port20-23 NC */
                    /* chip1 Port28 Alert1, chip1 Port29 Alert2 */
                    /*  if sum(sum(sum(abs(double(PortSpiRow)-double(ReadPortSpiRow)))))>0 */
                    /*  end */
                    /* TODO bypass */
                    break;
                  }

                  pause(0.1);
                }

                i2 = b_bitCnfg->size[0] * b_bitCnfg->size[1] * b_bitCnfg->size[2];
                b_bitCnfg->size[0] = 1;
                b_bitCnfg->size[1] = bitCnfg->size[1];
                loop_ub = bitCnfg->size[2];
                b_bitCnfg->size[2] = bitCnfg->size[2];
                emxEnsureCapacity_uint8_T(b_bitCnfg, i2);
                Pac2Vid31_data = b_bitCnfg->data;
                for (i2 = 0; i2 < loop_ub; i2++) {
                  b_loop_ub = bitCnfg->size[1];
                  for (i3 = 0; i3 < b_loop_ub; i3++) {
                    Pac2Vid31_data[i3 + b_bitCnfg->size[1] * i2] = bitCnfg_data
                      [(b_k_tstState + bitCnfg->size[0] * i3) + bitCnfg->size[0]
                      * bitCnfg->size[1] * i2];
                  }
                }

                c_bitCnfg = *b_bitCnfg;
                c_Nina219[0] = prm->Nmax.NbatMax;
                c_Nina219[1] = prm->Nmax.NbatMax;
                c_bitCnfg.size = &c_Nina219[0];
                c_bitCnfg.numDimensions = 2;
                CalcPortSpiPrm(prm->brd.spi.disconnect, prm->brd.spi.bypass,
                               prm->brd.spi.PortSpiRow_esp,
                               prm->brd.spi.SwitchMat_esp,
                               prm->brd.spi.PortSpiRow_esp2,
                               prm->brd.spi.SwitchMat_esp2, prm->brd.spi.Pac2Vid,
                               prm->brd.spi.Pac2Vid2, &c_bitCnfg, disConAll,
                               a__1_data, id2Bypass_data, a__2);
                k_ina219_ = Pac2Vid0All->size[1];
                loop_ub = Pac2Vid0All->size[2];
                for (i2 = 0; i2 < loop_ub; i2++) {
                  for (i3 = 0; i3 < k_ina219_; i3++) {
                    Pac2Vid0All_data[(((int)V_chr_RI + Pac2Vid0All->size[0] * i3)
                                      + Pac2Vid0All->size[0] * Pac2Vid0All->
                                      size[1] * i2) - 1] = a__2[i3 + k_ina219_ *
                      i2];
                  }
                }

                /* sort(bitCnfg{k_bitCnfg})); */
                switch (ProjectFlag) {
                 case 2:
                  /* esp32 */
                  break;

                 case 3:
                  /* esp32 ser */
                  /* UNTITLED Summary of this function goes here */
                  /*    Detailed explanation goes here */
                  /*  if coder.target('MATLAB') */
                  /*  read current */
                  /*   chip1     |   chip2     |   chip3 */
                  /* %comN2 Port4 chip1,comN1 Port11 chip1 */
                  /* %comP1 Port18 chip1 ,comP2 Port18 chip3 */
                  /*  */
                  /*  for k_row = 1:N_row */
                  /*      out123(k_row,:) = readSPI(SPI,PortSpiRow(k_row,1),N_chip); */
                  /*      oldPortSpiRow(k_row,1:2:end) = PortSpiRow(k_row,1:2:end); */
                  /*      oldPortSpiRow(k_row,2:2:end) = out123(k_row,2:2:end); */
                  /*  end */
                  /* mask current */
                  /* write current mask */
                  /*  for k_row = 1:N_row */
                  /*      write(SPI,oldPortSpiRowMask(k_row,:)); */
                  /*  end */
                  pause(0.001);

                  /* mask new */
                  /* write mask new */
                  /*  for k_row = 1:N_row */
                  /*      write(SPI0,PortSpiRowMask(k_row,:)); */
                  /*  end */
                  /* write new */
                  /*  for k_row = 1:N_row */
                  /*      write(SPI0,PortSpiRow(k_row,:)); */
                  /*  end */
                  /*  Rmsg = SPI_writeread(SPI0, PortSpiRow); */
                  /*  ReadPortSpiRow = RmsgData([[2:4] , 1] ,:); */
                  /*  ReadPortSpiRow(:,1:2:end) = ReadPortSpiRow(:,1:2:end);% - 128; */
                  /* mask current */
                  /* %chip1 Port6 Alert1, chip1 port 7 Alert3, chip1 port 8 Alert8, chip1 port 9 Alert6 */
                  /* %chip1 Port12 Alert7, chip1 Port13 Alert5, chip 1 port19 NC */
                  /* %chip1 Port20-23 NC */
                  /* chip1 Port28 Alert1, chip1 Port29 Alert2 */
                  /*  if sum(sum(sum(abs(double(PortSpiRow)-double(ReadPortSpiRow)))))>0 */
                  /*  end */
                  break;
                }
              } else {
                i2 = b_bitCnfg->size[0] * b_bitCnfg->size[1] * b_bitCnfg->size[2];
                b_bitCnfg->size[0] = 1;
                b_bitCnfg->size[1] = bitCnfg->size[1];
                loop_ub = bitCnfg->size[2];
                b_bitCnfg->size[2] = bitCnfg->size[2];
                emxEnsureCapacity_uint8_T(b_bitCnfg, i2);
                Pac2Vid31_data = b_bitCnfg->data;
                for (i2 = 0; i2 < loop_ub; i2++) {
                  b_loop_ub = bitCnfg->size[1];
                  for (i3 = 0; i3 < b_loop_ub; i3++) {
                    Pac2Vid31_data[i3 + b_bitCnfg->size[1] * i2] = bitCnfg_data
                      [(b_k_tstState + bitCnfg->size[0] * i3) + bitCnfg->size[0]
                      * bitCnfg->size[1] * i2];
                  }
                }

                c_bitCnfg = *b_bitCnfg;
                b_Nina219[0] = prm->Nmax.NbatMax;
                b_Nina219[1] = prm->Nmax.NbatMax;
                c_bitCnfg.size = &b_Nina219[0];
                c_bitCnfg.numDimensions = 2;
                CalcPortSpiPrm(prm->brd.spi.disconnect, prm->brd.spi.bypass,
                               prm->brd.spi.PortSpiRow_esp,
                               prm->brd.spi.SwitchMat_esp,
                               prm->brd.spi.PortSpiRow_esp2,
                               prm->brd.spi.SwitchMat_esp2, prm->brd.spi.Pac2Vid,
                               prm->brd.spi.Pac2Vid2, &c_bitCnfg, disConAll,
                               a__1_data, id2Bypass_data, a__2);
                k_ina219_ = Pac2Vid0All->size[1];
                id2Bypass_size = (int)VecIna219_data[d_tmp_data[k0]];
                loop_ub = Pac2Vid0All->size[2];
                for (i2 = 0; i2 < loop_ub; i2++) {
                  for (i3 = 0; i3 < k_ina219_; i3++) {
                    Pac2Vid0All_data[((id2Bypass_size + Pac2Vid0All->size[0] *
                                       i3) + Pac2Vid0All->size[0] *
                                      Pac2Vid0All->size[1] * i2) - 1] = a__2[i3
                      + k_ina219_ * i2];
                  }
                }

                /* sort(bitCnfg{k_bitCnfg}));sort(bitCnfg{k_bitCnfg})); */
              }

              /*  calc V */
              V_chr_RI = VecIna219_data[d_tmp_data[k0]];
              i2 = b_bitCnfg->size[0] * b_bitCnfg->size[1] * b_bitCnfg->size[2];
              b_bitCnfg->size[0] = 1;
              b_bitCnfg->size[1] = Pac2Vid0All->size[1];
              loop_ub = Pac2Vid0All->size[2];
              b_bitCnfg->size[2] = Pac2Vid0All->size[2];
              emxEnsureCapacity_uint8_T(b_bitCnfg, i2);
              Pac2Vid31_data = b_bitCnfg->data;
              for (i2 = 0; i2 < loop_ub; i2++) {
                b_loop_ub = Pac2Vid0All->size[1];
                for (i3 = 0; i3 < b_loop_ub; i3++) {
                  Pac2Vid31_data[i3 + b_bitCnfg->size[1] * i2] =
                    Pac2Vid0All_data[(((int)V_chr_RI + Pac2Vid0All->size[0] * i3)
                                      + Pac2Vid0All->size[0] * Pac2Vid0All->
                                      size[1] * i2) - 1];
                }
              }

              c_squeeze(b_bitCnfg, Pac2Vid31);
              Pac2Vid31_data = Pac2Vid31->data;

              /* Pac2Vid3(:,1); */
              end = Pac2Vid31->size[0] * Pac2Vid31->size[1] - 1;
              state_k = 0;
              for (b_i = 0; b_i <= end; b_i++) {
                if (Pac2Vid31_data[b_i] > 0) {
                  state_k++;
                }
              }

              c_vSumMax = prm->bat.Vd + (prm->bat.CutOffChrV[0] + 0.1F) * (float)
                state_k;

              /*  sumV      = sum(meanVbusTest(k_ina219,Pac2Vid31)); */
              switch (prm->seq.bit.ins[k_groups + (b_end << 3)]) {
               case 1:
                /* kunkin kp184 */
                /* CC */
                pause(0.05);
                pause(0.05);
                break;

               case 2:
                /* ka6005p */
                e_sprintf(c_vSumMax, b_vSumMax);

                /* ['VSET',num2str(ch),':',num2str(Val)];%[V] */
                pause(0.05);
                f_sprintf(prm->seq.bit.IdisChr[k_groups + (b_end << 3)], d_prm);

                /* ['ISET',num2str(ch),':',num2str(Val)];%[A] */
                pause(0.05);
                break;

               case 3:
                /* juntek+ACDC */
                /* Imax); */
                i_prm[0] = prm->seq.bit.IdisChr[k_groups + (b_end << 3)];
                i_prm[1] = ImaxAcDC;
                f = rt_roundf_snf(rt_roundf_snf(1000.0F * minimum(i_prm)) /
                                  1000.0F * 1000.0F);
                if (f < 2.14748365E+9F) {
                  if (f >= -2.14748365E+9F) {
                    i2 = (int)f;
                  } else {
                    i2 = MIN_int32_T;
                  }
                } else if (f >= 2.14748365E+9F) {
                  i2 = MAX_int32_T;
                } else {
                  i2 = 0;
                }

                g_sprintf(i2, r13);

                /* [':',juntek_address,'w11=',num2str(uint32(Val*1000),'%04d'),',',newline];%['VSET',num2str(ch),':',num2str(Val)];%[V] */
                /*  disp(['Juntek: I=',num2str(IchargeBit)]); */
                pause(0.1);
                f = rt_roundf_snf(c_vSumMax * 100.0F);
                if (f < 2.14748365E+9F) {
                  if (f >= -2.14748365E+9F) {
                    i2 = (int)f;
                  } else {
                    i2 = MIN_int32_T;
                  }
                } else if (f >= 2.14748365E+9F) {
                  i2 = MAX_int32_T;
                } else {
                  i2 = 0;
                }

                g_sprintf(i2, r14);

                /* [':',juntek_address,'w10=',num2str(uint32(Val*100),'%04d'),',',newline];%['VSET',num2str(ch),':',num2str(Val)];%[V] */
                /*  disp(['Juntek VSET=',num2str(vSumMax)]); */
                pause(0.1);

                /*  disp('juntek ON'); */
                break;
              }

              pause(0.2);

              /* switch board */
              if (prm->seq.bit.BrdBeforePSflag[b_end] == 0) {
                for (i2 = 0; i2 < trueCount; i2++) {
                  b_VecIna219_data[i2] = (VecIna219_data[d_tmp_data[i2]] !=
                    V_chr_RI);
                }

                id2Bypass_size = c_eml_find(b_VecIna219_data, trueCount,
                  id2Bypass_data);
                for (b_i = 0; b_i < id2Bypass_size; b_i++) {
                  /* for k_bypass = VecIna219(id2Bypass) */
                  /*                                for k_bypass = VecIna219(id2Bypass) */
                  switch (ProjectFlag) {
                   case 2:
                    /* ESP32 */
                    /* TODO bypass */
                    break;

                   case 3:
                    /* ESP32 ser */
                    /* UNTITLED Summary of this function goes here */
                    /*    Detailed explanation goes here */
                    /*  if coder.target('MATLAB') */
                    /*  read current */
                    /*   chip1     |   chip2     |   chip3 */
                    /* %comN2 Port4 chip1,comN1 Port11 chip1 */
                    /* %comP1 Port18 chip1 ,comP2 Port18 chip3 */
                    /*  */
                    /*  for k_row = 1:N_row */
                    /*      out123(k_row,:) = readSPI(SPI,PortSpiRow(k_row,1),N_chip); */
                    /*      oldPortSpiRow(k_row,1:2:end) = PortSpiRow(k_row,1:2:end); */
                    /*      oldPortSpiRow(k_row,2:2:end) = out123(k_row,2:2:end); */
                    /*  end */
                    /* mask current */
                    /* write current mask */
                    /*  for k_row = 1:N_row */
                    /*      write(SPI,oldPortSpiRowMask(k_row,:)); */
                    /*  end */
                    pause(0.001);

                    /* mask new */
                    /* write mask new */
                    /*  for k_row = 1:N_row */
                    /*      write(SPI0,PortSpiRowMask(k_row,:)); */
                    /*  end */
                    /* write new */
                    /*  for k_row = 1:N_row */
                    /*      write(SPI0,PortSpiRow(k_row,:)); */
                    /*  end */
                    /*  Rmsg = SPI_writeread(SPI0, PortSpiRow); */
                    /*  ReadPortSpiRow = RmsgData([[2:4] , 1] ,:); */
                    /*  ReadPortSpiRow(:,1:2:end) = ReadPortSpiRow(:,1:2:end);% - 128; */
                    /* mask current */
                    /* %chip1 Port6 Alert1, chip1 port 7 Alert3, chip1 port 8 Alert8, chip1 port 9 Alert6 */
                    /* %chip1 Port12 Alert7, chip1 Port13 Alert5, chip 1 port19 NC */
                    /* %chip1 Port20-23 NC */
                    /* chip1 Port28 Alert1, chip1 Port29 Alert2 */
                    /*  if sum(sum(sum(abs(double(PortSpiRow)-double(ReadPortSpiRow)))))>0 */
                    /*  end */
                    /* TODO bypass */
                    break;
                  }

                  pause(0.1);
                }

                i2 = b_bitCnfg->size[0] * b_bitCnfg->size[1] * b_bitCnfg->size[2];
                b_bitCnfg->size[0] = 1;
                b_bitCnfg->size[1] = bitCnfg->size[1];
                loop_ub = bitCnfg->size[2];
                b_bitCnfg->size[2] = bitCnfg->size[2];
                emxEnsureCapacity_uint8_T(b_bitCnfg, i2);
                Pac2Vid31_data = b_bitCnfg->data;
                for (i2 = 0; i2 < loop_ub; i2++) {
                  b_loop_ub = bitCnfg->size[1];
                  for (i3 = 0; i3 < b_loop_ub; i3++) {
                    Pac2Vid31_data[i3 + b_bitCnfg->size[1] * i2] = bitCnfg_data
                      [(b_k_tstState + bitCnfg->size[0] * i3) + bitCnfg->size[0]
                      * bitCnfg->size[1] * i2];
                  }
                }

                c_bitCnfg = *b_bitCnfg;
                d_Nina219[0] = prm->Nmax.NbatMax;
                d_Nina219[1] = prm->Nmax.NbatMax;
                c_bitCnfg.size = &d_Nina219[0];
                c_bitCnfg.numDimensions = 2;
                CalcPortSpiPrm(prm->brd.spi.disconnect, prm->brd.spi.bypass,
                               prm->brd.spi.PortSpiRow_esp,
                               prm->brd.spi.SwitchMat_esp,
                               prm->brd.spi.PortSpiRow_esp2,
                               prm->brd.spi.SwitchMat_esp2, prm->brd.spi.Pac2Vid,
                               prm->brd.spi.Pac2Vid2, &c_bitCnfg, disConAll,
                               a__1_data, id2Bypass_data, a__2);
                k_ina219_ = Pac2Vid0All->size[1];
                loop_ub = Pac2Vid0All->size[2];
                for (i2 = 0; i2 < loop_ub; i2++) {
                  for (i3 = 0; i3 < k_ina219_; i3++) {
                    Pac2Vid0All_data[(((int)V_chr_RI + Pac2Vid0All->size[0] * i3)
                                      + Pac2Vid0All->size[0] * Pac2Vid0All->
                                      size[1] * i2) - 1] = a__2[i3 + k_ina219_ *
                      i2];
                  }
                }

                /* sort(bitCnfg{k_bitCnfg})); */
                switch (ProjectFlag) {
                 case 2:
                  /* esp32 */
                  break;

                 case 3:
                  /* esp32 ser */
                  /* UNTITLED Summary of this function goes here */
                  /*    Detailed explanation goes here */
                  /*  if coder.target('MATLAB') */
                  /*  read current */
                  /*   chip1     |   chip2     |   chip3 */
                  /* %comN2 Port4 chip1,comN1 Port11 chip1 */
                  /* %comP1 Port18 chip1 ,comP2 Port18 chip3 */
                  /*  */
                  /*  for k_row = 1:N_row */
                  /*      out123(k_row,:) = readSPI(SPI,PortSpiRow(k_row,1),N_chip); */
                  /*      oldPortSpiRow(k_row,1:2:end) = PortSpiRow(k_row,1:2:end); */
                  /*      oldPortSpiRow(k_row,2:2:end) = out123(k_row,2:2:end); */
                  /*  end */
                  /* mask current */
                  /* write current mask */
                  /*  for k_row = 1:N_row */
                  /*      write(SPI,oldPortSpiRowMask(k_row,:)); */
                  /*  end */
                  pause(0.001);

                  /* mask new */
                  /* write mask new */
                  /*  for k_row = 1:N_row */
                  /*      write(SPI0,PortSpiRowMask(k_row,:)); */
                  /*  end */
                  /* write new */
                  /*  for k_row = 1:N_row */
                  /*      write(SPI0,PortSpiRow(k_row,:)); */
                  /*  end */
                  /*  Rmsg = SPI_writeread(SPI0, PortSpiRow); */
                  /*  ReadPortSpiRow = RmsgData([[2:4] , 1] ,:); */
                  /*  ReadPortSpiRow(:,1:2:end) = ReadPortSpiRow(:,1:2:end);% - 128; */
                  /* mask current */
                  /* %chip1 Port6 Alert1, chip1 port 7 Alert3, chip1 port 8 Alert8, chip1 port 9 Alert6 */
                  /* %chip1 Port12 Alert7, chip1 Port13 Alert5, chip 1 port19 NC */
                  /* %chip1 Port20-23 NC */
                  /* chip1 Port28 Alert1, chip1 Port29 Alert2 */
                  /*  if sum(sum(sum(abs(double(PortSpiRow)-double(ReadPortSpiRow)))))>0 */
                  /*  end */
                  break;
                }
              }

              /* switch board - End */
              pause(0.2);
              switch (ProjectFlag) {
               case 2:
                /* esp32 */
                /*  [VbrdChr(k_bitCnfg,k_ina219,k_bitIchr,:),IbrdChr(k_bitCnfg,k_ina219,k_bitIchr,:),~,~,errI2C,~] = readI2cVIfastTicTocV5_Rratio(Esp32_v1(k_ina219),true,true,true,VIpacId(k_ina219,:),Pac2Vid0,squeeze(polyFitPacV(k_ina219,:,:)),squeeze(Rval(k_ina219,:,:)),N_bat1,N_bat2); */
                for (i2 = 0; i2 < 16; i2++) {
                  g_prm[i2] = prm->brd.pac.VIpacId[((int)V_chr_RI + (i2 << 1)) -
                    1];
                }

                i2 = b_bitCnfg->size[0] * b_bitCnfg->size[1] * b_bitCnfg->size[2];
                b_bitCnfg->size[0] = 1;
                b_bitCnfg->size[1] = Pac2Vid0All->size[1];
                loop_ub = Pac2Vid0All->size[2];
                b_bitCnfg->size[2] = Pac2Vid0All->size[2];
                emxEnsureCapacity_uint8_T(b_bitCnfg, i2);
                Pac2Vid31_data = b_bitCnfg->data;
                for (i2 = 0; i2 < loop_ub; i2++) {
                  b_loop_ub = Pac2Vid0All->size[1];
                  for (i3 = 0; i3 < b_loop_ub; i3++) {
                    Pac2Vid31_data[i3 + b_bitCnfg->size[1] * i2] =
                      Pac2Vid0All_data[(((int)V_chr_RI + Pac2Vid0All->size[0] *
                                         i3) + Pac2Vid0All->size[0] *
                                        Pac2Vid0All->size[1] * i2) - 1];
                  }
                }

                i_prm[0] = pIacs758_data[(int)V_chr_RI - 1];
                i_prm[1] = pIacs758_data[((int)V_chr_RI + pIacs758->size[0]) - 1];
                i2 = b_Rwire->size[0] * b_Rwire->size[1];
                b_Rwire->size[0] = 1;
                loop_ub = Rwire->size[1];
                b_Rwire->size[1] = Rwire->size[1];
                emxEnsureCapacity_real32_T(b_Rwire, i2);
                b_Rwire_data = b_Rwire->data;
                for (i2 = 0; i2 < loop_ub; i2++) {
                  b_Rwire_data[i2] = Rwire_data[((int)V_chr_RI + Rwire->size[0] *
                    i2) - 1];
                }

                b_meanIbrd0 = ReadVI(ProjectFlag, g_prm, b_bitCnfg, N_bat1,
                                     N_bat_tmp_tmp_tmp, i_prm, Iacs758Flag,
                                     pIshunt, b_Rwire, a__36, b_I, r, a__37,
                                     a__38, a__39, a__40, VmV, a__42, tvv, a__45,
                                     (double *)&errI2C_data, errI2C_size,
                                     &showVdiff);
                IshuntTest2_data = r->data;
                IbatMat_data = b_I->data;
                loop_ub = Vbrd->size[3];
                for (i2 = 0; i2 < loop_ub; i2++) {
                  VmV_data[((b_k_tstState + Vbrd->size[0] * ((int)V_chr_RI - 1))
                            + Vbrd->size[0] * Vbrd->size[1] * Vbrd->size[2] * i2)
                    + Vbrd->size[0] * Vbrd->size[1] * Vbrd->size[2] * Vbrd->
                    size[3] * b_end] = IbatMat_data[i2];
                }

                loop_ub = Ibrd->size[3];
                for (i2 = 0; i2 < loop_ub; i2++) {
                  pIshunt_data[((b_k_tstState + Ibrd->size[0] * ((int)V_chr_RI -
                    1)) + Ibrd->size[0] * Ibrd->size[1] * Ibrd->size[2] * i2) +
                    Ibrd->size[0] * Ibrd->size[1] * Ibrd->size[2] * Ibrd->size[3]
                    * b_end] = IshuntTest2_data[i2];
                }

                /*  read V,I */
                break;

               case 3:
                /* esp32 ser */
                /*  [VbrdChr(k_bitCnfg,k_ina219,k_bitIchr,:),IbrdChr(k_bitCnfg,k_ina219,k_bitIchr,:),~,~,errI2C,~] = readI2cVIfastTicTocV5_Rratio_ser(Esp32_v1(k_ina219),true,true,prm.brd.pac.Rshunt(k_ina219),VIpacId(k_ina219,:),Pac2Vid0,squeeze(polyFitPacV(k_ina219,:,:)),squeeze(Rval(k_ina219,:,:)),N_bat1,N_bat2); */
                for (i2 = 0; i2 < 16; i2++) {
                  g_prm[i2] = prm->brd.pac.VIpacId[((int)V_chr_RI + (i2 << 1)) -
                    1];
                }

                i2 = b_bitCnfg->size[0] * b_bitCnfg->size[1] * b_bitCnfg->size[2];
                b_bitCnfg->size[0] = 1;
                b_bitCnfg->size[1] = Pac2Vid0All->size[1];
                loop_ub = Pac2Vid0All->size[2];
                b_bitCnfg->size[2] = Pac2Vid0All->size[2];
                emxEnsureCapacity_uint8_T(b_bitCnfg, i2);
                Pac2Vid31_data = b_bitCnfg->data;
                for (i2 = 0; i2 < loop_ub; i2++) {
                  b_loop_ub = Pac2Vid0All->size[1];
                  for (i3 = 0; i3 < b_loop_ub; i3++) {
                    Pac2Vid31_data[i3 + b_bitCnfg->size[1] * i2] =
                      Pac2Vid0All_data[(((int)V_chr_RI + Pac2Vid0All->size[0] *
                                         i3) + Pac2Vid0All->size[0] *
                                        Pac2Vid0All->size[1] * i2) - 1];
                  }
                }

                i_prm[0] = pIacs758_data[(int)V_chr_RI - 1];
                i_prm[1] = pIacs758_data[((int)V_chr_RI + pIacs758->size[0]) - 1];
                i2 = b_Rwire->size[0] * b_Rwire->size[1];
                b_Rwire->size[0] = 1;
                loop_ub = Rwire->size[1];
                b_Rwire->size[1] = Rwire->size[1];
                emxEnsureCapacity_real32_T(b_Rwire, i2);
                b_Rwire_data = b_Rwire->data;
                for (i2 = 0; i2 < loop_ub; i2++) {
                  b_Rwire_data[i2] = Rwire_data[((int)V_chr_RI + Rwire->size[0] *
                    i2) - 1];
                }

                b_meanIbrd0 = ReadVI(ProjectFlag, g_prm, b_bitCnfg, N_bat1,
                                     N_bat_tmp_tmp_tmp, i_prm, Iacs758Flag,
                                     pIshunt, b_Rwire, a__36, b_I, r, a__37,
                                     a__38, a__39, a__40, VmV, a__42, tvv, a__45,
                                     (double *)&errI2C_data, errI2C_size,
                                     &showVdiff);
                IshuntTest2_data = r->data;
                IbatMat_data = b_I->data;
                loop_ub = Vbrd->size[3];
                for (i2 = 0; i2 < loop_ub; i2++) {
                  VmV_data[((b_k_tstState + Vbrd->size[0] * ((int)V_chr_RI - 1))
                            + Vbrd->size[0] * Vbrd->size[1] * Vbrd->size[2] * i2)
                    + Vbrd->size[0] * Vbrd->size[1] * Vbrd->size[2] * Vbrd->
                    size[3] * b_end] = IbatMat_data[i2];
                }

                loop_ub = Ibrd->size[3];
                for (i2 = 0; i2 < loop_ub; i2++) {
                  pIshunt_data[((b_k_tstState + Ibrd->size[0] * ((int)V_chr_RI -
                    1)) + Ibrd->size[0] * Ibrd->size[1] * Ibrd->size[2] * i2) +
                    Ibrd->size[0] * Ibrd->size[1] * Ibrd->size[2] * Ibrd->size[3]
                    * b_end] = IshuntTest2_data[i2];
                }

                /*  read V,I */
                break;
              }

              switch (prm->seq.bit.meas.V[k_groups + (b_end << 3)]) {
               case 1:
                /* kunkin kp184 */
                pause(0.1);
                ControlKp184((double *)&a__3_data, a__3_size, (double *)
                             &b_meanIbrd0, a__4_size, (double *)&errI2C_data,
                             errI2C_size, (double *)&a__16_data, a__16_size);
                VbusTest2_data[(b_k_tstState + VbitInsMeas->size[0] * ((int)
                  V_chr_RI - 1)) + VbitInsMeas->size[0] * VbitInsMeas->size[1] *
                  VbitInsMeas->size[2] * b_end] = (float)errI2C_data -
                  prm->seq.bit.meas.Vd[k_groups + (b_end << 3)];
                break;

               case 2:
                /* ka6005p */
                /* read I */
                d = d_rand();
                VbusTest2_data[(b_k_tstState + VbitInsMeas->size[0] * ((int)
                  V_chr_RI - 1)) + VbitInsMeas->size[0] * VbitInsMeas->size[1] *
                  VbitInsMeas->size[2] * b_end] = (float)(d * 2.5 + 2.0) -
                  prm->seq.bit.meas.Vd[k_groups + (b_end << 3)];
                break;

               case 3:
                /* juntek+ACDC */
                pause(0.1);
                b_controlJuntekDPH8920(outVI, c_tmp_size);
                loop_ub = c_tmp_size[0] * c_tmp_size[1];
                c_vSumMax = prm->seq.bit.meas.Vd[k_groups + (b_end << 3)];
                for (i2 = 0; i2 < loop_ub; i2++) {
                  outVI[i2] = (float)outVI[i2] - c_vSumMax;
                }

                VbusTest2_data[(b_k_tstState + VbitInsMeas->size[0] * ((int)
                  V_chr_RI - 1)) + VbitInsMeas->size[0] * VbitInsMeas->size[1] *
                  VbitInsMeas->size[2] * b_end] = outVI[0];
                break;
              }

              switch (prm->seq.bit.meas.b_I[k_groups + (b_end << 3)]) {
               case 1:
                /* kunkin kp184 */
                pause(0.1);
                ControlKp184((double *)&a__3_data, a__3_size, (double *)
                             &b_meanIbrd0, a__4_size, (double *)&errI2C_data,
                             errI2C_size, (double *)&N0, tmp_size);
                break;

               case 2:
                /* ka6005p */
                /* read I */
                d_rand();
                break;

               case 3:
                /* juntek+ACDC */
                controlJuntekDPH8920(r31.data, r31.size);
                break;
              }

              switch (ProjectFlag) {
               case 2:
                /* esp32 */
                break;

               case 3:
                /* esp32 ser */
                /* UNTITLED Summary of this function goes here */
                /*    Detailed explanation goes here */
                /*  if coder.target('MATLAB') */
                /*  read current */
                /*   chip1     |   chip2     |   chip3 */
                /* %comN2 Port4 chip1,comN1 Port11 chip1 */
                /* %comP1 Port18 chip1 ,comP2 Port18 chip3 */
                /*  */
                /*  for k_row = 1:N_row */
                /*      out123(k_row,:) = readSPI(SPI,PortSpiRow(k_row,1),N_chip); */
                /*      oldPortSpiRow(k_row,1:2:end) = PortSpiRow(k_row,1:2:end); */
                /*      oldPortSpiRow(k_row,2:2:end) = out123(k_row,2:2:end); */
                /*  end */
                /* mask current */
                /* write current mask */
                /*  for k_row = 1:N_row */
                /*      write(SPI,oldPortSpiRowMask(k_row,:)); */
                /*  end */
                pause(0.001);

                /* mask new */
                /* write mask new */
                /*  for k_row = 1:N_row */
                /*      write(SPI0,PortSpiRowMask(k_row,:)); */
                /*  end */
                /* write new */
                /*  for k_row = 1:N_row */
                /*      write(SPI0,PortSpiRow(k_row,:)); */
                /*  end */
                /*  Rmsg = SPI_writeread(SPI0, PortSpiRow); */
                /*  ReadPortSpiRow = RmsgData([[2:4] , 1] ,:); */
                /*  ReadPortSpiRow(:,1:2:end) = ReadPortSpiRow(:,1:2:end);% - 128; */
                /* mask current */
                /* %chip1 Port6 Alert1, chip1 port 7 Alert3, chip1 port 8 Alert8, chip1 port 9 Alert6 */
                /* %chip1 Port12 Alert7, chip1 Port13 Alert5, chip 1 port19 NC */
                /* %chip1 Port20-23 NC */
                /* chip1 Port28 Alert1, chip1 Port29 Alert2 */
                /*  if sum(sum(sum(abs(double(PortSpiRow)-double(ReadPortSpiRow)))))>0 */
                /*  end */
                break;
              }

              i2 = b_bitCnfg->size[0] * b_bitCnfg->size[1] * b_bitCnfg->size[2];
              b_bitCnfg->size[0] = 1;
              b_bitCnfg->size[1] = Pac2Vid0All->size[1];
              loop_ub = Pac2Vid0All->size[2];
              b_bitCnfg->size[2] = Pac2Vid0All->size[2];
              emxEnsureCapacity_uint8_T(b_bitCnfg, i2);
              Pac2Vid31_data = b_bitCnfg->data;
              for (i2 = 0; i2 < loop_ub; i2++) {
                b_loop_ub = Pac2Vid0All->size[1];
                for (i3 = 0; i3 < b_loop_ub; i3++) {
                  Pac2Vid31_data[i3 + b_bitCnfg->size[1] * i2] =
                    Pac2Vid0All_data[(((int)V_chr_RI + Pac2Vid0All->size[0] * i3)
                                      + Pac2Vid0All->size[0] * Pac2Vid0All->
                                      size[1] * i2) - 1];
                }
              }

              c_squeeze(b_bitCnfg, Pac2Vid31);
              Pac2Vid31_data = Pac2Vid31->data;
              VdebugVec_size_idx_0_tmp = Pac2Vid31->size[0] * Pac2Vid31->size[1];
              end = VdebugVec_size_idx_0_tmp - 1;
              state_k = 0;
              for (b_i = 0; b_i <= end; b_i++) {
                if (Pac2Vid31_data[b_i] > 0) {
                  state_k++;
                }
              }

              i2 = r3->size[0];
              r3->size[0] = state_k;
              emxEnsureCapacity_int32_T(r3, i2);
              r32 = r3->data;
              loop_ub = 0;
              for (b_i = 0; b_i <= end; b_i++) {
                if (Pac2Vid31_data[b_i] > 0) {
                  r32[loop_ub] = b_i;
                  loop_ub++;
                }
              }

              i2 = x->size[0] * x->size[1] * x->size[2] * x->size[3];
              x->size[0] = 1;
              x->size[1] = 1;
              x->size[2] = 1;
              loop_ub = r3->size[0];
              x->size[3] = r3->size[0];
              emxEnsureCapacity_real_T(x, i2);
              Vdebug_data = x->data;
              for (i2 = 0; i2 < loop_ub; i2++) {
                Vdebug_data[i2] = pIshunt_data[((b_k_tstState + Ibrd->size[0] *
                  ((int)V_chr_RI - 1)) + Ibrd->size[0] * Ibrd->size[1] *
                  Ibrd->size[2] * (Pac2Vid31_data[r32[i2]] - 1)) + Ibrd->size[0]
                  * Ibrd->size[1] * Ibrd->size[2] * Ibrd->size[3] * b_end];
              }

              i2 = r3->size[0];
              RtotHelpPoly_data[(b_k_tstState + meanIbrd->size[0] * ((int)
                V_chr_RI - 1)) + meanIbrd->size[0] * meanIbrd->size[1] *
                meanIbrd->size[2] * b_end] = b_combineVectorElements(x) /
                (double)r3->size[0];
              b_meanIbrd0 = RtotHelpPoly_data[(b_k_tstState + meanIbrd->size[0] *
                ((int)V_chr_RI - 1)) + meanIbrd->size[0] * meanIbrd->size[1] *
                meanIbrd->size[2] * b_end];
              i3 = k_groups + (b_end << 3);
              if (prm->seq.bit.chr[i3] != 0) {
                b_meanIbrd0 = -b_meanIbrd0;
              }

              showVdiff = true;
              f = prm->seq.bit.IdisChr[i3];
              if ((float)fabs((float)b_meanIbrd0 - f) > prm->seq.bit.dIthr) {
                d_sprintf(b_meanIbrd0, meanIbrd0);
                k_sprintf(f, r15);
                k_sprintf(f, r16);
                if (V_chr_RI < 2.147483648E+9) {
                  i3 = (int)V_chr_RI;
                } else {
                  i3 = MAX_int32_T;
                }

                b_sprintf(i3, r17);
                for (id2Bypass_size = 0; id2Bypass_size < i2; id2Bypass_size++)
                {
                  printf("%d\n", (int)Pac2Vid31_data[r32[id2Bypass_size]]);
                  fflush(stdout);
                }

                showVdiff = false;
              }

              i2 = x->size[0] * x->size[1] * x->size[2] * x->size[3];
              x->size[0] = 1;
              x->size[1] = 1;
              x->size[2] = 1;
              loop_ub = r3->size[0];
              x->size[3] = r3->size[0];
              emxEnsureCapacity_real_T(x, i2);
              Vdebug_data = x->data;
              for (i2 = 0; i2 < loop_ub; i2++) {
                Vdebug_data[i2] = VmV_data[((b_k_tstState + Vbrd->size[0] *
                  ((int)V_chr_RI - 1)) + Vbrd->size[0] * Vbrd->size[1] *
                  Vbrd->size[2] * (Pac2Vid31_data[r32[i2]] - 1)) + Vbrd->size[0]
                  * Vbrd->size[1] * Vbrd->size[2] * Vbrd->size[3] * b_end];
              }

              b_meanIbrd0 = b_combineVectorElements(x) - VbusTest2_data
                [(b_k_tstState + VbitInsMeas->size[0] * ((int)V_chr_RI - 1)) +
                VbitInsMeas->size[0] * VbitInsMeas->size[1] * VbitInsMeas->size
                [2] * b_end];
              if ((fabs(b_meanIbrd0) > prm->seq.bit.dVthr) && showVdiff) {
                d_sprintf(b_meanIbrd0, dv0);
                if (V_chr_RI < 2.147483648E+9) {
                  i2 = (int)V_chr_RI;
                } else {
                  i2 = MAX_int32_T;
                }

                b_sprintf(i2, r18);
                i2 = r3->size[0];
                for (id2Bypass_size = 0; id2Bypass_size < i2; id2Bypass_size++)
                {
                  printf("%d\n", (int)Pac2Vid31_data[r32[id2Bypass_size]]);
                  fflush(stdout);
                }
              }

              /*  for k_bitI = 1:N_bitI */
            }

            /*  for k_ina219 = VecIna219_k */
          }

          /*  for k_groups = 1:length(uGroups) */
        }

        /*  for k_bitState = 1:NstBit */
      }

      /*  for k_bitCnfg = 1:N_bitCnfg */
    }

    /*  if prm.bit.bit_flag */
    /*  End - Bit */
    /*  Run parameters */
    /*  = 1;%[V] */
    /*      = 2e4; */
    N_bitCnfg = prm->run.Nt0;

    /*      = 2e4; */
    /*  SelVsT = zeros(Nt0,1); */
    /*  SelDualVsT = zeros(Nina219,Nt0); */
    i = (int)(prm->brd.Nina219 * (double)prm->brd.N_bat);
    i1 = Rtot->size[0] * Rtot->size[1];
    Rtot->size[0] = i;
    Rtot->size[1] = prm->run.Nt0;
    emxEnsureCapacity_real_T(Rtot, i1);
    Rtot_data = Rtot->data;
    VdebugVec_size_idx_0_tmp = i * prm->run.Nt0;
    for (i1 = 0; i1 < VdebugVec_size_idx_0_tmp; i1++) {
      Rtot_data[i1] = 0.0;
    }

    i1 = Vbat->size[0] * Vbat->size[1];
    Vbat->size[0] = Rtot->size[0];
    Vbat->size[1] = Rtot->size[1];
    emxEnsureCapacity_real_T(Vbat, i1);
    RtotHelpPoly_data = Vbat->data;
    for (i1 = 0; i1 < VdebugVec_size_idx_0_tmp; i1++) {
      RtotHelpPoly_data[i1] = 0.0;
    }

    i1 = IbatMat->size[0] * IbatMat->size[1] * IbatMat->size[2];
    IbatMat->size[0] = (int)prm->brd.Nina219;
    IbatMat->size[1] = prm->brd.N_bat;
    IbatMat->size[2] = prm->run.Nt0;
    emxEnsureCapacity_real_T(IbatMat, i1);
    IbatMat_data = IbatMat->data;
    k_t0 *= prm->run.Nt0;
    for (i1 = 0; i1 < k_t0; i1++) {
      IbatMat_data[i1] = 0.0;
    }

    i1 = VbatMat->size[0] * VbatMat->size[1] * VbatMat->size[2];
    VbatMat->size[0] = (int)prm->brd.Nina219;
    VbatMat->size[1] = prm->brd.N_bat;
    VbatMat->size[2] = prm->run.Nt0;
    emxEnsureCapacity_real_T(VbatMat, i1);
    VbusTest2_data = VbatMat->data;
    for (i1 = 0; i1 < k_t0; i1++) {
      VbusTest2_data[i1] = 0.0;
    }

    i1 = VbatMat0->size[0] * VbatMat0->size[1] * VbatMat0->size[2];
    VbatMat0->size[0] = (int)prm->brd.Nina219;
    VbatMat0->size[1] = prm->brd.N_bat;
    VbatMat0->size[2] = prm->run.Nt0;
    emxEnsureCapacity_real_T(VbatMat0, i1);
    ImKp184Test2_data = VbatMat0->data;
    for (i1 = 0; i1 < k_t0; i1++) {
      ImKp184Test2_data[i1] = 0.0;
    }

    i1 = Rtot->size[0] * Rtot->size[1];
    Rtot->size[0] = i;
    Rtot->size[1] = prm->run.Nt0;
    emxEnsureCapacity_real_T(Rtot, i1);
    Rtot_data = Rtot->data;
    for (i = 0; i < VdebugVec_size_idx_0_tmp; i++) {
      Rtot_data[i] = 0.0;
    }

    i = tV1->size[0] * tV1->size[1];
    tV1->size[0] = Rtot->size[0];
    tV1->size[1] = Rtot->size[1];
    emxEnsureCapacity_real_T(tV1, i);
    Iacs758_cal_data = tV1->data;
    for (i = 0; i < VdebugVec_size_idx_0_tmp; i++) {
      Iacs758_cal_data[i] = 0.0;
    }

    i = tV->size[0] * tV->size[1] * tV->size[2];
    tV->size[0] = (int)prm->brd.Nina219;
    tV->size[1] = prm->brd.N_bat;
    tV->size[2] = prm->run.Nt0;
    emxEnsureCapacity_real_T(tV, i);
    tV_data = tV->data;
    for (i = 0; i < k_t0; i++) {
      tV_data[i] = 0.0;
    }

    /*  Ishunt = zeros(Nina219,N_bat,Nt); */
    /*  coder.varsize('Pac2Vid0All',[16 32 32],[1 1 1]); */
    i = Pac2Vid0All->size[0] * Pac2Vid0All->size[1] * Pac2Vid0All->size[2];
    Pac2Vid0All->size[0] = (int)prm->brd.Nina219;
    Pac2Vid0All->size[1] = prm->brd.N_bat;
    Pac2Vid0All->size[2] = prm->brd.N_bat;
    emxEnsureCapacity_uint8_T(Pac2Vid0All, i);
    Pac2Vid0All_data = Pac2Vid0All->data;
    for (i = 0; i < c_k_Ttest; i++) {
      Pac2Vid0All_data[i] = 0U;
    }

    /*  RegStateMat = zeros(1,Nt0); */
    /*  strabcd = char(uint32('A')+uint32([0:N_bat-1]')); */
    /*  ina219State  = 0; */
    /*  kp184State   = 0; */
    /*  i_in = repmat(5,1,Nt);%[A] */
    /*  t = [0:50e3]; */
    /*  i = 6*abs(sin(2*pi/500*t)); */
    /*  if true */
    /*      bl = fir1(34,0.02,'low',chebwin(35,30)); */
    /*      %figure;freqz(bhi,1) */
    /*      i_in = filter(bl,1,i);%i_in_read(1:1370,2)); */
    /*  else */
    /*      i_in = i; */
    /*  end */
    /*  if juntek_Flag */
    /*      i_in = ImaxAcDC*ones(size(i_in)); */
    /*  end */
    /*  i_in = round(i_in*1000)/1000; */
    /*  */
    /*  if juntekOnlyChr_Flag */
    /*      i_in = ImaxAcDC*ones(size(i_in)); */
    /*  end */
    ImaxDis = prm->bat.ImaxDis;

    /* 15;%35;%maximum current from discharge bat */
    Icharge = prm->bat.Icharge;

    /* 5;%15;%;25;%5;%[A] */
    IchargePhase2_tmp_tmp = prm->bat.IchargePhase2;

    /* 2.5;%;15;%2.5;%2.5;%2.5;%[A] */
    IchargePhase2t = prm->bat.IchargePhase2;

    /* prm.bat.IchargePhase2t;%IchargePhase2; */
    IchargePhase3 = prm->bat.IchargePhase3;

    /* 2;%0.5;%0.3;%0.2;%[A] */
    /* 2;%4;%7;%0.4;%6.5*0.03; */
    /* 0.5;%3;%0.2; */
    /* 2.8;%2.7;%3.0;%2.8;%[V] */
    /* 4.1;%3.6;%4.1;%25;%4.1;%[V] */
    /*  seq            = prm.seq(prm.run.seq); */
    i = SelDual->size[0] * SelDual->size[1];
    SelDual->size[0] = 1;
    SelDual->size[1] = (int)prm->brd.Nina219;
    emxEnsureCapacity_real_T(SelDual, i);
    SelDual_data = SelDual->data;
    for (i = 0; i < b_loop_ub_tmp; i++) {
      SelDual_data[i] = 0.0;
    }

    i = SelDualHelp->size[0] * SelDualHelp->size[1];
    SelDualHelp->size[0] = 1;
    SelDualHelp->size[1] = (int)prm->brd.Nina219;
    emxEnsureCapacity_real_T(SelDualHelp, i);
    SelDualHelp_data = SelDualHelp->data;
    for (i = 0; i < b_loop_ub_tmp; i++) {
      SelDualHelp_data[i] = 0.0;
    }

    for (id2Bypass_size = 0; id2Bypass_size < b_size; id2Bypass_size++) {
      for (b_i = 0; b_i <= end_tmp; b_i++) {
        if (b_data[id2Bypass_size] == prm->ser.com.grp[tmp_data[b_i]]) {
          SelDual_data[b_i] = prm->seq.chr[id2Bypass_size];
        }
      }
    }

    /* display V & I on command window */
    /* %%%%%%% read voltage and current %%%%%%%%%%% */
    i = tLastToggle->size[0] * tLastToggle->size[1];
    tLastToggle->size[0] = 1;
    tLastToggle->size[1] = (int)prm->brd.Nina219;
    emxEnsureCapacity_real_T(tLastToggle, i);
    tLastToggle_data = tLastToggle->data;
    for (i = 0; i < b_loop_ub_tmp; i++) {
      tLastToggle_data[i] = 0.0;
    }

    k_t = 1.0;
    k_t0 = 0;
    i = switchFlag->size[0] * switchFlag->size[1];
    switchFlag->size[0] = 1;
    switchFlag->size[1] = (int)prm->brd.Nina219;
    emxEnsureCapacity_boolean_T(switchFlag, i);
    switchFlag_data = switchFlag->data;
    for (i = 0; i < b_loop_ub_tmp; i++) {
      switchFlag_data[i] = true;
    }

    i = switchFlagHelp->size[0] * switchFlagHelp->size[1];
    switchFlagHelp->size[0] = 1;
    switchFlagHelp->size[1] = (int)prm->brd.Nina219;
    emxEnsureCapacity_boolean_T(switchFlagHelp, i);
    switchFlagHelp_data = switchFlagHelp->data;
    for (i = 0; i < b_loop_ub_tmp; i++) {
      switchFlagHelp_data[i] = false;
    }

    i = changeConfigFlag->size[0] * changeConfigFlag->size[1];
    changeConfigFlag->size[0] = 1;
    changeConfigFlag->size[1] = (int)prm->brd.Nina219;
    emxEnsureCapacity_boolean_T(changeConfigFlag, i);
    changeConfigFlag_data = changeConfigFlag->data;
    for (i = 0; i < b_loop_ub_tmp; i++) {
      changeConfigFlag_data[i] = false;
    }

    tic();
    keepMeas = true;
    toc();
    onPowerFlag = false;
    state_k = 0;
    g_IchargeAct = 0.0;
    if (prm->seq.Nst == 0) {
      if (prm->brd.Nina219 < 1.0) {
        loop_ub_tmp = 0;
      } else {
        loop_ub_tmp = (int)prm->brd.Nina219;
      }

      i = outStruct->VbusTest->size[0] * outStruct->VbusTest->size[1];
      outStruct->VbusTest->size[0] = loop_ub_tmp;
      outStruct->VbusTest->size[1] = prm->brd.N_bat;
      emxEnsureCapacity_real_T(outStruct->VbusTest, i);
      for (i = 0; i < N_bat_tmp_tmp_tmp; i++) {
        for (i1 = 0; i1 < loop_ub_tmp; i1++) {
          outStruct->VbusTest->data[i1 + outStruct->VbusTest->size[0] * i] =
            VbusTest_data[i1 + (int)Nina219_tmp_tmp * i];
        }
      }

      i = outStruct->VmKp184Test->size[0] * outStruct->VmKp184Test->size[1];
      outStruct->VmKp184Test->size[0] = loop_ub_tmp;
      outStruct->VmKp184Test->size[1] = VmKp184Test_size_idx_1;
      emxEnsureCapacity_real_T(outStruct->VmKp184Test, i);
      for (i = 0; i < VmKp184Test_size_idx_1; i++) {
        for (i1 = 0; i1 < loop_ub_tmp; i1++) {
          outStruct->VmKp184Test->data[i1 + outStruct->VmKp184Test->size[0] * i]
            = VmKp184Test_data[i1 + VmKp184Test_size_idx_0 * i];
        }
      }
    } else {
      /* Init Plots */
      i = BattConfigPerIna->size[0] * BattConfigPerIna->size[1] *
        BattConfigPerIna->size[2];
      BattConfigPerIna->size[0] = (int)prm->brd.Nina219;
      BattConfigPerIna->size[1] = prm->brd.N_bat;
      BattConfigPerIna->size[2] = prm->brd.N_bat;
      emxEnsureCapacity_real_T(BattConfigPerIna, i);
      BattConfigPerIna_data = BattConfigPerIna->data;
      for (i = 0; i < c_k_Ttest; i++) {
        BattConfigPerIna_data[i] = 0.0;
      }

      /* nadav coder */
      if ((prm->seq.pwr.VthFlag[0] == 1) || (prm->seq.pwr.VthFlag[0] == 2)) {
        b_ReadVI(prm->brd.Nina219, prm->ins.ProjectFlag, prm->brd.pac.VIpacId,
                 Pac2Vid0All, prm->brd.N_bat1, prm->brd.N_bat, pIacs758,
                 prm->brd.pac.Iacs758Flag, pIshunt, Rwire, a__64, a__65, a__66,
                 a__67, Rtot, a__68, a__69, a__70, a__71, a__72, a__73,
                 RtotHelpPoly, (double *)&errI2C_data, errI2C_size);
        Rtot_data = Rtot->data;

        /*  read V,I */
        if ((int)prm->brd.Nina219 - 1 >= 0) {
          if (prm->brd.N_bat < 1) {
            y->size[0] = 1;
            y->size[1] = 0;
          } else {
            i = y->size[0] * y->size[1];
            y->size[0] = 1;
            y->size[1] = prm->brd.N_bat;
            emxEnsureCapacity_uint16_T(y, i);
            y_data = y->data;
            loop_ub = prm->brd.N_bat - 1;
            for (i = 0; i <= loop_ub; i++) {
              y_data[i] = (unsigned short)((unsigned int)i + 1U);
            }
          }

          c_loop_ub = y->size[1];
        }

        for (b_k_tstState = 0; b_k_tstState < b_loop_ub_tmp; b_k_tstState++) {
          b_meanIbrd0 = (((double)b_k_tstState + 1.0) - 1.0) * (double)
            N_bat_tmp_tmp_tmp;
          id2Bypass_size = y->size[1];
          for (i = 0; i < c_loop_ub; i++) {
            j_tmp_data[i] = (short)((short)(b_meanIbrd0 + (double)y_data[i]) - 1);
          }

          for (i = 0; i < id2Bypass_size; i++) {
            RtotHelpPoly_data[j_tmp_data[i]] = Rtot_data[b_k_tstState +
              Rtot->size[0] * i];
          }

          i = (int)SelDual_data[b_k_tstState];
          if (i == 0) {
            d_padArrUint8(prm->cnfg.BattConfigDis1, N_bat_tmp_tmp_tmp,
                          N_bat_tmp_tmp_tmp, Pac2Vid31);
            Pac2Vid31_data = Pac2Vid31->data;
            i = r4->size[0] * r4->size[1];
            r4->size[0] = Pac2Vid31->size[0];
            r4->size[1] = Pac2Vid31->size[1];
            emxEnsureCapacity_real_T(r4, i);
            IshuntTest2_data = r4->data;
            loop_ub = Pac2Vid31->size[0] * Pac2Vid31->size[1];
            for (i = 0; i < loop_ub; i++) {
              IshuntTest2_data[i] = Pac2Vid31_data[i];
            }

            k_ina219_ = BattConfigPerIna->size[1];
            loop_ub = BattConfigPerIna->size[2];
            for (i = 0; i < loop_ub; i++) {
              for (i1 = 0; i1 < k_ina219_; i1++) {
                BattConfigPerIna_data[(b_k_tstState + BattConfigPerIna->size[0] *
                  i1) + BattConfigPerIna->size[0] * BattConfigPerIna->size[1] *
                  i] = IshuntTest2_data[i1 + k_ina219_ * i];
              }
            }

            /* nadav coder */
            /*  BattConfigPerIna{k_ina219} = prm.cnfg.BattConfigDis1;%BattConfigPerInaHelp{k_ina219}; */
            /*  BattConfig{k_ina219}       = BattConfigPerIna{k_ina219} + (k_ina219-1)*N_bat; */
          } else if (i == 1) {
            d_padArrUint8(prm->cnfg.BattConfigChr1, N_bat_tmp_tmp_tmp,
                          N_bat_tmp_tmp_tmp, Pac2Vid31);
            Pac2Vid31_data = Pac2Vid31->data;
            i = r4->size[0] * r4->size[1];
            r4->size[0] = Pac2Vid31->size[0];
            r4->size[1] = Pac2Vid31->size[1];
            emxEnsureCapacity_real_T(r4, i);
            IshuntTest2_data = r4->data;
            loop_ub = Pac2Vid31->size[0] * Pac2Vid31->size[1];
            for (i = 0; i < loop_ub; i++) {
              IshuntTest2_data[i] = Pac2Vid31_data[i];
            }

            k_ina219_ = BattConfigPerIna->size[1];
            loop_ub = BattConfigPerIna->size[2];
            for (i = 0; i < loop_ub; i++) {
              for (i1 = 0; i1 < k_ina219_; i1++) {
                BattConfigPerIna_data[(b_k_tstState + BattConfigPerIna->size[0] *
                  i1) + BattConfigPerIna->size[0] * BattConfigPerIna->size[1] *
                  i] = IshuntTest2_data[i1 + k_ina219_ * i];
              }
            }

            /* nadav coder */
            /*  BattConfigPerIna{k_ina219} = prm.cnfg.BattConfigChr1;%BattConfigPerInaHelp{k_ina219}; */
            /*  BattConfig{k_ina219}       = BattConfigPerIna{k_ina219} + (k_ina219-1)*N_bat; */
          }
        }
      }

      i = BattConfigPerInaHelp->size[0] * BattConfigPerInaHelp->size[1] *
        BattConfigPerInaHelp->size[2];
      BattConfigPerInaHelp->size[0] = BattConfigPerIna->size[0];
      BattConfigPerInaHelp->size[1] = BattConfigPerIna->size[1];
      BattConfigPerInaHelp->size[2] = BattConfigPerIna->size[2];
      emxEnsureCapacity_real_T(BattConfigPerInaHelp, i);
      BattConfigPerInaHelp_data = BattConfigPerInaHelp->data;
      for (i = 0; i < c_k_Ttest; i++) {
        BattConfigPerInaHelp_data[i] = BattConfigPerIna_data[i];
      }

      /*  main while */
      exitg1 = false;
      while ((!exitg1) && keepMeas) {
        /*      try */
        toc00 = toc();
        i = b_changeConfigFlag->size[0] * b_changeConfigFlag->size[1];
        b_changeConfigFlag->size[0] = 1;
        if (switchFlag->size[1] == 1) {
          loop_ub = changeConfigFlag->size[1];
        } else {
          loop_ub = switchFlag->size[1];
        }

        b_changeConfigFlag->size[1] = loop_ub;
        emxEnsureCapacity_boolean_T(b_changeConfigFlag, i);
        b_changeConfigFlag_data = b_changeConfigFlag->data;
        VdebugVec_size_idx_0_tmp = (changeConfigFlag->size[1] != 1);
        for (i = 0; i < loop_ub; i++) {
          id2Bypass_size = i * VdebugVec_size_idx_0_tmp;
          b_changeConfigFlag_data[i] = (changeConfigFlag_data[id2Bypass_size] ||
            switchFlag_data[id2Bypass_size]);
        }

        if (b_any(b_changeConfigFlag)) {
          /* Off power and load */
          pause(0.1);
          onPowerFlag = true;
          pause(0.1);
        }

        /*  if change: limit Vchr calculation + Off power and load - End */
        b_guard1 = false;
        if ((prm->seq.pwr.VthFlag[state_k] == 1) || (prm->
             seq.pwr.VthFlag[state_k] == 2)) {
          /*  BattConfigAct = BattConfigAct(:); */
          if (Nina219_tmp_tmp < 1.0) {
            VecIna219->size[0] = 1;
            VecIna219->size[1] = 0;
          } else {
            i = VecIna219->size[0] * VecIna219->size[1];
            VecIna219->size[0] = 1;
            VecIna219->size[1] = (int)(Nina219_tmp_tmp - 1.0) + 1;
            emxEnsureCapacity_real_T(VecIna219, i);
            VecIna219_data = VecIna219->data;
            loop_ub = (int)(Nina219_tmp_tmp - 1.0);
            for (i = 0; i <= loop_ub; i++) {
              VecIna219_data[i] = (double)i + 1.0;
            }
          }

          if (b_size - 1 >= 0) {
            d_end = ipos_size - 1;
          }

          for (k_groups = 0; k_groups < b_size; k_groups++) {
            trueCount = 0;
            loop_ub = 0;
            for (b_i = 0; b_i <= d_end; b_i++) {
              if (k_groups + 1 == ipos_data[b_i]) {
                trueCount++;
                k_tmp_data[loop_ub] = (signed char)b_i;
                loop_ub++;
              }
            }

            /* VecIna219_k = find(k_groups == uGroupId); */
            nextStatePerGroup_size[0] = 1;
            nextStatePerGroup_size[1] = trueCount;
            for (i = 0; i < trueCount; i++) {
              nextStatePerGroup_data[i] = switchFlag_data[(int)
                VecIna219_data[k_tmp_data[i]] - 1];
            }

            b_VbusTest0_data.data = &nextStatePerGroup_data[0];
            b_VbusTest0_data.size = &nextStatePerGroup_size[0];
            b_VbusTest0_data.allocatedSize = 2;
            b_VbusTest0_data.numDimensions = 2;
            b_VbusTest0_data.canFreeData = false;
            if (b_any(&b_VbusTest0_data)) {
              for (i = 0; i < trueCount; i++) {
                id2Bypass_data[i] = (int)VecIna219_data[k_tmp_data[i]];
              }

              loop_ub = trueCount - 1;
              for (i = 0; i <= loop_ub; i++) {
                switchFlagHelp_data[id2Bypass_data[i] - 1] = true;
              }

              if (N_bat_tmp_tmp_tmp < 1) {
                y->size[0] = 1;
                y->size[1] = 0;
              } else {
                i = y->size[0] * y->size[1];
                y->size[0] = 1;
                y->size[1] = N_bat_tmp_tmp_tmp;
                emxEnsureCapacity_uint16_T(y, i);
                y_data = y->data;
                loop_ub = N_bat_tmp_tmp_tmp - 1;
                for (i = 0; i <= loop_ub; i++) {
                  y_data[i] = (unsigned short)((unsigned int)i + 1U);
                }
              }

              i = r1->size[0] * r1->size[1] * r1->size[2];
              r1->size[0] = trueCount;
              r1->size[1] = BattConfigPerIna->size[1];
              loop_ub = BattConfigPerIna->size[2];
              r1->size[2] = BattConfigPerIna->size[2];
              emxEnsureCapacity_real_T(r1, i);
              IshuntTest2_data = r1->data;
              for (i = 0; i < loop_ub; i++) {
                b_loop_ub = BattConfigPerIna->size[1];
                for (i1 = 0; i1 < b_loop_ub; i1++) {
                  for (i2 = 0; i2 < trueCount; i2++) {
                    IshuntTest2_data[(i2 + r1->size[0] * i1) + r1->size[0] *
                      r1->size[1] * i] = BattConfigPerIna_data[(((int)
                      VecIna219_data[k_tmp_data[i2]] + BattConfigPerIna->size[0]
                      * i1) + BattConfigPerIna->size[0] * BattConfigPerIna->
                      size[1] * i) - 1];
                  }
                }
              }

              c_tmp_size[0] = 1;
              c_tmp_size[1] = trueCount;
              d_tmp_size[0] = 1;
              d_tmp_size[1] = trueCount;
              e_tmp_size[0] = BattConfigAct->size[0];
              e_tmp_size[1] = trueCount;
              for (i = 0; i < trueCount; i++) {
                i1 = (int)VecIna219_data[k_tmp_data[i]] - 1;
                outVI[i] = SelDual_data[i1];
                o_tmp_data[i] = tLastToggle_data[i1];
                loop_ub = BattConfigAct->size[0];
                for (i2 = 0; i2 < loop_ub; i2++) {
                  p_tmp_data[i2 + e_tmp_size[0] * i] = BattConfigAct_data[i2 +
                    BattConfigAct->size[0] * i1];
                }
              }

              if (trueCount == y->size[1]) {
                b_k_t0[0] = (double)(k_t0 + 1) - 1.0;
                b_k_t0[1] = 1.0;
                i = (int)b_maximum(b_k_t0);
                i1 = VmV->size[0];
                VmV->size[0] = y->size[1];
                emxEnsureCapacity_real_T(VmV, i1);
                VmV_data = VmV->data;
                loop_ub = y->size[1];
                for (i1 = 0; i1 < loop_ub; i1++) {
                  VmV_data[i1] = RtotHelpPoly_data[((int)((double)y_data[i1] +
                    (VecIna219_data[k_tmp_data[i1]] - 1.0) * (double)
                    N_bat_tmp_tmp_tmp) + Vbat->size[0] * (i - 1)) - 1];
                }

                tLastToggle_size[0] = 1;
                tLastToggle_size[1] = trueCount;
                for (i = 0; i < trueCount; i++) {
                  i_prm[i] = ((float)tLastToggle_data[(int)
                              VecIna219_data[k_tmp_data[i]] - 1] +
                              prm->cnfg.Ttoggle) + 1.0F;
                }

                i = (int)prm->seq.vth[state_k] - 1;
                showVdiff = c_Esp32StepSwitchToggleCombAll_(outVI, c_tmp_size,
                  prm->bat.CutOffChrV[i], prm->bat.CutOffDisV[i], r1, VmV,
                  trueCount, prm->brd.Nbat, prm->brd.spi.disconnect,
                  prm->brd.spi.bypass, prm->cnfg.Ttoggle, prm->cnfg.NtoggleDrop,
                  prm->cnfg.minLenIna219, prm->seq.pwr, prm->
                  seq.pwr.VthFlag[state_k], i_prm, tLastToggle_size, o_tmp_data,
                  d_tmp_size, p_tmp_data, e_tmp_size, AllBypassPerGroup_data,
                  id2Bypass_data);
                IshuntTest2_data = r1->data;
              } else {
                showVdiff = binary_expand_op_3(outVI, c_tmp_size, prm, state_k,
                  r1, Vbat, y, VecIna219, k_tmp_data, &trueCount,
                  N_bat_tmp_tmp_tmp, k_t0, tLastToggle, o_tmp_data, d_tmp_size,
                  p_tmp_data, e_tmp_size, AllBypassPerGroup_data, id2Bypass_data);
                IshuntTest2_data = r1->data;
              }

              for (i = 0; i < trueCount; i++) {
                id2Bypass_data[i] = (int)VecIna219_data[k_tmp_data[i]] - 1;
              }

              k_ina219_ = BattConfigPerInaHelp->size[1];
              loop_ub = BattConfigPerInaHelp->size[2];
              for (i = 0; i < loop_ub; i++) {
                for (i1 = 0; i1 < k_ina219_; i1++) {
                  for (i2 = 0; i2 < trueCount; i2++) {
                    BattConfigPerInaHelp_data[(id2Bypass_data[i2] +
                      BattConfigPerInaHelp->size[0] * i1) +
                      BattConfigPerInaHelp->size[0] * BattConfigPerInaHelp->
                      size[1] * i] = IshuntTest2_data[(i2 + trueCount * i1) +
                      trueCount * k_ina219_ * i];
                  }
                }
              }

              for (i = 0; i < trueCount; i++) {
                id2Bypass_data[i] = (int)VecIna219_data[k_tmp_data[i]];
              }

              for (i = 0; i < trueCount; i++) {
                SelDualHelp_data[id2Bypass_data[i] - 1] = outVI[i];
              }

              for (i = 0; i < trueCount; i++) {
                id2Bypass_data[i] = (int)VecIna219_data[k_tmp_data[i]];
              }

              loop_ub = trueCount - 1;
              for (i = 0; i <= loop_ub; i++) {
                switchFlagHelp_data[id2Bypass_data[i] - 1] = showVdiff;
              }

              for (i = 0; i < trueCount; i++) {
                id2Bypass_data[i] = (int)VecIna219_data[k_tmp_data[i]] - 1;
              }

              Nina219[0] = BattConfigAct->size[0];
              for (i = 0; i < trueCount; i++) {
                loop_ub = Nina219[0];
                for (i1 = 0; i1 < loop_ub; i1++) {
                  BattConfigAct_data[i1 + BattConfigAct->size[0] *
                    id2Bypass_data[i]] = p_tmp_data[i1 + Nina219[0] * i];
                }
              }

              /* %squeeze(VbatMat(k_ina219,:,k_t))',... */
              /*  %t to initiate toggle */
            }

            /*                  for k_inaPerGroup = 1:length(VecIna219_k) */
            /*                      if BattConfigPerIna{VecIna219_k(k_inaPerGroup)}~=prm.brd.spi.bypass|BattConfigPerIna{VecIna219_k(k_inaPerGroup)}~=prm.brd.spi.disconnect */
            /*                          BattConfig{VecIna219_k(k_inaPerGroup)} = BattConfigPerIna{VecIna219_k(k_inaPerGroup)} + (VecIna219_k(k_inaPerGroup)-1)*N_bat; */
            /*                      else */
            /*                          BattConfig{VecIna219_k(k_inaPerGroup)} = BattConfigPerIna{VecIna219_k(k_inaPerGroup)}; */
            /*                      end */
            /*                  end */
            /*              end */
          }

          if (b_isequal(SelDualHelp, SelDual)) {
            i = b_x->size[0] * b_x->size[1];
            b_x->size[0] = 1;
            b_x->size[1] = switchFlagHelp->size[1];
            emxEnsureCapacity_boolean_T(b_x, i);
            b_changeConfigFlag_data = b_x->data;
            loop_ub = switchFlagHelp->size[1];
            for (i = 0; i < loop_ub; i++) {
              b_changeConfigFlag_data[i] = !switchFlagHelp_data[i];
            }

            showVdiff = true;
            VdebugVec_size_idx_0_tmp = 1;
            exitg2 = false;
            while ((!exitg2) && (VdebugVec_size_idx_0_tmp <= b_x->size[1])) {
              if (!b_changeConfigFlag_data[VdebugVec_size_idx_0_tmp - 1]) {
                showVdiff = false;
                exitg2 = true;
              } else {
                VdebugVec_size_idx_0_tmp++;
              }
            }

            if (showVdiff) {
              for (b_k_tstState = 0; b_k_tstState < VbusTest_size_idx_0;
                   b_k_tstState++) {
                /* Start new mode */
                if (switchFlag_data[b_k_tstState]) {
                  /* nadav Coder */
                  d = SelDual_data[b_k_tstState];
                  if (d == 0.0) {
                    i = b_BattConfigPerInaHelp->size[0] *
                      b_BattConfigPerInaHelp->size[1] *
                      b_BattConfigPerInaHelp->size[2];
                    b_BattConfigPerInaHelp->size[0] = 1;
                    b_BattConfigPerInaHelp->size[1] = BattConfigPerInaHelp->
                      size[1];
                    loop_ub = BattConfigPerInaHelp->size[2];
                    b_BattConfigPerInaHelp->size[2] = BattConfigPerInaHelp->
                      size[2];
                    emxEnsureCapacity_real_T(b_BattConfigPerInaHelp, i);
                    Vdebug_data = b_BattConfigPerInaHelp->data;
                    for (i = 0; i < loop_ub; i++) {
                      b_loop_ub = BattConfigPerInaHelp->size[1];
                      for (i1 = 0; i1 < b_loop_ub; i1++) {
                        Vdebug_data[i1 + b_BattConfigPerInaHelp->size[1] * i] =
                          BattConfigPerInaHelp_data[(b_k_tstState +
                          BattConfigPerInaHelp->size[0] * i1) +
                          BattConfigPerInaHelp->size[0] *
                          BattConfigPerInaHelp->size[1] * i];
                      }
                    }

                    k_ina219_ = BattConfigPerIna->size[1];
                    loop_ub = BattConfigPerIna->size[2];
                    for (i = 0; i < loop_ub; i++) {
                      for (i1 = 0; i1 < k_ina219_; i1++) {
                        BattConfigPerIna_data[(b_k_tstState +
                          BattConfigPerIna->size[0] * i1) +
                          BattConfigPerIna->size[0] * BattConfigPerIna->size[1] *
                          i] = Vdebug_data[i1 + k_ina219_ * i];
                      }
                    }
                  } else if (d == 1.0) {
                    i = b_BattConfigPerInaHelp->size[0] *
                      b_BattConfigPerInaHelp->size[1] *
                      b_BattConfigPerInaHelp->size[2];
                    b_BattConfigPerInaHelp->size[0] = 1;
                    b_BattConfigPerInaHelp->size[1] = BattConfigPerInaHelp->
                      size[1];
                    loop_ub = BattConfigPerInaHelp->size[2];
                    b_BattConfigPerInaHelp->size[2] = BattConfigPerInaHelp->
                      size[2];
                    emxEnsureCapacity_real_T(b_BattConfigPerInaHelp, i);
                    Vdebug_data = b_BattConfigPerInaHelp->data;
                    for (i = 0; i < loop_ub; i++) {
                      b_loop_ub = BattConfigPerInaHelp->size[1];
                      for (i1 = 0; i1 < b_loop_ub; i1++) {
                        Vdebug_data[i1 + b_BattConfigPerInaHelp->size[1] * i] =
                          BattConfigPerInaHelp_data[(b_k_tstState +
                          BattConfigPerInaHelp->size[0] * i1) +
                          BattConfigPerInaHelp->size[0] *
                          BattConfigPerInaHelp->size[1] * i];
                      }
                    }

                    k_ina219_ = BattConfigPerIna->size[1];
                    loop_ub = BattConfigPerIna->size[2];
                    for (i = 0; i < loop_ub; i++) {
                      for (i1 = 0; i1 < k_ina219_; i1++) {
                        BattConfigPerIna_data[(b_k_tstState +
                          BattConfigPerIna->size[0] * i1) +
                          BattConfigPerIna->size[0] * BattConfigPerIna->size[1] *
                          i] = Vdebug_data[i1 + k_ina219_ * i];
                      }
                    }
                  } else if (d == 1.5) {
                    i = b_BattConfigPerInaHelp->size[0] *
                      b_BattConfigPerInaHelp->size[1] *
                      b_BattConfigPerInaHelp->size[2];
                    b_BattConfigPerInaHelp->size[0] = 1;
                    b_BattConfigPerInaHelp->size[1] = BattConfigPerInaHelp->
                      size[1];
                    loop_ub = BattConfigPerInaHelp->size[2];
                    b_BattConfigPerInaHelp->size[2] = BattConfigPerInaHelp->
                      size[2];
                    emxEnsureCapacity_real_T(b_BattConfigPerInaHelp, i);
                    Vdebug_data = b_BattConfigPerInaHelp->data;
                    for (i = 0; i < loop_ub; i++) {
                      b_loop_ub = BattConfigPerInaHelp->size[1];
                      for (i1 = 0; i1 < b_loop_ub; i1++) {
                        Vdebug_data[i1 + b_BattConfigPerInaHelp->size[1] * i] =
                          BattConfigPerInaHelp_data[(b_k_tstState +
                          BattConfigPerInaHelp->size[0] * i1) +
                          BattConfigPerInaHelp->size[0] *
                          BattConfigPerInaHelp->size[1] * i];
                      }
                    }

                    k_ina219_ = BattConfigPerIna->size[1];
                    loop_ub = BattConfigPerIna->size[2];
                    for (i = 0; i < loop_ub; i++) {
                      for (i1 = 0; i1 < k_ina219_; i1++) {
                        BattConfigPerIna_data[(b_k_tstState +
                          BattConfigPerIna->size[0] * i1) +
                          BattConfigPerIna->size[0] * BattConfigPerIna->size[1] *
                          i] = Vdebug_data[i1 + k_ina219_ * i];
                      }
                    }
                  } else if (d == 1.6) {
                    i = b_BattConfigPerInaHelp->size[0] *
                      b_BattConfigPerInaHelp->size[1] *
                      b_BattConfigPerInaHelp->size[2];
                    b_BattConfigPerInaHelp->size[0] = 1;
                    b_BattConfigPerInaHelp->size[1] = BattConfigPerInaHelp->
                      size[1];
                    loop_ub = BattConfigPerInaHelp->size[2];
                    b_BattConfigPerInaHelp->size[2] = BattConfigPerInaHelp->
                      size[2];
                    emxEnsureCapacity_real_T(b_BattConfigPerInaHelp, i);
                    Vdebug_data = b_BattConfigPerInaHelp->data;
                    for (i = 0; i < loop_ub; i++) {
                      b_loop_ub = BattConfigPerInaHelp->size[1];
                      for (i1 = 0; i1 < b_loop_ub; i1++) {
                        Vdebug_data[i1 + b_BattConfigPerInaHelp->size[1] * i] =
                          BattConfigPerInaHelp_data[(b_k_tstState +
                          BattConfigPerInaHelp->size[0] * i1) +
                          BattConfigPerInaHelp->size[0] *
                          BattConfigPerInaHelp->size[1] * i];
                      }
                    }

                    k_ina219_ = BattConfigPerIna->size[1];
                    loop_ub = BattConfigPerIna->size[2];
                    for (i = 0; i < loop_ub; i++) {
                      for (i1 = 0; i1 < k_ina219_; i1++) {
                        BattConfigPerIna_data[(b_k_tstState +
                          BattConfigPerIna->size[0] * i1) +
                          BattConfigPerIna->size[0] * BattConfigPerIna->size[1] *
                          i] = Vdebug_data[i1 + k_ina219_ * i];
                      }
                    }
                  } else if (d == prm->brd.spi.disconnect) {
                    b_padArr(prm->brd.spi.disconnect, N_bat_tmp_tmp_tmp,
                             N_bat_tmp_tmp_tmp, r4);
                    IshuntTest2_data = r4->data;
                    k_ina219_ = BattConfigPerIna->size[1];
                    loop_ub = BattConfigPerIna->size[2];
                    for (i = 0; i < loop_ub; i++) {
                      for (i1 = 0; i1 < k_ina219_; i1++) {
                        BattConfigPerIna_data[(b_k_tstState +
                          BattConfigPerIna->size[0] * i1) +
                          BattConfigPerIna->size[0] * BattConfigPerIna->size[1] *
                          i] = IshuntTest2_data[i1 + k_ina219_ * i];
                      }
                    }
                  } else if (d == prm->brd.spi.bypass) {
                    b_padArr(prm->brd.spi.bypass, N_bat_tmp_tmp_tmp,
                             N_bat_tmp_tmp_tmp, r4);
                    IshuntTest2_data = r4->data;
                    k_ina219_ = BattConfigPerIna->size[1];
                    loop_ub = BattConfigPerIna->size[2];
                    for (i = 0; i < loop_ub; i++) {
                      for (i1 = 0; i1 < k_ina219_; i1++) {
                        BattConfigPerIna_data[(b_k_tstState +
                          BattConfigPerIna->size[0] * i1) +
                          BattConfigPerIna->size[0] * BattConfigPerIna->size[1] *
                          i] = IshuntTest2_data[i1 + k_ina219_ * i];
                      }
                    }
                  }

                  /* nadav Coder End */
                  /*  if SelDual(k_ina219)==0 */
                  /*      BattConfigPerIna{k_ina219} = BattConfigPerInaHelp{k_ina219}; */
                  /*      BattConfig{k_ina219}       = BattConfigPerIna{k_ina219} + (k_ina219-1)*N_bat; */
                  /*  elseif SelDual(k_ina219)==1 */
                  /*      BattConfigPerIna{k_ina219} = BattConfigPerInaHelp{k_ina219}; */
                  /*      BattConfig{k_ina219}       = BattConfigPerIna{k_ina219} + (k_ina219-1)*N_bat; */
                  /*  elseif SelDual(k_ina219)==1.5 */
                  /*      BattConfigPerIna{k_ina219} = BattConfigPerInaHelp{k_ina219}; */
                  /*      BattConfig{k_ina219}       = BattConfigPerIna{k_ina219} + (k_ina219-1)*N_bat; */
                  /*  elseif SelDual(k_ina219)==1.6 */
                  /*      BattConfigPerIna{k_ina219} = BattConfigPerInaHelp{k_ina219}; */
                  /*      BattConfig{k_ina219}       = BattConfigPerIna{k_ina219} + (k_ina219-1)*N_bat; */
                  /*  elseif SelDual(k_ina219)==prm.brd.spi.disconnect */
                  /*      BattConfigPerIna{k_ina219} = prm.brd.spi.disconnect; */
                  /*      BattConfig{k_ina219}       = prm.brd.spi.disconnect; */
                  /*  elseif SelDual(k_ina219)==prm.brd.spi.bypass */
                  /*      BattConfigPerIna{k_ina219} = prm.brd.spi.bypass; */
                  /*      BattConfig{k_ina219}       = prm.brd.spi.bypass; */
                  /*  end */
                }
              }

              b_guard1 = true;
            } else {
              exitg1 = true;
            }
          } else {
            exitg1 = true;
          }
        } else {
          for (b_k_tstState = 0; b_k_tstState < VbusTest_size_idx_0;
               b_k_tstState++) {
            /* Start new mode */
            if (switchFlag_data[b_k_tstState]) {
              d = SelDual_data[b_k_tstState];
              if (d == 0.0) {
                d_padArrUint8(prm->cnfg.BattConfigDis1, N_bat_tmp_tmp_tmp,
                              N_bat_tmp_tmp_tmp, Pac2Vid31);
                Pac2Vid31_data = Pac2Vid31->data;
                i = r4->size[0] * r4->size[1];
                r4->size[0] = Pac2Vid31->size[0];
                r4->size[1] = Pac2Vid31->size[1];
                emxEnsureCapacity_real_T(r4, i);
                IshuntTest2_data = r4->data;
                loop_ub = Pac2Vid31->size[0] * Pac2Vid31->size[1];
                for (i = 0; i < loop_ub; i++) {
                  IshuntTest2_data[i] = Pac2Vid31_data[i];
                }

                k_ina219_ = BattConfigPerIna->size[1];
                loop_ub = BattConfigPerIna->size[2];
                for (i = 0; i < loop_ub; i++) {
                  for (i1 = 0; i1 < k_ina219_; i1++) {
                    BattConfigPerIna_data[(b_k_tstState + BattConfigPerIna->
                      size[0] * i1) + BattConfigPerIna->size[0] *
                      BattConfigPerIna->size[1] * i] = IshuntTest2_data[i1 +
                      k_ina219_ * i];
                  }
                }
              } else if (d == 1.0) {
                d_padArrUint8(prm->cnfg.BattConfigChr1, N_bat_tmp_tmp_tmp,
                              N_bat_tmp_tmp_tmp, Pac2Vid31);
                Pac2Vid31_data = Pac2Vid31->data;
                i = r4->size[0] * r4->size[1];
                r4->size[0] = Pac2Vid31->size[0];
                r4->size[1] = Pac2Vid31->size[1];
                emxEnsureCapacity_real_T(r4, i);
                IshuntTest2_data = r4->data;
                loop_ub = Pac2Vid31->size[0] * Pac2Vid31->size[1];
                for (i = 0; i < loop_ub; i++) {
                  IshuntTest2_data[i] = Pac2Vid31_data[i];
                }

                k_ina219_ = BattConfigPerIna->size[1];
                loop_ub = BattConfigPerIna->size[2];
                for (i = 0; i < loop_ub; i++) {
                  for (i1 = 0; i1 < k_ina219_; i1++) {
                    BattConfigPerIna_data[(b_k_tstState + BattConfigPerIna->
                      size[0] * i1) + BattConfigPerIna->size[0] *
                      BattConfigPerIna->size[1] * i] = IshuntTest2_data[i1 +
                      k_ina219_ * i];
                  }
                }
              } else if (d == 1.5) {
                d_padArrUint8(prm->cnfg.BattConfigChr2, N_bat_tmp_tmp_tmp,
                              N_bat_tmp_tmp_tmp, Pac2Vid31);
                Pac2Vid31_data = Pac2Vid31->data;
                i = r4->size[0] * r4->size[1];
                r4->size[0] = Pac2Vid31->size[0];
                r4->size[1] = Pac2Vid31->size[1];
                emxEnsureCapacity_real_T(r4, i);
                IshuntTest2_data = r4->data;
                loop_ub = Pac2Vid31->size[0] * Pac2Vid31->size[1];
                for (i = 0; i < loop_ub; i++) {
                  IshuntTest2_data[i] = Pac2Vid31_data[i];
                }

                k_ina219_ = BattConfigPerIna->size[1];
                loop_ub = BattConfigPerIna->size[2];
                for (i = 0; i < loop_ub; i++) {
                  for (i1 = 0; i1 < k_ina219_; i1++) {
                    BattConfigPerIna_data[(b_k_tstState + BattConfigPerIna->
                      size[0] * i1) + BattConfigPerIna->size[0] *
                      BattConfigPerIna->size[1] * i] = IshuntTest2_data[i1 +
                      k_ina219_ * i];
                  }
                }
              } else if (d == 1.6) {
                d_padArrUint8(prm->cnfg.BattConfigChr3, N_bat_tmp_tmp_tmp,
                              N_bat_tmp_tmp_tmp, Pac2Vid31);
                Pac2Vid31_data = Pac2Vid31->data;
                i = r4->size[0] * r4->size[1];
                r4->size[0] = Pac2Vid31->size[0];
                r4->size[1] = Pac2Vid31->size[1];
                emxEnsureCapacity_real_T(r4, i);
                IshuntTest2_data = r4->data;
                loop_ub = Pac2Vid31->size[0] * Pac2Vid31->size[1];
                for (i = 0; i < loop_ub; i++) {
                  IshuntTest2_data[i] = Pac2Vid31_data[i];
                }

                k_ina219_ = BattConfigPerIna->size[1];
                loop_ub = BattConfigPerIna->size[2];
                for (i = 0; i < loop_ub; i++) {
                  for (i1 = 0; i1 < k_ina219_; i1++) {
                    BattConfigPerIna_data[(b_k_tstState + BattConfigPerIna->
                      size[0] * i1) + BattConfigPerIna->size[0] *
                      BattConfigPerIna->size[1] * i] = IshuntTest2_data[i1 +
                      k_ina219_ * i];
                  }
                }
              } else if (d == prm->brd.spi.disconnect) {
                e_padArrUint8(prm->brd.spi.disconnect, N_bat_tmp_tmp_tmp,
                              N_bat_tmp_tmp_tmp, Pac2Vid31);
                Pac2Vid31_data = Pac2Vid31->data;
                i = r4->size[0] * r4->size[1];
                r4->size[0] = Pac2Vid31->size[0];
                r4->size[1] = Pac2Vid31->size[1];
                emxEnsureCapacity_real_T(r4, i);
                IshuntTest2_data = r4->data;
                loop_ub = Pac2Vid31->size[0] * Pac2Vid31->size[1];
                for (i = 0; i < loop_ub; i++) {
                  IshuntTest2_data[i] = Pac2Vid31_data[i];
                }

                k_ina219_ = BattConfigPerIna->size[1];
                loop_ub = BattConfigPerIna->size[2];
                for (i = 0; i < loop_ub; i++) {
                  for (i1 = 0; i1 < k_ina219_; i1++) {
                    BattConfigPerIna_data[(b_k_tstState + BattConfigPerIna->
                      size[0] * i1) + BattConfigPerIna->size[0] *
                      BattConfigPerIna->size[1] * i] = IshuntTest2_data[i1 +
                      k_ina219_ * i];
                  }
                }
              } else if (d == prm->brd.spi.bypass) {
                e_padArrUint8(prm->brd.spi.bypass, N_bat_tmp_tmp_tmp,
                              N_bat_tmp_tmp_tmp, Pac2Vid31);
                Pac2Vid31_data = Pac2Vid31->data;
                i = r4->size[0] * r4->size[1];
                r4->size[0] = Pac2Vid31->size[0];
                r4->size[1] = Pac2Vid31->size[1];
                emxEnsureCapacity_real_T(r4, i);
                IshuntTest2_data = r4->data;
                loop_ub = Pac2Vid31->size[0] * Pac2Vid31->size[1];
                for (i = 0; i < loop_ub; i++) {
                  IshuntTest2_data[i] = Pac2Vid31_data[i];
                }

                k_ina219_ = BattConfigPerIna->size[1];
                loop_ub = BattConfigPerIna->size[2];
                for (i = 0; i < loop_ub; i++) {
                  for (i1 = 0; i1 < k_ina219_; i1++) {
                    BattConfigPerIna_data[(b_k_tstState + BattConfigPerIna->
                      size[0] * i1) + BattConfigPerIna->size[0] *
                      BattConfigPerIna->size[1] * i] = IshuntTest2_data[i1 +
                      k_ina219_ * i];
                  }
                }
              }

              /*  if SelDual(k_ina219)==0 */
              /*      BattConfigPerIna{k_ina219} = BattConfigDis1; */
              /*      BattConfig{k_ina219}       = BattConfigDis1 + (k_ina219-1)*N_bat; */
              /*  elseif SelDual(k_ina219)==1 */
              /*      BattConfigPerIna{k_ina219} = BattConfigChr1; */
              /*      BattConfig{k_ina219}       = BattConfigChr1 + (k_ina219-1)*N_bat; */
              /*  elseif SelDual(k_ina219)==1.5 */
              /*      BattConfigPerIna{k_ina219} = BattConfigChr2; */
              /*      BattConfig{k_ina219}       = BattConfigChr2 + (k_ina219-1)*N_bat; */
              /*  elseif SelDual(k_ina219)==1.6 */
              /*      BattConfigPerIna{k_ina219} = BattConfigChr3; */
              /*      BattConfig{k_ina219}       = BattConfigChr3 + (k_ina219-1)*N_bat; */
              /*  elseif SelDual(k_ina219)==prm.brd.spi.disconnect */
              /*      BattConfigPerIna{k_ina219} = prm.brd.spi.disconnect; */
              /*      BattConfig{k_ina219}       = prm.brd.spi.disconnect; */
              /*  elseif SelDual(k_ina219)==prm.brd.spi.bypass */
              /*      BattConfigPerIna{k_ina219} = prm.brd.spi.bypass; */
              /*      BattConfig{k_ina219}       = prm.brd.spi.bypass; */
              /*  end */
            }
          }

          b_guard1 = true;
        }

        if (b_guard1) {
          for (b_k_tstState = 0; b_k_tstState < VbusTest_size_idx_0;
               b_k_tstState++) {
            /*              %Start new mode */
            /*              if switchFlag(k_ina219) */
            /*                  if SelDual(k_ina219)==0 */
            /*                      BattConfigPerIna{k_ina219} = BattConfigDis1; */
            /*                      BattConfig{k_ina219}       = BattConfigDis1 + (k_ina219-1)*N_bat; */
            /*                  elseif SelDual(k_ina219)==1 */
            /*                      BattConfigPerIna{k_ina219} = BattConfigChr1; */
            /*                      BattConfig{k_ina219}       = BattConfigChr1 + (k_ina219-1)*N_bat; */
            /*                  elseif SelDual(k_ina219)==1.5 */
            /*                      BattConfigPerIna{k_ina219} = BattConfigChr2; */
            /*                      BattConfig{k_ina219}       = BattConfigChr2 + (k_ina219-1)*N_bat; */
            /*                  elseif SelDual(k_ina219)==1.6 */
            /*                      BattConfigPerIna{k_ina219} = BattConfigChr3; */
            /*                      BattConfig{k_ina219}       = BattConfigChr3 + (k_ina219-1)*N_bat; */
            /*                  elseif SelDual(k_ina219)==prm.brd.spi.disconnect */
            /*                      BattConfigPerIna{k_ina219} = prm.brd.spi.disconnect; */
            /*                      BattConfig{k_ina219}       = prm.brd.spi.disconnect; */
            /*                  elseif SelDual(k_ina219)==prm.brd.spi.bypass */
            /*                      BattConfigPerIna{k_ina219} = prm.brd.spi.bypass; */
            /*                      BattConfig{k_ina219}       = prm.brd.spi.bypass; */
            /*                  end */
            /*              end */
            /* Start new mode - End */
            if (changeConfigFlag_data[b_k_tstState] ||
                switchFlag_data[b_k_tstState]) {
              /* [SpiPortRow0,ina219State,Pac2Vid0] = CalcPortSpi(SwitchMat_esp,SwitchMat_esp2,PortSpiRow_esp,PortSpiRow_esp2,Pac2Vid,Pac2Vid2,BattConfigPerIna{k_ina219}); */
              i = b_BattConfigPerInaHelp->size[0] * b_BattConfigPerInaHelp->
                size[1] * b_BattConfigPerInaHelp->size[2];
              b_BattConfigPerInaHelp->size[0] = 1;
              b_BattConfigPerInaHelp->size[1] = BattConfigPerIna->size[1];
              loop_ub = BattConfigPerIna->size[2];
              b_BattConfigPerInaHelp->size[2] = BattConfigPerIna->size[2];
              emxEnsureCapacity_real_T(b_BattConfigPerInaHelp, i);
              Vdebug_data = b_BattConfigPerInaHelp->data;
              for (i = 0; i < loop_ub; i++) {
                b_loop_ub = BattConfigPerIna->size[1];
                for (i1 = 0; i1 < b_loop_ub; i1++) {
                  Vdebug_data[i1 + b_BattConfigPerInaHelp->size[1] * i] =
                    BattConfigPerIna_data[(b_k_tstState + BattConfigPerIna->
                    size[0] * i1) + BattConfigPerIna->size[0] *
                    BattConfigPerIna->size[1] * i];
                }
              }

              VdebugVec_size_idx_0_tmp = prm->Nmax.NbatMax;
              c_VdebugVec_data = *b_BattConfigPerInaHelp;
              e_Nina219[0] = prm->Nmax.NbatMax;
              e_Nina219[1] = prm->Nmax.NbatMax;
              c_VdebugVec_data.size = &e_Nina219[0];
              c_VdebugVec_data.numDimensions = 2;
              c_CalcPortSpiPrm(prm->brd.spi.disconnect, prm->brd.spi.bypass,
                               prm->brd.spi.PortSpiRow_esp,
                               prm->brd.spi.SwitchMat_esp,
                               prm->brd.spi.PortSpiRow_esp2,
                               prm->brd.spi.SwitchMat_esp2, prm->brd.spi.Pac2Vid,
                               prm->brd.spi.Pac2Vid2, &c_VdebugVec_data,
                               disConAll, a__1_data, id2Bypass_data, a__2);
              k_ina219_ = Pac2Vid0All->size[1];
              loop_ub = Pac2Vid0All->size[2];
              for (i = 0; i < loop_ub; i++) {
                for (i1 = 0; i1 < k_ina219_; i1++) {
                  Pac2Vid0All_data[(b_k_tstState + Pac2Vid0All->size[0] * i1) +
                    Pac2Vid0All->size[0] * Pac2Vid0All->size[1] * i] = a__2[i1 +
                    k_ina219_ * i];
                }
              }

              if (prm->seq.BrdBeforePSflag[state_k]) {
                switch (ProjectFlag) {
                 case 2:
                  break;

                 case 3:
                  /* UNTITLED Summary of this function goes here */
                  /*    Detailed explanation goes here */
                  /*  if coder.target('MATLAB') */
                  /*  read current */
                  /*   chip1     |   chip2     |   chip3 */
                  /* %comN2 Port4 chip1,comN1 Port11 chip1 */
                  /* %comP1 Port18 chip1 ,comP2 Port18 chip3 */
                  /*  */
                  /*  for k_row = 1:N_row */
                  /*      out123(k_row,:) = readSPI(SPI,PortSpiRow(k_row,1),N_chip); */
                  /*      oldPortSpiRow(k_row,1:2:end) = PortSpiRow(k_row,1:2:end); */
                  /*      oldPortSpiRow(k_row,2:2:end) = out123(k_row,2:2:end); */
                  /*  end */
                  /* mask current */
                  /* write current mask */
                  /*  for k_row = 1:N_row */
                  /*      write(SPI,oldPortSpiRowMask(k_row,:)); */
                  /*  end */
                  pause(0.001);

                  /* mask new */
                  /* write mask new */
                  /*  for k_row = 1:N_row */
                  /*      write(SPI0,PortSpiRowMask(k_row,:)); */
                  /*  end */
                  /* write new */
                  /*  for k_row = 1:N_row */
                  /*      write(SPI0,PortSpiRow(k_row,:)); */
                  /*  end */
                  /*  Rmsg = SPI_writeread(SPI0, PortSpiRow); */
                  /*  ReadPortSpiRow = RmsgData([[2:4] , 1] ,:); */
                  /*  ReadPortSpiRow(:,1:2:end) = ReadPortSpiRow(:,1:2:end);% - 128; */
                  /* mask current */
                  /* %chip1 Port6 Alert1, chip1 port 7 Alert3, chip1 port 8 Alert8, chip1 port 9 Alert6 */
                  /* %chip1 Port12 Alert7, chip1 Port13 Alert5, chip 1 port19 NC */
                  /* %chip1 Port20-23 NC */
                  /* chip1 Port28 Alert1, chip1 Port29 Alert2 */
                  /*  if sum(sum(sum(abs(double(PortSpiRow)-double(ReadPortSpiRow)))))>0 */
                  /*  end */
                  break;
                }

                /*                      disp([sprintf('%d',int32(k_ina219)) ':',num2str(Pac2Vid0(:,1)')]); */
                if ((unsigned int)b_k_tstState + 1U < 2147483648U) {
                  i = b_k_tstState + 1;
                } else {
                  i = MAX_int32_T;
                }

                b_sprintf(i, r20);
                for (id2Bypass_size = 0; id2Bypass_size <
                     VdebugVec_size_idx_0_tmp; id2Bypass_size++) {
                  NstTst_tmp_tmp = a__2[id2Bypass_size];
                  if (NstTst_tmp_tmp > 127) {
                    NstTst_tmp_tmp = 127U;
                  }

                  printf(" %d", (signed char)NstTst_tmp_tmp);
                  fflush(stdout);
                }

                changeConfigFlag_data[b_k_tstState] = false;
                switchFlag_data[b_k_tstState] = false;
                pause(0.05);
              }
            }
          }

          /* for k_ina219 */
          /*  change switch - End */
          b_meanIbrd0 = 0.0;
          V_dis_all = 0.0;

          /*  IchrPerPhase = zeros(1,sum(SelDual)); */
          /*  k0 = 1; */
          V_chr_RI = 0.0;
          N0 = 0.0;
          for (b_k_tstState = 0; b_k_tstState < VbusTest_size_idx_0;
               b_k_tstState++) {
            d = SelDual_data[b_k_tstState];
            if ((d >= 1.0) && (d < 2.0)) {
              /* Pac2Vid3(:,1); */
              loop_ub = Pac2Vid0All->size[1];
              VdebugVec_size_idx_0_tmp = Pac2Vid0All->size[1];
              for (i = 0; i < loop_ub; i++) {
                l_tmp_data[i] = (Pac2Vid0All_data[b_k_tstState +
                                 Pac2Vid0All->size[0] * i] > 0);
              }

              if (k_t0 >= 1) {
                id2Bypass_size = k_t0;
              } else {
                id2Bypass_size = 1;
              }

              /* ins */
              end = VdebugVec_size_idx_0_tmp - 1;
              trueCount = 0;
              loop_ub = 0;
              for (b_i = 0; b_i <= end; b_i++) {
                if (l_tmp_data[b_i]) {
                  trueCount++;
                  m_tmp_data[loop_ub] = (short)b_i;
                  loop_ub++;
                }
              }

              i = a__38->size[0] * a__38->size[1];
              a__38->size[0] = 1;
              a__38->size[1] = trueCount;
              emxEnsureCapacity_real_T(a__38, i);
              Vdebug_data = a__38->data;
              for (i = 0; i < trueCount; i++) {
                Vdebug_data[i] = ImKp184Test2_data[(b_k_tstState +
                  VbatMat0->size[0] * (Pac2Vid0All_data[b_k_tstState +
                  Pac2Vid0All->size[0] * m_tmp_data[i]] - 1)) + VbatMat0->size[0]
                  * VbatMat0->size[1] * (id2Bypass_size - 1)];
              }

              b_meanIbrd0 += b_sum(a__38);

              /* +I0'*Rwire(k_ina219,:);%prm.brd.Rint; */
              /*  limit Vchr calculation */
              /*  if SelDual(k_ina219)>=1&&SelDual(k_ina219)<2 */
              i = b_BattConfigPerInaHelp->size[0] * b_BattConfigPerInaHelp->
                size[1] * b_BattConfigPerInaHelp->size[2];
              b_BattConfigPerInaHelp->size[0] = 1;
              b_BattConfigPerInaHelp->size[1] = BattConfigPerIna->size[1];
              loop_ub = BattConfigPerIna->size[2];
              b_BattConfigPerInaHelp->size[2] = BattConfigPerIna->size[2];
              emxEnsureCapacity_real_T(b_BattConfigPerInaHelp, i);
              Vdebug_data = b_BattConfigPerInaHelp->data;
              for (i = 0; i < loop_ub; i++) {
                b_loop_ub = BattConfigPerIna->size[1];
                for (i1 = 0; i1 < b_loop_ub; i1++) {
                  Vdebug_data[i1 + b_BattConfigPerInaHelp->size[1] * i] =
                    BattConfigPerIna_data[(b_k_tstState + BattConfigPerIna->
                    size[0] * i1) + BattConfigPerIna->size[0] *
                    BattConfigPerIna->size[1] * i];
                }
              }

              remPadArr(b_BattConfigPerInaHelp, r4);
              N0 += (double)r4->size[0];

              /* size(BattConfigPerIna{k_ina0},1);%N_row(ina219StateAll(k_ina0)); */
              /*  end */
              IbatMat_size[0] = 1;
              IbatMat_size[1] = IbatMat->size[1];
              loop_ub = IbatMat->size[1];
              for (i = 0; i < loop_ub; i++) {
                b_IbatMat_data[i] = IbatMat_data[(b_k_tstState + IbatMat->size[0]
                  * i) + IbatMat->size[0] * IbatMat->size[1] * (id2Bypass_size -
                  1)];
              }

              b_abs(b_IbatMat_data, IbatMat_size, c_VmKp184Test_data,
                    VmKp184Test_size);
              d = 0.0;
              loop_ub = VmKp184Test_size[1];
              for (i = 0; i < loop_ub; i++) {
                a__16_data = c_VmKp184Test_data[i];
                if ((g_IchargeAct >= a__16_data) || rtIsNaN(a__16_data)) {
                  d1 = g_IchargeAct;
                } else {
                  d1 = a__16_data;
                }

                d += d1 * Rwire_data[b_k_tstState + Rwire->size[0] * i];
              }

              V_chr_RI += d;

              /* prm.brd.Rint; */
            }

            d = SelDual_data[b_k_tstState];
            if ((d >= 0.0) && (d < 1.0)) {
              /* Pac2Vid3(:,1); */
              loop_ub = Pac2Vid0All->size[1];
              VdebugVec_size_idx_0_tmp = Pac2Vid0All->size[1];
              for (i = 0; i < loop_ub; i++) {
                l_tmp_data[i] = (Pac2Vid0All_data[b_k_tstState +
                                 Pac2Vid0All->size[0] * i] > 0);
              }

              if (k_t0 >= 1) {
                id2Bypass_size = k_t0;
              } else {
                id2Bypass_size = 1;
              }

              end = VdebugVec_size_idx_0_tmp - 1;
              trueCount = 0;
              loop_ub = 0;
              for (b_i = 0; b_i <= end; b_i++) {
                if (l_tmp_data[b_i]) {
                  trueCount++;
                  n_tmp_data[loop_ub] = (short)b_i;
                  loop_ub++;
                }
              }

              i = a__38->size[0] * a__38->size[1];
              a__38->size[0] = 1;
              a__38->size[1] = trueCount;
              emxEnsureCapacity_real_T(a__38, i);
              Vdebug_data = a__38->data;
              for (i = 0; i < trueCount; i++) {
                Vdebug_data[i] = ImKp184Test2_data[(b_k_tstState +
                  VbatMat0->size[0] * (Pac2Vid0All_data[b_k_tstState +
                  Pac2Vid0All->size[0] * n_tmp_data[i]] - 1)) + VbatMat0->size[0]
                  * VbatMat0->size[1] * (id2Bypass_size - 1)];
              }

              V_dis_all += b_sum(a__38);

              /* -I0'*Rwire(k_ina219,:);%prm.brd.Rint; */
            }
          }

          e_Vcharge0 = (prm->bat.Vd + (c_maximum(prm->bat.CutOffChrV) + 0.1F) *
                        (float)N0) + (float)V_chr_RI;
          if (b_meanIbrd0 > 0.0) {
            ImaxChrB2B = (float)(V_dis_all / b_meanIbrd0) * ImaxDis *
              juntekEfficencyFactor;
          } else {
            /* k_t = 1; */
            V_dis_all = prm->ins.prm.jun.minVjuntekInput + 1.0;
            ImaxChrB2B = ImaxDis * juntekEfficencyFactor;
          }

          /* calc max I allowed & V discharge - End */
          b_meanIbrd0 = toc();
          f = b_mod(b_meanIbrd0, prm->bat.T);
          for (id2Bypass_size = 0; id2Bypass_size < 1024; id2Bypass_size++) {
            varargin_1[id2Bypass_size] = (float)fabs(prm->bat.t[id2Bypass_size]
              - f);
          }

          c_minimum(varargin_1, &VdebugVec_size_idx_0_tmp);
          c_vSumMax = rt_roundf_snf(1000.0F * prm->
            bat.i_in[VdebugVec_size_idx_0_tmp - 1]) / 1000.0F;

          /* calc I to discharge- End */
          if (onPowerFlag) {
            /*  if prm.ins.juntek */
            /*      controlJuntekDPH8920(prm.ser.s_juntek,'ON'); */
            /*  end */
            /*  if prm.ins.ka6005p */
            /*      ControlKa6005P(prm.ser.s_ka6005p,'On'); */
            /*  end */
            for (i = 0; i < 8; i++) {
              b_prm_data[i] = (prm->seq.ins[i + (state_k << 3)] == 1);
            }

            if (vectorAny(b_prm_data)) {
              /* prm.ins.kp184 */
              pause(0.05);
            }

            for (i = 0; i < 8; i++) {
              b_prm_data[i] = (prm->seq.ins[i + (state_k << 3)] == 2);
            }

            if (vectorAny(b_prm_data)) {
              /* prm.ins.ka6005p */
              b_k_tstState = 0;
              exitg2 = false;
              while ((!exitg2) && (b_k_tstState <= (int)Nina219_tmp_tmp - 1)) {
                if (SelDual_data[b_k_tstState] == 1.0) {
                  VdebugVec_size_idx_0_tmp = 0;
                } else if (SelDual_data[b_k_tstState] == 1.5) {
                  VdebugVec_size_idx_0_tmp = 1;
                } else if (SelDual_data[b_k_tstState] == 1.6) {
                  VdebugVec_size_idx_0_tmp = 2;
                } else {
                  VdebugVec_size_idx_0_tmp = -1;
                }

                switch (VdebugVec_size_idx_0_tmp) {
                 case 0:
                  g_IchargeAct = Icharge;
                  exitg2 = true;
                  break;

                 case 1:
                  g_IchargeAct = IchargePhase2t;
                  exitg2 = true;
                  break;

                 case 2:
                  g_IchargeAct = IchargePhase3;
                  exitg2 = true;
                  break;

                 default:
                  b_k_tstState++;
                  break;
                }
              }

              e_sprintf(e_Vcharge0, Vcharge0);

              /* ['VSET',num2str(ch),':',num2str(Val)];%[V] */
              k_sprintf(e_Vcharge0, b_Vcharge0);
              pause(0.05);
              l_sprintf(g_IchargeAct, IchargeAct);

              /* ['ISET',num2str(ch),':',num2str(Val)];%[A] */
              d_sprintf(g_IchargeAct, b_IchargeAct);
              pause(0.05);
            }

            for (i = 0; i < 8; i++) {
              b_prm_data[i] = (prm->seq.ins[i + (state_k << 3)] == 3);
            }

            if (vectorAny(b_prm_data)) {
              /*  3-juntek+ACDC */
              b_k_tstState = 0;
              exitg2 = false;
              while ((!exitg2) && (b_k_tstState <= (int)Nina219_tmp_tmp - 1)) {
                if (SelDual_data[b_k_tstState] == 1.0) {
                  VdebugVec_size_idx_0_tmp = 0;
                } else if (SelDual_data[b_k_tstState] == 1.5) {
                  VdebugVec_size_idx_0_tmp = 1;
                } else if (SelDual_data[b_k_tstState] == 1.6) {
                  VdebugVec_size_idx_0_tmp = 2;
                } else {
                  VdebugVec_size_idx_0_tmp = -1;
                }

                switch (VdebugVec_size_idx_0_tmp) {
                 case 0:
                  g_IchargeAct = Icharge;
                  exitg2 = true;
                  break;

                 case 1:
                  g_IchargeAct = IchargePhase2t;
                  exitg2 = true;
                  break;

                 case 2:
                  g_IchargeAct = IchargePhase3;
                  exitg2 = true;
                  break;

                 default:
                  b_k_tstState++;
                  break;
                }
              }

              /* Imax); */
              b_k_t0[0] = g_IchargeAct;
              b_k_t0[1] = ImaxAcDC;
              g_IchargeAct = rt_roundd_snf(1000.0 * d_minimum(b_k_t0)) / 1000.0;
              d = rt_roundd_snf(g_IchargeAct * 1000.0);
              if (d < 2.147483648E+9) {
                if (d >= -2.147483648E+9) {
                  i = (int)d;
                } else {
                  i = MIN_int32_T;
                }
              } else if (d >= 2.147483648E+9) {
                i = MAX_int32_T;
              } else {
                i = 0;
              }

              g_sprintf(i, r24);

              /* [':',juntek_address,'w11=',num2str(uint32(Val*1000),'%04d'),',',newline];%['VSET',num2str(ch),':',num2str(Val)];%[V] */
              d_sprintf(g_IchargeAct, d_IchargeAct);
              pause(0.1);
              f = rt_roundf_snf(e_Vcharge0 * 100.0F);
              if (f < 2.14748365E+9F) {
                if (f >= -2.14748365E+9F) {
                  i = (int)f;
                } else {
                  i = MIN_int32_T;
                }
              } else if (f >= 2.14748365E+9F) {
                i = MAX_int32_T;
              } else {
                i = 0;
              }

              g_sprintf(i, r27);

              /* [':',juntek_address,'w10=',num2str(uint32(Val*100),'%04d'),',',newline];%['VSET',num2str(ch),':',num2str(Val)];%[V] */
              k_sprintf(e_Vcharge0, c_Vcharge0);
              pause(0.1);
            }

            for (i = 0; i < 8; i++) {
              b_prm_data[i] = (prm->seq.ins[i + (state_k << 3)] == 4);
            }

            if (vectorAny(b_prm_data)) {
              /*  4-juntek B2B */
              b_k_tstState = 0;
              exitg2 = false;
              while ((!exitg2) && (b_k_tstState <= (int)Nina219_tmp_tmp - 1)) {
                if (SelDual_data[b_k_tstState] == 1.0) {
                  VdebugVec_size_idx_0_tmp = 0;
                } else if (SelDual_data[b_k_tstState] == 1.5) {
                  VdebugVec_size_idx_0_tmp = 1;
                } else if (SelDual_data[b_k_tstState] == 1.6) {
                  VdebugVec_size_idx_0_tmp = 2;
                } else {
                  VdebugVec_size_idx_0_tmp = -1;
                }

                switch (VdebugVec_size_idx_0_tmp) {
                 case 0:
                  g_IchargeAct = c_vSumMax;
                  exitg2 = true;
                  break;

                 case 1:
                  g_IchargeAct = IchargePhase2t;
                  exitg2 = true;
                  break;

                 case 2:
                  g_IchargeAct = IchargePhase3;
                  exitg2 = true;
                  break;

                 default:
                  b_k_tstState++;
                  break;
                }
              }

              /* Imax); */
              b_k_t0[0] = g_IchargeAct;
              b_k_t0[1] = ImaxChrB2B;
              g_IchargeAct = rt_roundd_snf(1000.0 * d_minimum(b_k_t0)) / 1000.0;
              d = rt_roundd_snf(g_IchargeAct * 1000.0);
              if (d < 2.147483648E+9) {
                if (d >= -2.147483648E+9) {
                  i = (int)d;
                } else {
                  i = MIN_int32_T;
                }
              } else if (d >= 2.147483648E+9) {
                i = MAX_int32_T;
              } else {
                i = 0;
              }

              g_sprintf(i, r26);

              /* [':',juntek_address,'w11=',num2str(uint32(Val*1000),'%04d'),',',newline];%['VSET',num2str(ch),':',num2str(Val)];%[V] */
              d_sprintf(g_IchargeAct, f_IchargeAct);
              pause(0.1);
              f = rt_roundf_snf(e_Vcharge0 * 100.0F);
              if (f < 2.14748365E+9F) {
                if (f >= -2.14748365E+9F) {
                  i = (int)f;
                } else {
                  i = MIN_int32_T;
                }
              } else if (f >= 2.14748365E+9F) {
                i = MAX_int32_T;
              } else {
                i = 0;
              }

              g_sprintf(i, r29);

              /* [':',juntek_address,'w10=',num2str(uint32(Val*100),'%04d'),',',newline];%['VSET',num2str(ch),':',num2str(Val)];%[V] */
              k_sprintf(e_Vcharge0, d_Vcharge0);
              pause(0.1);
            }
          } else {
            /*  if onPowerFlag == true; */
            for (i = 0; i < 8; i++) {
              k_groups_tmp = prm->seq.ins[i + (state_k << 3)];
              iv[i] = k_groups_tmp;
              b_prm_data[i] = (k_groups_tmp == 3);
            }

            if (vectorAny(b_prm_data)) {
              /*  3-juntek ACDC */
              i = b_changeConfigFlag->size[0] * b_changeConfigFlag->size[1];
              b_changeConfigFlag->size[0] = 1;
              b_changeConfigFlag->size[1] = SelDual->size[1];
              emxEnsureCapacity_boolean_T(b_changeConfigFlag, i);
              b_changeConfigFlag_data = b_changeConfigFlag->data;
              loop_ub = SelDual->size[1];
              for (i = 0; i < loop_ub; i++) {
                d = SelDual_data[i];
                b_changeConfigFlag_data[i] = ((d >= 1.0) && (d < 2.0));
              }

              if (b_any(b_changeConfigFlag)) {
                b_k_tstState = 0;
                exitg2 = false;
                while ((!exitg2) && (b_k_tstState <= (int)Nina219_tmp_tmp - 1))
                {
                  if (SelDual_data[b_k_tstState] == 1.0) {
                    VdebugVec_size_idx_0_tmp = 0;
                  } else if (SelDual_data[b_k_tstState] == 1.5) {
                    VdebugVec_size_idx_0_tmp = 1;
                  } else if (SelDual_data[b_k_tstState] == 1.6) {
                    VdebugVec_size_idx_0_tmp = 2;
                  } else {
                    VdebugVec_size_idx_0_tmp = -1;
                  }

                  switch (VdebugVec_size_idx_0_tmp) {
                   case 0:
                    g_IchargeAct = Icharge;
                    exitg2 = true;
                    break;

                   case 1:
                    g_IchargeAct = IchargePhase2t;
                    exitg2 = true;
                    break;

                   case 2:
                    g_IchargeAct = IchargePhase3;
                    exitg2 = true;
                    break;

                   default:
                    b_k_tstState++;
                    break;
                  }
                }

                /* Imax); */
                b_k_t0[0] = g_IchargeAct;
                b_k_t0[1] = ImaxAcDC;
                g_IchargeAct = rt_roundd_snf(1000.0 * d_minimum(b_k_t0)) /
                  1000.0;
                f = rt_roundf_snf(e_Vcharge0 * 100.0F);
                if (f < 2.14748365E+9F) {
                  if (f >= -2.14748365E+9F) {
                    i = (int)f;
                  } else {
                    i = MIN_int32_T;
                  }
                } else if (f >= 2.14748365E+9F) {
                  i = MAX_int32_T;
                } else {
                  i = 0;
                }

                g_sprintf(i, r21);

                /* [':',juntek_address,'w10=',num2str(uint32(Val*100),'%04d'),',',newline];%['VSET',num2str(ch),':',num2str(Val)];%[V] */
                d = rt_roundd_snf(g_IchargeAct * 1000.0);
                if (d < 2.147483648E+9) {
                  if (d >= -2.147483648E+9) {
                    i = (int)d;
                  } else {
                    i = MIN_int32_T;
                  }
                } else if (d >= 2.147483648E+9) {
                  i = MAX_int32_T;
                } else {
                  i = 0;
                }

                g_sprintf(i, r23);

                /* [':',juntek_address,'w11=',num2str(uint32(Val*1000),'%04d'),',',newline];%['VSET',num2str(ch),':',num2str(Val)];%[V] */
                d_sprintf(g_IchargeAct, c_IchargeAct);
                pause(0.1);
              }
            }

            for (i = 0; i < 8; i++) {
              b_prm_data[i] = (iv[i] == 4);
            }

            if (vectorAny(b_prm_data)) {
              /*  4-juntek B2B */
              i = b_changeConfigFlag->size[0] * b_changeConfigFlag->size[1];
              b_changeConfigFlag->size[0] = 1;
              b_changeConfigFlag->size[1] = SelDual->size[1];
              emxEnsureCapacity_boolean_T(b_changeConfigFlag, i);
              b_changeConfigFlag_data = b_changeConfigFlag->data;
              loop_ub = SelDual->size[1];
              for (i = 0; i < loop_ub; i++) {
                d = SelDual_data[i];
                b_changeConfigFlag_data[i] = ((d >= 1.0) && (d < 2.0));
              }

              if (b_any(b_changeConfigFlag)) {
                /* Imax); */
                b_k_t0[0] = c_vSumMax;
                b_k_t0[1] = ImaxChrB2B;
                g_IchargeAct = rt_roundd_snf(1000.0 * d_minimum(b_k_t0)) /
                  1000.0;
                f = rt_roundf_snf(e_Vcharge0 * 100.0F);
                if (f < 2.14748365E+9F) {
                  if (f >= -2.14748365E+9F) {
                    i = (int)f;
                  } else {
                    i = MIN_int32_T;
                  }
                } else if (f >= 2.14748365E+9F) {
                  i = MAX_int32_T;
                } else {
                  i = 0;
                }

                g_sprintf(i, r22);

                /* [':',juntek_address,'w10=',num2str(uint32(Val*100),'%04d'),',',newline];%['VSET',num2str(ch),':',num2str(Val)];%[V] */
                d = rt_roundd_snf(g_IchargeAct * 1000.0);
                if (d < 2.147483648E+9) {
                  if (d >= -2.147483648E+9) {
                    i = (int)d;
                  } else {
                    i = MIN_int32_T;
                  }
                } else if (d >= 2.147483648E+9) {
                  i = MAX_int32_T;
                } else {
                  i = 0;
                }

                g_sprintf(i, r25);

                /* [':',juntek_address,'w11=',num2str(uint32(Val*1000),'%04d'),',',newline];%['VSET',num2str(ch),':',num2str(Val)];%[V] */
                d_sprintf(g_IchargeAct, e_IchargeAct);
                pause(0.1);
              }
            }

            for (i = 0; i < 8; i++) {
              b_prm_data[i] = (iv[i] == 1);
            }

            if (vectorAny(b_prm_data)) {
              /* prm.ins.kp184 */
              pause(0.05);

              /*  ControlKp184(prm.ser.s_kp184,'On'); */
            }
          }

          onPowerFlag = false;
          pause(0.1);
          for (b_k_tstState = 0; b_k_tstState < VbusTest_size_idx_0;
               b_k_tstState++) {
            if (!prm->seq.BrdBeforePSflag[state_k]) {
              i = b_BattConfigPerInaHelp->size[0] * b_BattConfigPerInaHelp->
                size[1] * b_BattConfigPerInaHelp->size[2];
              b_BattConfigPerInaHelp->size[0] = 1;
              b_BattConfigPerInaHelp->size[1] = BattConfigPerIna->size[1];
              loop_ub = BattConfigPerIna->size[2];
              b_BattConfigPerInaHelp->size[2] = BattConfigPerIna->size[2];
              emxEnsureCapacity_real_T(b_BattConfigPerInaHelp, i);
              Vdebug_data = b_BattConfigPerInaHelp->data;
              for (i = 0; i < loop_ub; i++) {
                b_loop_ub = BattConfigPerIna->size[1];
                for (i1 = 0; i1 < b_loop_ub; i1++) {
                  Vdebug_data[i1 + b_BattConfigPerInaHelp->size[1] * i] =
                    BattConfigPerIna_data[(b_k_tstState + BattConfigPerIna->
                    size[0] * i1) + BattConfigPerIna->size[0] *
                    BattConfigPerIna->size[1] * i];
                }
              }

              VdebugVec_size_idx_0_tmp = prm->Nmax.NbatMax;
              c_VdebugVec_data = *b_BattConfigPerInaHelp;
              f_Nina219[0] = prm->Nmax.NbatMax;
              f_Nina219[1] = prm->Nmax.NbatMax;
              c_VdebugVec_data.size = &f_Nina219[0];
              c_VdebugVec_data.numDimensions = 2;
              c_CalcPortSpiPrm(prm->brd.spi.disconnect, prm->brd.spi.bypass,
                               prm->brd.spi.PortSpiRow_esp,
                               prm->brd.spi.SwitchMat_esp,
                               prm->brd.spi.PortSpiRow_esp2,
                               prm->brd.spi.SwitchMat_esp2, prm->brd.spi.Pac2Vid,
                               prm->brd.spi.Pac2Vid2, &c_VdebugVec_data,
                               disConAll, a__1_data, id2Bypass_data, a__2);
              k_ina219_ = Pac2Vid0All->size[1];
              loop_ub = Pac2Vid0All->size[2];
              for (i = 0; i < loop_ub; i++) {
                for (i1 = 0; i1 < k_ina219_; i1++) {
                  Pac2Vid0All_data[(b_k_tstState + Pac2Vid0All->size[0] * i1) +
                    Pac2Vid0All->size[0] * Pac2Vid0All->size[1] * i] = a__2[i1 +
                    k_ina219_ * i];
                }
              }

              if (changeConfigFlag_data[b_k_tstState] ||
                  switchFlag_data[b_k_tstState]) {
                switch (ProjectFlag) {
                 case 2:
                  break;

                 case 3:
                  /* UNTITLED Summary of this function goes here */
                  /*    Detailed explanation goes here */
                  /*  if coder.target('MATLAB') */
                  /*  read current */
                  /*   chip1     |   chip2     |   chip3 */
                  /* %comN2 Port4 chip1,comN1 Port11 chip1 */
                  /* %comP1 Port18 chip1 ,comP2 Port18 chip3 */
                  /*  */
                  /*  for k_row = 1:N_row */
                  /*      out123(k_row,:) = readSPI(SPI,PortSpiRow(k_row,1),N_chip); */
                  /*      oldPortSpiRow(k_row,1:2:end) = PortSpiRow(k_row,1:2:end); */
                  /*      oldPortSpiRow(k_row,2:2:end) = out123(k_row,2:2:end); */
                  /*  end */
                  /* mask current */
                  /* write current mask */
                  /*  for k_row = 1:N_row */
                  /*      write(SPI,oldPortSpiRowMask(k_row,:)); */
                  /*  end */
                  pause(0.001);

                  /* mask new */
                  /* write mask new */
                  /*  for k_row = 1:N_row */
                  /*      write(SPI0,PortSpiRowMask(k_row,:)); */
                  /*  end */
                  /* write new */
                  /*  for k_row = 1:N_row */
                  /*      write(SPI0,PortSpiRow(k_row,:)); */
                  /*  end */
                  /*  Rmsg = SPI_writeread(SPI0, PortSpiRow); */
                  /*  ReadPortSpiRow = RmsgData([[2:4] , 1] ,:); */
                  /*  ReadPortSpiRow(:,1:2:end) = ReadPortSpiRow(:,1:2:end);% - 128; */
                  /* mask current */
                  /* %chip1 Port6 Alert1, chip1 port 7 Alert3, chip1 port 8 Alert8, chip1 port 9 Alert6 */
                  /* %chip1 Port12 Alert7, chip1 Port13 Alert5, chip 1 port19 NC */
                  /* %chip1 Port20-23 NC */
                  /* chip1 Port28 Alert1, chip1 Port29 Alert2 */
                  /*  if sum(sum(sum(abs(double(PortSpiRow)-double(ReadPortSpiRow)))))>0 */
                  /*  end */
                  break;
                }

                /*  disp([num2str(k_ina219) ':',num2str(Pac2Vid0(:,1)')]); */
                if ((unsigned int)b_k_tstState + 1U < 2147483648U) {
                  i = b_k_tstState + 1;
                } else {
                  i = MAX_int32_T;
                }

                b_sprintf(i, r28);
                for (id2Bypass_size = 0; id2Bypass_size <
                     VdebugVec_size_idx_0_tmp; id2Bypass_size++) {
                  NstTst_tmp_tmp = a__2[id2Bypass_size];
                  if (NstTst_tmp_tmp > 127) {
                    NstTst_tmp_tmp = 127U;
                  }

                  printf(" %d", (signed char)NstTst_tmp_tmp);
                  fflush(stdout);
                }

                changeConfigFlag_data[b_k_tstState] = false;
                switchFlag_data[b_k_tstState] = false;
                pause(0.05);
              }
            }
          }

          /* On power and load - End */
          keepMeas = b_ReadVI(Nina219_tmp_tmp, ProjectFlag, prm->brd.pac.VIpacId,
                              Pac2Vid0All, N_bat1, N_bat_tmp_tmp_tmp, pIacs758,
                              Iacs758Flag, pIshunt, Rwire, a__64, a__65, a__66,
                              a__67, Rtot, a__68, a__69, a__70, a__71, a__72,
                              a__73, RtotHelpPoly, (double *)&errI2C_data,
                              errI2C_size);
          Vdebug_data = a__70->data;
          VmV_data = a__69->data;
          pIshunt_data = a__68->data;
          Rtot_data = Rtot->data;
          pIacs758_data = a__67->data;
          b_Rwire_data = a__64->data;
          loop_ub = Vbat->size[0];
          for (i = 0; i < loop_ub; i++) {
            RtotHelpPoly_data[i + Vbat->size[0] * k_t0] = b_Rwire_data[i];
          }

          Nina219[0] = VbatMat->size[0];
          loop_ub = VbatMat->size[1];
          for (i = 0; i < loop_ub; i++) {
            b_loop_ub = Nina219[0];
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              VbusTest2_data[(i1 + VbatMat->size[0] * i) + VbatMat->size[0] *
                VbatMat->size[1] * k_t0] = pIacs758_data[i1 + Nina219[0] * i];
            }
          }

          Nina219[0] = VbatMat0->size[0];
          loop_ub = VbatMat0->size[1];
          for (i = 0; i < loop_ub; i++) {
            b_loop_ub = Nina219[0];
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              ImKp184Test2_data[(i1 + VbatMat0->size[0] * i) + VbatMat0->size[0]
                * VbatMat0->size[1] * k_t0] = Rtot_data[i1 + Nina219[0] * i];
            }
          }

          Nina219[0] = IbatMat->size[0];
          loop_ub = IbatMat->size[1];
          for (i = 0; i < loop_ub; i++) {
            b_loop_ub = Nina219[0];
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              IbatMat_data[(i1 + IbatMat->size[0] * i) + IbatMat->size[0] *
                IbatMat->size[1] * k_t0] = pIshunt_data[i1 + Nina219[0] * i];
            }
          }

          Nina219[0] = tV->size[0];
          loop_ub = tV->size[1];
          for (i = 0; i < loop_ub; i++) {
            b_loop_ub = Nina219[0];
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              tV_data[(i1 + tV->size[0] * i) + tV->size[0] * tV->size[1] * k_t0]
                = VmV_data[i1 + Nina219[0] * i];
            }
          }

          loop_ub = tV1->size[0];
          for (i = 0; i < loop_ub; i++) {
            Iacs758_cal_data[i + tV1->size[0] * k_t0] = Vdebug_data[i];
          }

          /*  read V,I */
          for (b_k_tstState = 0; b_k_tstState < VbusTest_size_idx_0;
               b_k_tstState++) {
            if (kalmanFlag) {
              VmKp184Test_size[0] = 1;
              loop_ub = tV->size[1];
              VmKp184Test_size[1] = tV->size[1];
              for (i = 0; i < loop_ub; i++) {
                c_VmKp184Test_data[i] = tV_data[(b_k_tstState + tV->size[0] * i)
                  + tV->size[0] * tV->size[1] * k_t0];
              }

              loop_ub = IbatMat->size[1];
              for (i = 0; i < loop_ub; i++) {
                b_IbatMat_data[i] = IbatMat_data[(b_k_tstState + IbatMat->size[0]
                  * i) + IbatMat->size[0] * IbatMat->size[1] * k_t0];
              }

              loop_ub = VbatMat->size[1];
              for (i = 0; i < loop_ub; i++) {
                VbatMat_data[i] = VbusTest2_data[(b_k_tstState + VbatMat->size[0]
                  * i) + VbatMat->size[0] * VbatMat->size[1] * k_t0];
              }

              b_CalcStructKalman(&prm->klm.b_struct[b_k_tstState], Ta,
                                 c_VmKp184Test_data, VmKp184Test_size,
                                 b_IbatMat_data, VbatMat_data);
            }
          }

          /* kalman - End */
          for (i = 0; i < 8; i++) {
            b_prm_data[i] = (prm->seq.ins[i + (state_k << 3)] == 3);
          }

          guard5 = false;
          if (vectorAny(b_prm_data)) {
            guard5 = true;
          } else {
            for (i = 0; i < 8; i++) {
              b_prm_data[i] = (prm->seq.ins[i + (state_k << 3)] == 4);
            }

            if (vectorAny(b_prm_data)) {
              guard5 = true;
            } else {
              for (i = 0; i < 8; i++) {
                b_prm_data[i] = (prm->seq.ins[i + (state_k << 3)] == 1);
              }

              if (vectorAny(b_prm_data)) {
                ControlKp184((double *)&a__3_data, a__3_size, (double *)
                             &b_meanIbrd0, a__4_size, (double *)&N0, tmp_size,
                             (double *)&a__16_data, a__16_size);
              } else {
                for (i = 0; i < 8; i++) {
                  b_prm_data[i] = (prm->seq.ins[i + (state_k << 3)] == 2);
                }

                if (vectorAny(b_prm_data)) {
                  /* read I */
                  d_rand();

                  /* read I */
                  d_rand();
                }
              }
            }
          }

          if (guard5) {
            controlJuntekDPH8920(r31.data, r31.size);
            pause(0.1);
            b_controlJuntekDPH8920(r31.data, r31.size);
          }

          i = SelDual_nm1->size[0] * SelDual_nm1->size[1];
          SelDual_nm1->size[0] = 1;
          SelDual_nm1->size[1] = SelDual->size[1];
          emxEnsureCapacity_real_T(SelDual_nm1, i);
          Vdebug_data = SelDual_nm1->data;
          loop_ub = SelDual->size[1];
          for (i = 0; i < loop_ub; i++) {
            Vdebug_data[i] = SelDual_data[i];
          }

          /*  BattConfigAct = BattConfigAct(:); */
          if (Nina219_tmp_tmp < 1.0) {
            VecIna219->size[0] = 1;
            VecIna219->size[1] = 0;
          } else {
            i = VecIna219->size[0] * VecIna219->size[1];
            VecIna219->size[0] = 1;
            VecIna219->size[1] = (int)(Nina219_tmp_tmp - 1.0) + 1;
            emxEnsureCapacity_real_T(VecIna219, i);
            VecIna219_data = VecIna219->data;
            loop_ub = (int)(Nina219_tmp_tmp - 1.0);
            for (i = 0; i <= loop_ub; i++) {
              VecIna219_data[i] = (double)i + 1.0;
            }
          }

          if (b_size - 1 >= 0) {
            e_end = ipos_size - 1;
          }

          for (k_groups = 0; k_groups < b_size; k_groups++) {
            trueCount = 0;
            loop_ub = 0;
            for (b_i = 0; b_i <= e_end; b_i++) {
              if (k_groups + 1 == ipos_data[b_i]) {
                trueCount++;
                q_tmp_data[loop_ub] = (signed char)b_i;
                loop_ub++;
              }
            }

            /* VecIna219_k = find(k_groups == uGroupId); */
            if (N_bat_tmp_tmp_tmp < 1) {
              y->size[0] = 1;
              y->size[1] = 0;
            } else {
              i = y->size[0] * y->size[1];
              y->size[0] = 1;
              y->size[1] = N_bat_tmp_tmp_tmp;
              emxEnsureCapacity_uint16_T(y, i);
              y_data = y->data;
              loop_ub = N_bat_tmp_tmp_tmp - 1;
              for (i = 0; i <= loop_ub; i++) {
                y_data[i] = (unsigned short)((unsigned int)i + 1U);
              }
            }

            /*          for k_ina219 = 1:Nina219 */
            /*              if SelDual(k_ina219) ~= prm.brd.spi.bypass&&SelDual(k_ina219) ~= prm.brd.spi.disconnect */
            i = r1->size[0] * r1->size[1] * r1->size[2];
            r1->size[0] = trueCount;
            r1->size[1] = BattConfigPerIna->size[1];
            loop_ub = BattConfigPerIna->size[2];
            r1->size[2] = BattConfigPerIna->size[2];
            emxEnsureCapacity_real_T(r1, i);
            IshuntTest2_data = r1->data;
            for (i = 0; i < loop_ub; i++) {
              b_loop_ub = BattConfigPerIna->size[1];
              for (i1 = 0; i1 < b_loop_ub; i1++) {
                for (i2 = 0; i2 < trueCount; i2++) {
                  IshuntTest2_data[(i2 + r1->size[0] * i1) + r1->size[0] *
                    r1->size[1] * i] = BattConfigPerIna_data[(((int)
                    VecIna219_data[q_tmp_data[i2]] + BattConfigPerIna->size[0] *
                    i1) + BattConfigPerIna->size[0] * BattConfigPerIna->size[1] *
                    i) - 1];
                }
              }
            }

            c_tmp_size[0] = 1;
            c_tmp_size[1] = trueCount;
            d_tmp_size[0] = 1;
            d_tmp_size[1] = trueCount;
            e_tmp_size[0] = BattConfigAct->size[0];
            e_tmp_size[1] = trueCount;
            for (i = 0; i < trueCount; i++) {
              i1 = (int)VecIna219_data[q_tmp_data[i]] - 1;
              outVI[i] = SelDual_data[i1];
              o_tmp_data[i] = tLastToggle_data[i1];
              loop_ub = BattConfigAct->size[0];
              for (i2 = 0; i2 < loop_ub; i2++) {
                p_tmp_data[i2 + e_tmp_size[0] * i] = BattConfigAct_data[i2 +
                  BattConfigAct->size[0] * i1];
              }
            }

            if (trueCount == y->size[1]) {
              i = VmV->size[0];
              VmV->size[0] = y->size[1];
              emxEnsureCapacity_real_T(VmV, i);
              VmV_data = VmV->data;
              loop_ub = y->size[1];
              for (i = 0; i < loop_ub; i++) {
                VmV_data[i] = RtotHelpPoly_data[((int)((double)y_data[i] +
                  (VecIna219_data[q_tmp_data[i]] - 1.0) * (double)
                  N_bat_tmp_tmp_tmp) + Vbat->size[0] * k_t0) - 1];
              }

              i = (int)prm->seq.vth[state_k] - 1;
              showVdiff = d_Esp32StepSwitchToggleCombAll_(outVI, c_tmp_size,
                prm->bat.CutOffChrV[i], prm->bat.CutOffDisV[i], r1, VmV,
                trueCount, prm->brd.Nbat, prm->brd.spi.disconnect,
                prm->brd.spi.bypass, prm->cnfg.Ttoggle, prm->cnfg.NtoggleDrop,
                prm->cnfg.minLenIna219, prm->seq.pwr, prm->
                seq.pwr.VthFlag[state_k], tV_data[tV->size[0] * tV->size[1] *
                k_t0], o_tmp_data, d_tmp_size, p_tmp_data, e_tmp_size,
                AllBypassPerGroup_data, id2Bypass_data);
              IshuntTest2_data = r1->data;
            } else {
              showVdiff = binary_expand_op_4(outVI, c_tmp_size, prm, state_k, r1,
                Vbat, y, VecIna219, q_tmp_data, &trueCount, N_bat_tmp_tmp_tmp,
                k_t0, tV, o_tmp_data, d_tmp_size, p_tmp_data, e_tmp_size,
                AllBypassPerGroup_data, id2Bypass_data);
              IshuntTest2_data = r1->data;
            }

            for (i = 0; i < trueCount; i++) {
              id2Bypass_data[i] = (int)VecIna219_data[q_tmp_data[i]];
            }

            b_loop_ub_tmp = trueCount - 1;
            for (i = 0; i <= b_loop_ub_tmp; i++) {
              changeConfigFlag_data[id2Bypass_data[i] - 1] =
                AllBypassPerGroup_data[i];
            }

            for (i = 0; i < trueCount; i++) {
              id2Bypass_data[i] = (int)VecIna219_data[q_tmp_data[i]] - 1;
            }

            k_ina219_ = BattConfigPerIna->size[1];
            loop_ub = BattConfigPerIna->size[2];
            for (i = 0; i < loop_ub; i++) {
              for (i1 = 0; i1 < k_ina219_; i1++) {
                for (i2 = 0; i2 < trueCount; i2++) {
                  BattConfigPerIna_data[(id2Bypass_data[i2] +
                    BattConfigPerIna->size[0] * i1) + BattConfigPerIna->size[0] *
                    BattConfigPerIna->size[1] * i] = IshuntTest2_data[(i2 +
                    trueCount * i1) + trueCount * k_ina219_ * i];
                }
              }
            }

            for (i = 0; i < trueCount; i++) {
              id2Bypass_data[i] = (int)VecIna219_data[q_tmp_data[i]];
            }

            for (i = 0; i < trueCount; i++) {
              SelDual_data[id2Bypass_data[i] - 1] = outVI[i];
            }

            for (i = 0; i < trueCount; i++) {
              id2Bypass_data[i] = (int)VecIna219_data[q_tmp_data[i]];
            }

            for (i = 0; i <= b_loop_ub_tmp; i++) {
              switchFlag_data[id2Bypass_data[i] - 1] = showVdiff;
            }

            for (i = 0; i < trueCount; i++) {
              id2Bypass_data[i] = (int)VecIna219_data[q_tmp_data[i]];
            }

            for (i = 0; i <= b_loop_ub_tmp; i++) {
              tLastToggle_data[id2Bypass_data[i] - 1] = o_tmp_data[i];
            }

            for (i = 0; i < trueCount; i++) {
              id2Bypass_data[i] = (int)VecIna219_data[q_tmp_data[i]] - 1;
            }

            Nina219[0] = BattConfigAct->size[0];
            for (i = 0; i < trueCount; i++) {
              loop_ub = Nina219[0];
              for (i1 = 0; i1 < loop_ub; i1++) {
                BattConfigAct_data[i1 + BattConfigAct->size[0] *
                  id2Bypass_data[i]] = p_tmp_data[i1 + Nina219[0] * i];
              }
            }

            /* %squeeze(VbatMat(k_ina219,:,k_t0))',... */
            /* %(:,state_k),... */
          }

          /*          SelDual_nm1 = SelDual; */
          /*          vt_id_state_k = seq.vth(state_k); */
          /*          CutOffChrV_k = CutOffChrV(vt_id_state_k); */
          /*          CutOffDisV_k = CutOffDisV(vt_id_state_k); */
          /*          for k_ina219 = 1:Nina219 */
          /*              if SelDual(k_ina219) ~= prm.brd.spi.bypass&&SelDual(k_ina219) ~= prm.brd.spi.disconnect */
          /*                  [changeConfigFlag(k_ina219),... */
          /*                      BattConfig(k_ina219),... */
          /*                      BattConfigPerIna(k_ina219),... */
          /*                      SelDual(k_ina219),switchFlag(k_ina219),tLastToggle(k_ina219),... */
          /*                      BattConfigAct(k_ina219)] = ... */
          /*                      Esp32StepSwitchToggleCombAll_ESP(... */
          /*                      SelDual(k_ina219),CutOffChrV(seq.vth(state_k)),CutOffDisV(seq.vth(state_k)),... */
          /*                      BattConfigPerIna(k_ina219),... */
          /*                      Vbat((1:N_bat)+(k_ina219-1)*N_bat ,k_t),...%squeeze(VbatMat(k_ina219,:,k_t))',... */
          /*                      1,... */
          /*                      SwitchMat_esp,SwitchMat_esp2,PortSpiRow_esp,PortSpiRow_esp2,minLenIna219,... */
          /*                      ToggleFlag,NtoggleDrop,Ttoggle,tV(1,1,k_t),tLastToggle(k_ina219),N_bat,... */
          /*                      BattConfigAct(k_ina219)); */
          /*                  if ~isempty(BattConfig{k_ina219}) */
          /*                      BattConfig{k_ina219} = BattConfig{k_ina219} + (k_ina219-1)*N_bat; */
          /*                  end */
          /*              end */
          /*          end */
          /*          SelDualVsT(:,k_t) = SelDual; */
          /*  test toggle - End */
          newState = false;
          for (b_k_tstState = 0; b_k_tstState < VbusTest_size_idx_0;
               b_k_tstState++) {
            if (switchFlag_data[b_k_tstState]) {
              SelDual_data[b_k_tstState] = prm->brd.spi.bypass;

              /* bypass (short) */
              /*  BattConfigPerIna{k_ina219} = prm.brd.spi.bypass; */
              /*  BattConfig{k_ina219}       = prm.brd.spi.bypass; */
            }
          }

          /*  test all groups */
          nextStatePerGroup_size[0] = 1;
          nextStatePerGroup_size[1] = b_size;
          for (id2Bypass_size = 0; id2Bypass_size < b_size; id2Bypass_size++) {
            nextStatePerGroup_data[id2Bypass_size] = false;
            trueCount = 0;
            for (b_i = 0; b_i <= end_tmp; b_i++) {
              if (b_data[id2Bypass_size] == prm->ser.com.grp[tmp_data[b_i]]) {
                trueCount++;
              }
            }

            i = b_x->size[0] * b_x->size[1];
            b_x->size[0] = 1;
            b_x->size[1] = trueCount;
            emxEnsureCapacity_boolean_T(b_x, i);
            b_changeConfigFlag_data = b_x->data;
            loop_ub = 0;
            for (b_i = 0; b_i <= end_tmp; b_i++) {
              if (b_data[id2Bypass_size] == prm->ser.com.grp[tmp_data[b_i]]) {
                b_changeConfigFlag_data[loop_ub] = (SelDual_data[b_i] ==
                  prm->brd.spi.bypass);
                loop_ub++;
              }
            }

            showVdiff = true;
            VdebugVec_size_idx_0_tmp = 1;
            exitg2 = false;
            while ((!exitg2) && (VdebugVec_size_idx_0_tmp <= b_x->size[1])) {
              if (!b_changeConfigFlag_data[VdebugVec_size_idx_0_tmp - 1]) {
                showVdiff = false;
                exitg2 = true;
              } else {
                VdebugVec_size_idx_0_tmp++;
              }
            }

            if (showVdiff) {
              trueCount = 0;
              loop_ub = 0;
              for (b_i = 0; b_i <= end_tmp; b_i++) {
                i5 = b_data[id2Bypass_size];
                i = prm->ser.com.grp[tmp_data[b_i]];
                if (i5 == i) {
                  trueCount++;
                  r_tmp_data[loop_ub] = (signed char)b_i;
                  loop_ub++;
                }
              }

              SelDual_nm1_size[0] = 1;
              SelDual_nm1_size[1] = trueCount;
              for (i = 0; i < trueCount; i++) {
                SelDual_nm1_data[i] = (Vdebug_data[r_tmp_data[i]] == 0.0);
              }

              c_VbusTest0_data.data = &SelDual_nm1_data[0];
              c_VbusTest0_data.size = &SelDual_nm1_size[0];
              c_VbusTest0_data.allocatedSize = 2;
              c_VbusTest0_data.numDimensions = 2;
              c_VbusTest0_data.canFreeData = false;
              if (b_any(&c_VbusTest0_data)) {
                nextStatePerGroup_data[id2Bypass_size] = true;
                prm->seq.ins[id2Bypass_size + (state_k << 3)] = 0;
                for (b_i = 0; b_i <= end_tmp; b_i++) {
                  if (b_data[id2Bypass_size] == prm->ser.com.grp[tmp_data[b_i]])
                  {
                    SelDual_data[b_i] = prm->brd.spi.disconnect;
                  }
                }
              } else {
                trueCount = 0;
                loop_ub = 0;
                for (b_i = 0; b_i <= end_tmp; b_i++) {
                  i5 = b_data[id2Bypass_size];
                  i = prm->ser.com.grp[tmp_data[b_i]];
                  if (i5 == i) {
                    trueCount++;
                    s_tmp_data[loop_ub] = (signed char)b_i;
                    loop_ub++;
                  }
                }

                SelDual_nm1_size[0] = 1;
                SelDual_nm1_size[1] = trueCount;
                for (i = 0; i < trueCount; i++) {
                  SelDual_nm1_data[i] = (Vdebug_data[s_tmp_data[i]] == 1.0);
                }

                b_SelDual_nm1_data.data = &SelDual_nm1_data[0];
                b_SelDual_nm1_data.size = &SelDual_nm1_size[0];
                b_SelDual_nm1_data.allocatedSize = 2;
                b_SelDual_nm1_data.numDimensions = 2;
                b_SelDual_nm1_data.canFreeData = false;
                if (b_any(&b_SelDual_nm1_data)) {
                  for (b_i = 0; b_i <= end_tmp; b_i++) {
                    if (b_data[id2Bypass_size] == prm->ser.com.grp[tmp_data[b_i]])
                    {
                      SelDual_data[b_i] = 1.5;
                    }
                  }

                  IchargePhase2t = IchargePhase2_tmp_tmp;
                } else {
                  trueCount = 0;
                  loop_ub = 0;
                  for (b_i = 0; b_i <= end_tmp; b_i++) {
                    i5 = b_data[id2Bypass_size];
                    i = prm->ser.com.grp[tmp_data[b_i]];
                    if (i5 == i) {
                      trueCount++;
                      t_tmp_data[loop_ub] = (signed char)b_i;
                      loop_ub++;
                    }
                  }

                  SelDual_nm1_size[0] = 1;
                  SelDual_nm1_size[1] = trueCount;
                  for (i = 0; i < trueCount; i++) {
                    SelDual_nm1_data[i] = (Vdebug_data[t_tmp_data[i]] == 1.5);
                  }

                  c_SelDual_nm1_data.data = &SelDual_nm1_data[0];
                  c_SelDual_nm1_data.size = &SelDual_nm1_size[0];
                  c_SelDual_nm1_data.allocatedSize = 2;
                  c_SelDual_nm1_data.numDimensions = 2;
                  c_SelDual_nm1_data.canFreeData = false;
                  if (b_any(&c_SelDual_nm1_data)) {
                    IchargePhase2t -= prm->bat.dIphase2;
                    if (IchargePhase2t < prm->bat.minIphase2) {
                      for (b_i = 0; b_i <= end_tmp; b_i++) {
                        if (b_data[id2Bypass_size] == prm->
                            ser.com.grp[tmp_data[b_i]]) {
                          SelDual_data[b_i] = 1.6;
                        }
                      }

                      IchargePhase2t = IchargePhase2_tmp_tmp;
                    } else {
                      for (b_i = 0; b_i <= end_tmp; b_i++) {
                        if (b_data[id2Bypass_size] == prm->
                            ser.com.grp[tmp_data[b_i]]) {
                          SelDual_data[b_i] = 1.5;
                        }
                      }
                    }
                  } else {
                    trueCount = 0;
                    loop_ub = 0;
                    for (b_i = 0; b_i <= end_tmp; b_i++) {
                      i5 = b_data[id2Bypass_size];
                      i = prm->ser.com.grp[tmp_data[b_i]];
                      if (i5 == i) {
                        trueCount++;
                        u_tmp_data[loop_ub] = (signed char)b_i;
                        loop_ub++;
                      }
                    }

                    SelDual_nm1_size[0] = 1;
                    SelDual_nm1_size[1] = trueCount;
                    for (i = 0; i < trueCount; i++) {
                      SelDual_nm1_data[i] = (Vdebug_data[u_tmp_data[i]] == 1.6);
                    }

                    d_SelDual_nm1_data.data = &SelDual_nm1_data[0];
                    d_SelDual_nm1_data.size = &SelDual_nm1_size[0];
                    d_SelDual_nm1_data.allocatedSize = 2;
                    d_SelDual_nm1_data.numDimensions = 2;
                    d_SelDual_nm1_data.canFreeData = false;
                    if (b_any(&d_SelDual_nm1_data)) {
                      nextStatePerGroup_data[id2Bypass_size] = true;
                      prm->seq.ins[id2Bypass_size + (state_k << 3)] = 0;
                      for (b_i = 0; b_i <= end_tmp; b_i++) {
                        if (b_data[id2Bypass_size] == prm->
                            ser.com.grp[tmp_data[b_i]]) {
                          SelDual_data[b_i] = prm->brd.spi.disconnect;
                        }
                      }
                    }
                  }
                }
              }
            }
          }

          showVdiff = true;
          VdebugVec_size_idx_0_tmp = 1;
          exitg2 = false;
          while ((!exitg2) && (VdebugVec_size_idx_0_tmp <=
                               nextStatePerGroup_size[1])) {
            if (!nextStatePerGroup_data[VdebugVec_size_idx_0_tmp - 1]) {
              showVdiff = false;
              exitg2 = true;
            } else {
              VdebugVec_size_idx_0_tmp++;
            }
          }

          guard5 = false;
          if (showVdiff || ((V_dis_all < prm->ins.prm.jun.minVjuntekInput) &&
                            (prm->seq.VminDisFlag[state_k] != 0.0F))) {
            guard5 = true;
          } else {
            i = b_x->size[0] * b_x->size[1];
            b_x->size[0] = 1;
            b_x->size[1] = SelDual->size[1];
            emxEnsureCapacity_boolean_T(b_x, i);
            b_changeConfigFlag_data = b_x->data;
            loop_ub = SelDual->size[1];
            for (i = 0; i < loop_ub; i++) {
              b_changeConfigFlag_data[i] = (SelDual_data[i] ==
                prm->brd.spi.disconnect);
            }

            showVdiff = true;
            VdebugVec_size_idx_0_tmp = 1;
            exitg2 = false;
            while ((!exitg2) && (VdebugVec_size_idx_0_tmp <= b_x->size[1])) {
              if (!b_changeConfigFlag_data[VdebugVec_size_idx_0_tmp - 1]) {
                showVdiff = false;
                exitg2 = true;
              } else {
                VdebugVec_size_idx_0_tmp++;
              }
            }

            if (showVdiff) {
              guard5 = true;
            }
          }

          if (guard5) {
            state_k++;
            newState = true;
          }

          if (state_k + 1 > prm->seq.Nst) {
            /* size(seq.mod,2) */
            keepMeas = false;
          } else if (newState) {
            for (id2Bypass_size = 0; id2Bypass_size < b_size; id2Bypass_size++)
            {
              for (b_i = 0; b_i <= end_tmp; b_i++) {
                if (b_data[id2Bypass_size] == prm->ser.com.grp[tmp_data[b_i]]) {
                  SelDual_data[b_i] = prm->seq.chr[id2Bypass_size + (state_k <<
                    3)];
                }
              }

              for (b_i = 0; b_i <= end_tmp; b_i++) {
                if (b_data[id2Bypass_size] == prm->ser.com.grp[tmp_data[b_i]]) {
                  switchFlag_data[b_i] = true;
                }
              }
            }
          }

          /*  decision - End */
          k_t++;
          k_t0++;
          while (!(toc() - toc00 >= prm->run.dt)) {
          }

          keepMeas = (((errI2C_size[0] == 0) || (errI2C_size[1] == 0)) &&
                      keepMeas);
          if (k_t == prm->run.Nt) {
            keepMeas = false;
          } else {
            VbusTest_size = tV1->size[0];
            loop_ub = tV1->size[0];
            for (i = 0; i < loop_ub; i++) {
              c_VbusTest_data[i] = (Iacs758_cal_data[i + tV1->size[0] * k_t0] >
                                    prm->run.MaxTime);
            }

            if (ifWhileCond(c_VbusTest_data, VbusTest_size)) {
              keepMeas = false;
            }
          }

          if (k_t0 + 1 == N_bitCnfg) {
            t2_data = datetime_datetime();
            datetime_datevec(t2_data, dateVec);
            d = rt_roundd_snf(dateVec[0]);
            if (d < 2.147483648E+9) {
              if (d >= -2.147483648E+9) {
                i = (int)d;
              } else {
                i = MIN_int32_T;
              }
            } else if (d >= 2.147483648E+9) {
              i = MAX_int32_T;
            } else {
              i = 0;
            }

            d = rt_roundd_snf(dateVec[1]);
            if (d < 2.147483648E+9) {
              if (d >= -2.147483648E+9) {
                i1 = (int)d;
              } else {
                i1 = MIN_int32_T;
              }
            } else if (d >= 2.147483648E+9) {
              i1 = MAX_int32_T;
            } else {
              i1 = 0;
            }

            d = rt_roundd_snf(dateVec[2]);
            if (d < 2.147483648E+9) {
              if (d >= -2.147483648E+9) {
                i2 = (int)d;
              } else {
                i2 = MIN_int32_T;
              }
            } else if (d >= 2.147483648E+9) {
              i2 = MAX_int32_T;
            } else {
              i2 = 0;
            }

            d = rt_roundd_snf(dateVec[3]);
            if (d < 2.147483648E+9) {
              if (d >= -2.147483648E+9) {
                i3 = (int)d;
              } else {
                i3 = MIN_int32_T;
              }
            } else if (d >= 2.147483648E+9) {
              i3 = MAX_int32_T;
            } else {
              i3 = 0;
            }

            d = rt_roundd_snf(dateVec[4]);
            if (d < 2.147483648E+9) {
              if (d >= -2.147483648E+9) {
                c_k_Ttest = (int)d;
              } else {
                c_k_Ttest = MIN_int32_T;
              }
            } else if (d >= 2.147483648E+9) {
              c_k_Ttest = MAX_int32_T;
            } else {
              c_k_Ttest = 0;
            }

            d = rt_roundd_snf(dateVec[5]);
            if (d < 2.147483648E+9) {
              if (d >= -2.147483648E+9) {
                id2Bypass_size = (int)d;
              } else {
                id2Bypass_size = MIN_int32_T;
              }
            } else if (d >= 2.147483648E+9) {
              id2Bypass_size = MAX_int32_T;
            } else {
              id2Bypass_size = 0;
            }

            m_sprintf(i, i1, i2, i3, c_k_Ttest, id2Bypass_size, r30);

            /* char(dateStr); */
            i = (int)(Nina219_tmp_tmp * (double)N_bat_tmp_tmp_tmp);
            i1 = b_Vbat->size[0] * b_Vbat->size[1];
            b_Vbat->size[0] = i;
            b_Vbat->size[1] = N_bitCnfg;
            emxEnsureCapacity_int8_T(b_Vbat, i1);
            Vbat_data = b_Vbat->data;
            b_loop_ub_tmp = i * N_bitCnfg;
            for (i1 = 0; i1 < b_loop_ub_tmp; i1++) {
              Vbat_data[i1] = 0;
            }

            i1 = Vbat->size[0] * Vbat->size[1];
            Vbat->size[0] = b_Vbat->size[0];
            Vbat->size[1] = b_Vbat->size[1];
            emxEnsureCapacity_real_T(Vbat, i1);
            RtotHelpPoly_data = Vbat->data;
            for (i1 = 0; i1 < b_loop_ub_tmp; i1++) {
              RtotHelpPoly_data[i1] = 0.0;
            }

            i1 = IbatMat->size[0] * IbatMat->size[1] * IbatMat->size[2];
            IbatMat->size[0] = (int)Nina219_tmp_tmp;
            IbatMat->size[1] = N_bat_tmp_tmp_tmp;
            IbatMat->size[2] = N_bitCnfg;
            emxEnsureCapacity_real_T(IbatMat, i1);
            IbatMat_data = IbatMat->data;
            k_t0 = loop_ub_tmp * N_bitCnfg;
            i1 = VbatMat->size[0] * VbatMat->size[1] * VbatMat->size[2];
            VbatMat->size[0] = (int)Nina219_tmp_tmp;
            VbatMat->size[1] = N_bat_tmp_tmp_tmp;
            VbatMat->size[2] = N_bitCnfg;
            emxEnsureCapacity_real_T(VbatMat, i1);
            VbusTest2_data = VbatMat->data;
            i1 = VbatMat0->size[0] * VbatMat0->size[1] * VbatMat0->size[2];
            VbatMat0->size[0] = (int)Nina219_tmp_tmp;
            VbatMat0->size[1] = N_bat_tmp_tmp_tmp;
            VbatMat0->size[2] = N_bitCnfg;
            emxEnsureCapacity_real_T(VbatMat0, i1);
            ImKp184Test2_data = VbatMat0->data;
            for (i1 = 0; i1 < k_t0; i1++) {
              IbatMat_data[i1] = 0.0;
              VbusTest2_data[i1] = 0.0;
              ImKp184Test2_data[i1] = 0.0;
            }

            i1 = b_Vbat->size[0] * b_Vbat->size[1];
            b_Vbat->size[0] = i;
            b_Vbat->size[1] = N_bitCnfg;
            emxEnsureCapacity_int8_T(b_Vbat, i1);
            Vbat_data = b_Vbat->data;
            for (i = 0; i < b_loop_ub_tmp; i++) {
              Vbat_data[i] = 0;
            }

            i = tV1->size[0] * tV1->size[1];
            tV1->size[0] = b_Vbat->size[0];
            tV1->size[1] = b_Vbat->size[1];
            emxEnsureCapacity_real_T(tV1, i);
            Iacs758_cal_data = tV1->data;
            for (i = 0; i < b_loop_ub_tmp; i++) {
              Iacs758_cal_data[i] = 0.0;
            }

            i = tV->size[0] * tV->size[1] * tV->size[2];
            tV->size[0] = (int)Nina219_tmp_tmp;
            tV->size[1] = N_bat_tmp_tmp_tmp;
            tV->size[2] = N_bitCnfg;
            emxEnsureCapacity_real_T(tV, i);
            tV_data = tV->data;
            for (i = 0; i < k_t0; i++) {
              tV_data[i] = 0.0;
            }

            /*  Ishunt = zeros(Nina219,N_bat,Nt); */
            k_t0 = 0;
          }

          /*      catch ME */
          /*          keepMeas = 0; */
          /*          disp(['Error:',ME.message]) */
          /*          disp(ME.stack) */
          /*      end */
        }
      }

      /*  End function */
      i = (int)prm->brd.Nina219;
      for (b_k_tstState = 0; b_k_tstState < i; b_k_tstState++) {
        switch (ProjectFlag) {
         case 2:
          /* esp32 */
          break;

         case 3:
          /* esp32 ser */
          /* UNTITLED Summary of this function goes here */
          /*    Detailed explanation goes here */
          /*  if coder.target('MATLAB') */
          /*  read current */
          /*   chip1     |   chip2     |   chip3 */
          /* %comN2 Port4 chip1,comN1 Port11 chip1 */
          /* %comP1 Port18 chip1 ,comP2 Port18 chip3 */
          /*  */
          /*  for k_row = 1:N_row */
          /*      out123(k_row,:) = readSPI(SPI,PortSpiRow(k_row,1),N_chip); */
          /*      oldPortSpiRow(k_row,1:2:end) = PortSpiRow(k_row,1:2:end); */
          /*      oldPortSpiRow(k_row,2:2:end) = out123(k_row,2:2:end); */
          /*  end */
          /* mask current */
          /* write current mask */
          /*  for k_row = 1:N_row */
          /*      write(SPI,oldPortSpiRowMask(k_row,:)); */
          /*  end */
          pause(0.001);

          /* mask new */
          /* write mask new */
          /*  for k_row = 1:N_row */
          /*      write(SPI0,PortSpiRowMask(k_row,:)); */
          /*  end */
          /* write new */
          /*  for k_row = 1:N_row */
          /*      write(SPI0,PortSpiRow(k_row,:)); */
          /*  end */
          /*  Rmsg = SPI_writeread(SPI0, PortSpiRow); */
          /*  ReadPortSpiRow = RmsgData([[2:4] , 1] ,:); */
          /*  ReadPortSpiRow(:,1:2:end) = ReadPortSpiRow(:,1:2:end);% - 128; */
          /* mask current */
          /* %chip1 Port6 Alert1, chip1 port 7 Alert3, chip1 port 8 Alert8, chip1 port 9 Alert6 */
          /* %chip1 Port12 Alert7, chip1 Port13 Alert5, chip 1 port19 NC */
          /* %chip1 Port20-23 NC */
          /* chip1 Port28 Alert1, chip1 Port29 Alert2 */
          /*  if sum(sum(sum(abs(double(PortSpiRow)-double(ReadPortSpiRow)))))>0 */
          /*  end */
          break;
        }
      }

      t2_data = datetime_datetime();
      datetime_datevec(t2_data, dateVec);
      d = rt_roundd_snf(dateVec[0]);
      if (d < 2.147483648E+9) {
        if (d >= -2.147483648E+9) {
          i = (int)d;
        } else {
          i = MIN_int32_T;
        }
      } else if (d >= 2.147483648E+9) {
        i = MAX_int32_T;
      } else {
        i = 0;
      }

      d = rt_roundd_snf(dateVec[1]);
      if (d < 2.147483648E+9) {
        if (d >= -2.147483648E+9) {
          i1 = (int)d;
        } else {
          i1 = MIN_int32_T;
        }
      } else if (d >= 2.147483648E+9) {
        i1 = MAX_int32_T;
      } else {
        i1 = 0;
      }

      d = rt_roundd_snf(dateVec[2]);
      if (d < 2.147483648E+9) {
        if (d >= -2.147483648E+9) {
          i2 = (int)d;
        } else {
          i2 = MIN_int32_T;
        }
      } else if (d >= 2.147483648E+9) {
        i2 = MAX_int32_T;
      } else {
        i2 = 0;
      }

      d = rt_roundd_snf(dateVec[3]);
      if (d < 2.147483648E+9) {
        if (d >= -2.147483648E+9) {
          i3 = (int)d;
        } else {
          i3 = MIN_int32_T;
        }
      } else if (d >= 2.147483648E+9) {
        i3 = MAX_int32_T;
      } else {
        i3 = 0;
      }

      d = rt_roundd_snf(dateVec[4]);
      if (d < 2.147483648E+9) {
        if (d >= -2.147483648E+9) {
          c_k_Ttest = (int)d;
        } else {
          c_k_Ttest = MIN_int32_T;
        }
      } else if (d >= 2.147483648E+9) {
        c_k_Ttest = MAX_int32_T;
      } else {
        c_k_Ttest = 0;
      }

      d = rt_roundd_snf(dateVec[5]);
      if (d < 2.147483648E+9) {
        if (d >= -2.147483648E+9) {
          id2Bypass_size = (int)d;
        } else {
          id2Bypass_size = MIN_int32_T;
        }
      } else if (d >= 2.147483648E+9) {
        id2Bypass_size = MAX_int32_T;
      } else {
        id2Bypass_size = 0;
      }

      m_sprintf(i, i1, i2, i3, c_k_Ttest, id2Bypass_size, r19);

      /* char(dateStr); */
      /*  ind0 = strfind((dateStr),':'); */
      /*  dateStr(ind0) = '-'; */
    }
  } else {
    /* CalibrateBaordFlag */
  }

  emxFree_char_T(&r30);
  emxFree_char_T(&d_Vcharge0);
  emxFree_char_T(&r29);
  emxFree_char_T(&r28);
  emxFree_char_T(&c_Vcharge0);
  emxFree_char_T(&r27);
  emxFree_char_T(&f_IchargeAct);
  emxFree_char_T(&r26);
  emxFree_char_T(&e_IchargeAct);
  emxFree_char_T(&d_IchargeAct);
  emxFree_char_T(&c_IchargeAct);
  emxFree_char_T(&r25);
  emxFree_char_T(&r24);
  emxFree_char_T(&r23);
  emxFree_char_T(&r22);
  emxFree_char_T(&r21);
  emxFree_char_T(&r20);
  emxFree_char_T(&b_IchargeAct);
  emxFree_char_T(&IchargeAct);
  emxFree_char_T(&b_Vcharge0);
  emxFree_char_T(&Vcharge0);
  emxFree_char_T(&r19);
  emxFree_char_T(&r18);
  emxFree_char_T(&dv0);
  emxFree_char_T(&r17);
  emxFree_char_T(&r16);
  emxFree_char_T(&r15);
  emxFree_char_T(&meanIbrd0);
  emxFree_char_T(&r14);
  emxFree_char_T(&r13);
  emxFree_char_T(&d_prm);
  emxFree_char_T(&b_vSumMax);
  emxFree_char_T(&b_IshuntTest2);
  emxFree_char_T(&b_k_Ttest);
  emxFree_char_T(&c_VbusTest2);
  emxFree_char_T(&k_Ttest);
  emxFree_char_T(&r12);
  emxFree_char_T(&c_prm);
  emxFree_char_T(&r11);
  emxFree_char_T(&r10);
  emxFree_char_T(&r9);
  emxFree_char_T(&b_prm);
  emxFree_char_T(&r8);
  emxFree_char_T(&vSumMax);
  emxFree_char_T(&b_VmKp184Test_data);
  emxFree_char_T(&r7);
  emxFree_char_T(&b_k_bat);
  emxFree_char_T(&b_VbusTest_data);
  emxFree_char_T(&k_bat);
  emxFree_char_T(&r6);
  emxFree_char_T(&r5);
  emxFree_real_T(&r4);
  emxFree_real_T(&b_BattConfigPerInaHelp);
  emxFree_boolean_T(&b_changeConfigFlag);
  emxFree_real32_T(&b_Rwire);
  emxFree_real_T(&b_ImKp184Test2_bat_0);
  emxFree_uint8_T(&b_bitCnfg);
  emxFree_real_T(&b_VbusTest2);
  emxFree_boolean_T(&b_x);
  emxFree_real_T(&x);
  emxFree_uint16_T(&y);
  emxFree_int32_T(&r3);
  emxFree_real_T(&b_I);
  emxFree_int8_T(&b_Vbat);
  emxFree_real_T(&r2);
  emxFree_real_T(&VecIna219);
  emxFree_uint8_T(&Pac2Vid31);
  emxFree_real_T(&VbusTest0);
  emxFree_real_T(&tvv);
  emxFree_real_T(&r1);
  emxFree_real_T(&r);
  emxFree_uint8_T(&bitCnfg);
  emxFree_real_T(&SelDual_nm1);
  emxFree_real_T(&BattConfigPerInaHelp);
  emxFree_real_T(&a__73);
  emxFree_real_T(&a__72);
  emxFree_real_T(&a__71);
  emxFree_real_T(&a__70);
  emxFree_real_T(&a__69);
  emxFree_real_T(&a__68);
  emxFree_real32_T(&a__67);
  emxFree_real_T(&a__66);
  emxFree_real_T(&a__65);
  emxFree_real32_T(&a__64);
  emxFree_real_T(&BattConfigPerIna);
  emxFree_boolean_T(&changeConfigFlag);
  emxFree_boolean_T(&switchFlagHelp);
  emxFree_boolean_T(&switchFlag);
  emxFree_real_T(&tLastToggle);
  emxFree_real_T(&SelDualHelp);
  emxFree_real_T(&SelDual);
  emxFree_real_T(&tV);
  emxFree_real_T(&tV1);
  emxFree_real_T(&VbatMat0);
  emxFree_real_T(&VbatMat);
  emxFree_real_T(&IbatMat);
  emxFree_real_T(&Vbat);
  emxFree_real_T(&a__45);
  emxFree_real_T(&a__42);
  emxFree_real_T(&a__40);
  emxFree_real_T(&a__39);
  emxFree_real_T(&a__38);
  emxFree_real32_T(&a__37);
  emxFree_real32_T(&a__36);
  emxFree_real_T(&VbitInsMeas);
  emxFree_real_T(&meanIbrd);
  emxFree_real_T(&Ibrd);
  emxFree_real_T(&Vbrd);
  emxFree_uint8_T(&Pac2Vid0All);
  emxFree_real_T(&ImKp184Test2_bat_0);
  emxFree_real_T(&ImKp184Test2_0);
  emxFree_real_T(&Iacs758_cal0);
  emxFree_real_T(&RtotHelpPoly);
  emxFree_real_T(&pIshunt);
  emxFree_real_T(&Rtot);
  emxFree_real_T(&Iacs758_cal);
  emxFree_real_T(&ImKp184Test2);
  emxFree_real_T(&IshuntTest2);
  emxFree_real_T(&VbusTest2);
  emxFree_real_T(&Vdebug1N);
  emxFree_real_T(&Vdebug);
  emxFree_real_T(&VmV);
  emxFree_boolean_T(&BattConfigAct);
  emxFree_real32_T(&Rwire);
  emxFree_real32_T(&pIacs758);
}

/*
 * File trailer for Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229.c
 *
 * [EOF]
 */
