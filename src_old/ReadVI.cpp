/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: ReadVI.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 31-Mar-2024 07:04:52
 */

/* Include Files */
#include "ReadVI.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_emxutil.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h"
#include "randn.h"
#include "readI2cVIfastTicTocV5_Rratio_ser.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static void binary_expand_op_5(emxArray_real32_T *in1,
                               const emxArray_real_T *in2,
                               const emxArray_real32_T *in3,
                               const emxArray_real_T *in4);

static void binary_expand_op_6(emxArray_real32_T *in1,
                               const emxArray_real_T *in2,
                               const emxArray_real32_T *in3,
                               const emxArray_real_T *in4);

static void binary_expand_op_7(emxArray_real32_T *in1,
                               const emxArray_real_T *in2,
                               const emxArray_real32_T *in3,
                               const emxArray_real_T *in4);

static void binary_expand_op_8(emxArray_real32_T *in1,
                               const emxArray_real_T *in2,
                               const emxArray_real32_T *in3,
                               const emxArray_real_T *in4);

/* Function Definitions */
/*
 * Arguments    : emxArray_real32_T *in1
 *                const emxArray_real_T *in2
 *                const emxArray_real32_T *in3
 *                const emxArray_real_T *in4
 * Return Type  : void
 */
static void binary_expand_op_5(emxArray_real32_T *in1,
                               const emxArray_real_T *in2,
                               const emxArray_real32_T *in3,
                               const emxArray_real_T *in4)
{
  const double *in2_data;
  const double *in4_data;
  const float *in3_data;
  float *in1_data;
  int i;
  int loop_ub;
  int stride_0_1;
  int stride_1_1;
  int stride_2_1;
  in4_data = in4->data;
  in3_data = in3->data;
  in2_data = in2->data;
  i = in1->size[0] * in1->size[1];
  in1->size[0] = 1;
  emxEnsureCapacity_real32_T(in1, i);
  if (in4->size[1] == 1) {
    i = in3->size[1];
  } else {
    i = in4->size[1];
  }
  if (i == 1) {
    loop_ub = in2->size[1];
  } else {
    loop_ub = i;
  }
  i = in1->size[0] * in1->size[1];
  in1->size[1] = loop_ub;
  emxEnsureCapacity_real32_T(in1, i);
  in1_data = in1->data;
  stride_0_1 = (in2->size[1] != 1);
  stride_1_1 = (in3->size[1] != 1);
  stride_2_1 = (in4->size[1] != 1);
  for (i = 0; i < loop_ub; i++) {
    in1_data[i] = (float)in2_data[i * stride_0_1] +
                  in3_data[i * stride_1_1] * (float)in4_data[i * stride_2_1];
  }
}

/*
 * Arguments    : emxArray_real32_T *in1
 *                const emxArray_real_T *in2
 *                const emxArray_real32_T *in3
 *                const emxArray_real_T *in4
 * Return Type  : void
 */
static void binary_expand_op_6(emxArray_real32_T *in1,
                               const emxArray_real_T *in2,
                               const emxArray_real32_T *in3,
                               const emxArray_real_T *in4)
{
  const double *in2_data;
  const double *in4_data;
  const float *in3_data;
  float *in1_data;
  int i;
  int loop_ub;
  int stride_0_0;
  int stride_1_0;
  int stride_2_0;
  in4_data = in4->data;
  in3_data = in3->data;
  in2_data = in2->data;
  if (in4->size[0] == 1) {
    i = in3->size[1];
  } else {
    i = in4->size[0];
  }
  if (i == 1) {
    loop_ub = in2->size[0];
  } else {
    loop_ub = i;
  }
  i = in1->size[0];
  in1->size[0] = loop_ub;
  emxEnsureCapacity_real32_T(in1, i);
  in1_data = in1->data;
  stride_0_0 = (in2->size[0] != 1);
  stride_1_0 = (in3->size[1] != 1);
  stride_2_0 = (in4->size[0] != 1);
  for (i = 0; i < loop_ub; i++) {
    in1_data[i] = (float)in2_data[i * stride_0_0] +
                  in3_data[i * stride_1_0] * (float)in4_data[i * stride_2_0];
  }
}

/*
 * Arguments    : emxArray_real32_T *in1
 *                const emxArray_real_T *in2
 *                const emxArray_real32_T *in3
 *                const emxArray_real_T *in4
 * Return Type  : void
 */
static void binary_expand_op_7(emxArray_real32_T *in1,
                               const emxArray_real_T *in2,
                               const emxArray_real32_T *in3,
                               const emxArray_real_T *in4)
{
  const double *in2_data;
  const double *in4_data;
  const float *in3_data;
  float *in1_data;
  int aux_0_1;
  int aux_1_1;
  int aux_2_1;
  int b_loop_ub;
  int i;
  int i1;
  int loop_ub;
  int stride_0_0;
  int stride_0_1;
  int stride_1_0;
  int stride_1_1;
  int stride_2_0;
  int stride_2_1;
  in4_data = in4->data;
  in3_data = in3->data;
  in2_data = in2->data;
  if (in4->size[0] == 1) {
    i = in3->size[0];
  } else {
    i = in4->size[0];
  }
  if (i == 1) {
    loop_ub = in2->size[0];
  } else {
    loop_ub = i;
  }
  i = in1->size[0] * in1->size[1];
  in1->size[0] = loop_ub;
  emxEnsureCapacity_real32_T(in1, i);
  if (in4->size[1] == 1) {
    i = in3->size[1];
  } else {
    i = in4->size[1];
  }
  if (i == 1) {
    b_loop_ub = in2->size[1];
  } else {
    b_loop_ub = i;
  }
  i = in1->size[0] * in1->size[1];
  in1->size[1] = b_loop_ub;
  emxEnsureCapacity_real32_T(in1, i);
  in1_data = in1->data;
  stride_0_0 = (in2->size[0] != 1);
  stride_0_1 = (in2->size[1] != 1);
  stride_1_0 = (in3->size[0] != 1);
  stride_1_1 = (in3->size[1] != 1);
  stride_2_0 = (in4->size[0] != 1);
  stride_2_1 = (in4->size[1] != 1);
  aux_0_1 = 0;
  aux_1_1 = 0;
  aux_2_1 = 0;
  for (i = 0; i < b_loop_ub; i++) {
    for (i1 = 0; i1 < loop_ub; i1++) {
      in1_data[i1 + in1->size[0] * i] =
          (float)in2_data[i1 * stride_0_0 + in2->size[0] * aux_0_1] +
          in3_data[i1 * stride_1_0 + in3->size[0] * aux_1_1] *
              (float)in4_data[i1 * stride_2_0 + in4->size[0] * aux_2_1];
    }
    aux_2_1 += stride_2_1;
    aux_1_1 += stride_1_1;
    aux_0_1 += stride_0_1;
  }
}

/*
 * Arguments    : emxArray_real32_T *in1
 *                const emxArray_real_T *in2
 *                const emxArray_real32_T *in3
 *                const emxArray_real_T *in4
 * Return Type  : void
 */
static void binary_expand_op_8(emxArray_real32_T *in1,
                               const emxArray_real_T *in2,
                               const emxArray_real32_T *in3,
                               const emxArray_real_T *in4)
{
  const double *in2_data;
  const double *in4_data;
  const float *in3_data;
  float *in1_data;
  int b_in2;
  int b_in3;
  int b_in4;
  int i;
  int loop_ub;
  int stride_0_0;
  in4_data = in4->data;
  in3_data = in3->data;
  in2_data = in2->data;
  b_in2 = in2->size[0] * in2->size[1];
  b_in3 = in3->size[0] * in3->size[1];
  b_in4 = in4->size[0] * in4->size[1];
  if (b_in4 == 1) {
    i = b_in3;
  } else {
    i = b_in4;
  }
  if (i == 1) {
    loop_ub = b_in2;
  } else {
    loop_ub = i;
  }
  i = in1->size[0];
  in1->size[0] = loop_ub;
  emxEnsureCapacity_real32_T(in1, i);
  in1_data = in1->data;
  stride_0_0 = (b_in2 != 1);
  b_in3 = (b_in3 != 1);
  b_in2 = (b_in4 != 1);
  for (i = 0; i < loop_ub; i++) {
    in1_data[i] = (float)in2_data[i * stride_0_0] +
                  in3_data[i * b_in3] * (float)in4_data[i * b_in2];
  }
}

/*
 * read V,I
 *  Vbat   = zeros(Nina219*N_bat);
 *
 * Arguments    : signed char ProjectFlag
 *                const signed char VIpacId[16]
 *                const emxArray_uint8_T *Pac2Vid0All
 *                short N_bat1
 *                double N_bat
 *                const float pIacs758[2]
 *                signed char Iacs758Flag
 *                const emxArray_real_T *pIshunt
 *                const emxArray_real32_T *Rwire
 *                emxArray_real32_T *Vbat
 *                emxArray_real_T *Vbat0
 *                emxArray_real_T *Ibat
 *                emxArray_real32_T *VbatMat
 *                emxArray_real_T *VbatMat0
 *                emxArray_real_T *Ishunt
 *                emxArray_real_T *tV
 *                emxArray_real_T *tV1
 *                emxArray_real_T *tI
 *                emxArray_real_T *tI1
 *                emxArray_real_T *Vdebug0
 *                double errI2C_data[]
 *                int errI2C_size[2]
 *                boolean_T *keepMeas
 * Return Type  : double
 */
double ReadVI(signed char ProjectFlag, const signed char VIpacId[16],
              const emxArray_uint8_T *Pac2Vid0All, short N_bat1, double N_bat,
              const float pIacs758[2], signed char Iacs758Flag,
              const emxArray_real_T *pIshunt, const emxArray_real32_T *Rwire,
              emxArray_real32_T *Vbat, emxArray_real_T *Vbat0,
              emxArray_real_T *Ibat, emxArray_real32_T *VbatMat,
              emxArray_real_T *VbatMat0, emxArray_real_T *Ishunt,
              emxArray_real_T *tV, emxArray_real_T *tV1, emxArray_real_T *tI,
              emxArray_real_T *tI1, emxArray_real_T *Vdebug0,
              double errI2C_data[], int errI2C_size[2], boolean_T *keepMeas)
{
  static short c_tmp_data[32767];
  emxArray_real_T *V;
  emxArray_real_T *b_I;
  emxArray_real_T *r;
  emxArray_real_T *tii;
  emxArray_real_T *tvv;
  double b_tii[16];
  double b_tvv[16];
  const double *pIshunt_data;
  double Iacs758;
  double b;
  double *I_data;
  double *Ishunt_data;
  double *V_data;
  double *VbatMat0_data;
  double *Vdebug0_data;
  double *r1;
  double *tI_data;
  double *tV_data;
  double *tii_data;
  double *tvv_data;
  const float *Rwire_data;
  float *Vbat_data;
  int I_idx_0;
  int i;
  int loop_ub;
  int loop_ub_tmp;
  int trueCount;
  unsigned char b_tmp_data[32767];
  const unsigned char *Pac2Vid0All_data;
  boolean_T tmp_data[32767];
  boolean_T b_b;
  Rwire_data = Rwire->data;
  pIshunt_data = pIshunt->data;
  Pac2Vid0All_data = Pac2Vid0All->data;
  *keepMeas = true;
  i = Vdebug0->size[0] * Vdebug0->size[1] * Vdebug0->size[2];
  Vdebug0->size[0] = 1;
  loop_ub_tmp = (int)N_bat;
  Vdebug0->size[1] = (int)N_bat;
  Vdebug0->size[2] = 2;
  emxEnsureCapacity_real_T(Vdebug0, i);
  Vdebug0_data = Vdebug0->data;
  loop_ub = (int)N_bat << 1;
  for (i = 0; i < loop_ub; i++) {
    Vdebug0_data[i] = 0.0;
  }
  i = VbatMat0->size[0] * VbatMat0->size[1];
  VbatMat0->size[0] = 1;
  VbatMat0->size[1] = (int)N_bat;
  emxEnsureCapacity_real_T(VbatMat0, i);
  VbatMat0_data = VbatMat0->data;
  for (i = 0; i < loop_ub_tmp; i++) {
    VbatMat0_data[i] = 0.0;
  }
  /*  Ibat   = zeros(Nina219*N_bat); */
  /*  tV1    = zeros(Nina219*N_bat); */
  /*  tI1    = zeros(Nina219*N_bat); */
  i = tV->size[0] * tV->size[1];
  tV->size[0] = 1;
  tV->size[1] = (int)N_bat;
  emxEnsureCapacity_real_T(tV, i);
  tV_data = tV->data;
  for (i = 0; i < loop_ub_tmp; i++) {
    tV_data[i] = 0.0;
  }
  i = tI->size[0] * tI->size[1];
  tI->size[0] = 1;
  tI->size[1] = (int)N_bat;
  emxEnsureCapacity_real_T(tI, i);
  tI_data = tI->data;
  for (i = 0; i < loop_ub_tmp; i++) {
    tI_data[i] = 0.0;
  }
  i = Ishunt->size[0] * Ishunt->size[1];
  Ishunt->size[0] = 1;
  Ishunt->size[1] = (int)N_bat;
  emxEnsureCapacity_real_T(Ishunt, i);
  Ishunt_data = Ishunt->data;
  for (i = 0; i < loop_ub_tmp; i++) {
    Ishunt_data[i] = 0.0;
  }
  emxInit_real_T(&b_I, 1);
  i = b_I->size[0];
  b_I->size[0] = (int)N_bat;
  emxEnsureCapacity_real_T(b_I, i);
  I_data = b_I->data;
  for (i = 0; i < loop_ub_tmp; i++) {
    I_data[i] = 0.0;
  }
  emxInit_real_T(&V, 1);
  i = V->size[0];
  V->size[0] = (int)N_bat;
  emxEnsureCapacity_real_T(V, i);
  V_data = V->data;
  for (i = 0; i < loop_ub_tmp; i++) {
    V_data[i] = 0.0;
  }
  emxInit_real_T(&tvv, 1);
  i = tvv->size[0];
  tvv->size[0] = (int)N_bat;
  emxEnsureCapacity_real_T(tvv, i);
  tvv_data = tvv->data;
  for (i = 0; i < loop_ub_tmp; i++) {
    tvv_data[i] = 0.0;
  }
  emxInit_real_T(&tii, 1);
  i = tii->size[0];
  tii->size[0] = (int)N_bat;
  emxEnsureCapacity_real_T(tii, i);
  tii_data = tii->data;
  for (i = 0; i < loop_ub_tmp; i++) {
    tii_data[i] = 0.0;
  }
  Iacs758 = 0.0;
  errI2C_size[0] = 0;
  errI2C_size[1] = 0;
  switch (ProjectFlag) {
  case 2:
    /* ESP32 */
    /*                  toc0 = toc; */
    i = V->size[0];
    V->size[0] = 16;
    emxEnsureCapacity_real_T(V, i);
    V_data = V->data;
    i = b_I->size[0];
    b_I->size[0] = 16;
    emxEnsureCapacity_real_T(b_I, i);
    I_data = b_I->data;
    i = tvv->size[0];
    tvv->size[0] = 16;
    emxEnsureCapacity_real_T(tvv, i);
    tvv_data = tvv->data;
    i = tii->size[0];
    tii->size[0] = 16;
    emxEnsureCapacity_real_T(tii, i);
    tii_data = tii->data;
    for (i = 0; i < 16; i++) {
      V_data[i] = 0.0;
      I_data[i] = 0.0;
      tvv_data[i] = 0.0;
      tii_data[i] = 0.0;
    }
    errI2C_size[0] = 1;
    errI2C_size[1] = 1;
    errI2C_data[0] = -1.0;
    if (N_bat < 1.0) {
      loop_ub = 0;
    } else {
      loop_ub = (int)N_bat;
    }
    for (i = 0; i < 2; i++) {
      for (I_idx_0 = 0; I_idx_0 < loop_ub; I_idx_0++) {
        Vdebug0_data[I_idx_0 + Vdebug0->size[1] * i] = 0.0;
      }
    }
    break;
  case 3:
    /* ESP32 ser */
    /*                  toc0 = toc; */
    emxInit_real_T(&r, 2);
    d_readI2cVIfastTicTocV5_Rratio_(VIpacId, N_bat1, V, b_I, b_tvv, b_tii,
                                    errI2C_data, errI2C_size, r);
    r1 = r->data;
    I_data = b_I->data;
    V_data = V->data;
    i = tvv->size[0];
    tvv->size[0] = 16;
    emxEnsureCapacity_real_T(tvv, i);
    tvv_data = tvv->data;
    i = tii->size[0];
    tii->size[0] = 16;
    emxEnsureCapacity_real_T(tii, i);
    tii_data = tii->data;
    for (i = 0; i < 16; i++) {
      tvv_data[i] = b_tvv[i];
      tii_data[i] = b_tii[i];
    }
    if (N_bat < 1.0) {
      loop_ub = 0;
    } else {
      loop_ub = (int)N_bat;
    }
    for (i = 0; i < 2; i++) {
      for (I_idx_0 = 0; I_idx_0 < loop_ub; I_idx_0++) {
        Vdebug0_data[I_idx_0 + Vdebug0->size[1] * i] =
            r1[I_idx_0 + loop_ub * i];
      }
    }
    emxFree_real_T(&r);
    break;
  }
  if ((errI2C_size[0] != 0) && (errI2C_size[1] != 0)) {
    *keepMeas = false;
  } else {
    if (Iacs758Flag == 1) {
      Iacs758 = pIacs758[0] * 0.0F + pIacs758[1];
      I_idx_0 = b_I->size[0];
      i = b_I->size[0];
      b_I->size[0] = I_idx_0;
      emxEnsureCapacity_real_T(b_I, i);
      I_data = b_I->data;
      for (i = 0; i < I_idx_0; i++) {
        I_data[i] = 0.0;
      }
      /* Pac2Vid0(:,1); */
      I_idx_0 = Pac2Vid0All->size[1];
      loop_ub = Pac2Vid0All->size[1];
      for (i = 0; i < loop_ub; i++) {
        tmp_data[i] = (Pac2Vid0All_data[i] > 0);
      }
      I_idx_0--;
      trueCount = 0;
      loop_ub = 0;
      for (i = 0; i <= I_idx_0; i++) {
        if (tmp_data[i]) {
          trueCount++;
          c_tmp_data[loop_ub] = (short)i;
          loop_ub++;
        }
      }
      for (i = 0; i < trueCount; i++) {
        b_tmp_data[i] = Pac2Vid0All_data[c_tmp_data[i]];
      }
      for (i = 0; i < trueCount; i++) {
        I_data[b_tmp_data[i] - 1] = Iacs758;
      }
    } else if (Iacs758Flag == 2) {
      b = 0.0;
      switch (ProjectFlag) {
      case 2:
        /* ESP32 */
        break;
      case 3:
        /* ESP32 ser */
        /* UNTITLED3 Summary of this function goes here */
        /*    Detailed explanation goes here */
        b = randn() * 5.0;
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
        break;
      }
      Iacs758 = pIacs758[0] * (float)b + pIacs758[1];
      I_idx_0 = b_I->size[0];
      i = b_I->size[0];
      b_I->size[0] = I_idx_0;
      emxEnsureCapacity_real_T(b_I, i);
      I_data = b_I->data;
      for (i = 0; i < I_idx_0; i++) {
        I_data[i] = 0.0;
      }
      /* Pac2Vid0(:,1); */
      I_idx_0 = Pac2Vid0All->size[1];
      loop_ub = Pac2Vid0All->size[1];
      for (i = 0; i < loop_ub; i++) {
        tmp_data[i] = (Pac2Vid0All_data[i] > 0);
      }
      I_idx_0--;
      trueCount = 0;
      loop_ub = 0;
      for (i = 0; i <= I_idx_0; i++) {
        if (tmp_data[i]) {
          trueCount++;
          c_tmp_data[loop_ub] = (short)i;
          loop_ub++;
        }
      }
      for (i = 0; i < trueCount; i++) {
        b_tmp_data[i] = Pac2Vid0All_data[c_tmp_data[i]];
      }
      for (i = 0; i < trueCount; i++) {
        I_data[b_tmp_data[i] - 1] = Iacs758;
      }
    }
    for (i = 0; i < loop_ub_tmp; i++) {
      VbatMat0_data[i] = V_data[i];
    }
    /*  + squeeze(Voffset(k_ina219,:,:))'; */
    for (I_idx_0 = 0; I_idx_0 < loop_ub_tmp; I_idx_0++) {
      Ishunt_data[I_idx_0] =
          I_data[I_idx_0] * pIshunt_data[pIshunt->size[0] * I_idx_0] +
          pIshunt_data[(I_idx_0 + pIshunt->size[1]) * pIshunt->size[0]];
    }
    for (i = 0; i < loop_ub_tmp; i++) {
      tV_data[i] = tvv_data[i];
    }
    /* (toc0+toc1)/2; */
    for (i = 0; i < loop_ub_tmp; i++) {
      tI_data[i] = tii_data[i];
    }
    /* tV(k_ina219,:,k_t); */
    /*  switch ProjectFlag */
    /*      case 2%ESP32 */
    /*  end */
  }
  emxFree_real_T(&tii);
  emxFree_real_T(&tvv);
  emxFree_real_T(&V);
  emxFree_real_T(&b_I);
  i = Vbat0->size[0];
  Vbat0->size[0] = VbatMat0->size[1];
  emxEnsureCapacity_real_T(Vbat0, i);
  I_data = Vbat0->data;
  loop_ub = VbatMat0->size[1];
  for (i = 0; i < loop_ub; i++) {
    I_data[i] = VbatMat0_data[i];
  }
  i = tV1->size[0];
  tV1->size[0] = tV->size[1];
  emxEnsureCapacity_real_T(tV1, i);
  Vdebug0_data = tV1->data;
  loop_ub = tV->size[1];
  for (i = 0; i < loop_ub; i++) {
    Vdebug0_data[i] = tV_data[i];
  }
  i = tI1->size[0];
  tI1->size[0] = tI->size[1];
  emxEnsureCapacity_real_T(tI1, i);
  Vdebug0_data = tI1->data;
  loop_ub = tI->size[1];
  for (i = 0; i < loop_ub; i++) {
    Vdebug0_data[i] = tI_data[i];
  }
  i = Ibat->size[0];
  Ibat->size[0] = Ishunt->size[1];
  emxEnsureCapacity_real_T(Ibat, i);
  Vdebug0_data = Ibat->data;
  loop_ub = Ishunt->size[1];
  for (i = 0; i < loop_ub; i++) {
    Vdebug0_data[i] = Ishunt_data[i];
  }
  if (Rwire->size[1] == 1) {
    loop_ub_tmp = Ibat->size[0];
  } else {
    loop_ub_tmp = Rwire->size[1];
  }
  b_b = ((Ibat->size[0] == Rwire->size[1]) && (Vbat0->size[0] == loop_ub_tmp));
  if (b_b) {
    i = Vbat->size[0];
    Vbat->size[0] = Vbat0->size[0];
    emxEnsureCapacity_real32_T(Vbat, i);
    Vbat_data = Vbat->data;
    loop_ub = Vbat0->size[0];
    for (i = 0; i < loop_ub; i++) {
      Vbat_data[i] = (float)I_data[i] + Rwire_data[i] * (float)Vdebug0_data[i];
    }
    i = VbatMat->size[0] * VbatMat->size[1];
    VbatMat->size[0] = 1;
    VbatMat->size[1] = VbatMat0->size[1];
    emxEnsureCapacity_real32_T(VbatMat, i);
    Vbat_data = VbatMat->data;
    loop_ub = VbatMat0->size[1];
    for (i = 0; i < loop_ub; i++) {
      Vbat_data[i] =
          (float)VbatMat0_data[i] + Rwire_data[i] * (float)Ishunt_data[i];
    }
  } else {
    binary_expand_op_6(Vbat, Vbat0, Rwire, Ibat);
    binary_expand_op_5(VbatMat, VbatMat0, Rwire, Ishunt);
  }
  return Iacs758;
}

/*
 * read V,I
 *  Vbat   = zeros(Nina219*N_bat);
 *
 * Arguments    : double Nina219
 *                signed char ProjectFlag
 *                const signed char VIpacId[32]
 *                const emxArray_uint8_T *Pac2Vid0All
 *                short N_bat1
 *                double N_bat
 *                const emxArray_real32_T *pIacs758
 *                signed char Iacs758Flag
 *                const emxArray_real_T *pIshunt
 *                const emxArray_real32_T *Rwire
 *                emxArray_real32_T *Vbat
 *                emxArray_real_T *Vbat0
 *                emxArray_real_T *Ibat
 *                emxArray_real32_T *VbatMat
 *                emxArray_real_T *VbatMat0
 *                emxArray_real_T *Ishunt
 *                emxArray_real_T *tV
 *                emxArray_real_T *tV1
 *                emxArray_real_T *tI
 *                emxArray_real_T *tI1
 *                emxArray_real_T *Iacs758
 *                emxArray_real_T *Vdebug0
 *                double errI2C_data[]
 *                int errI2C_size[2]
 * Return Type  : boolean_T
 */
boolean_T
b_ReadVI(double Nina219, signed char ProjectFlag, const signed char VIpacId[32],
         const emxArray_uint8_T *Pac2Vid0All, short N_bat1, double N_bat,
         const emxArray_real32_T *pIacs758, signed char Iacs758Flag,
         const emxArray_real_T *pIshunt, const emxArray_real32_T *Rwire,
         emxArray_real32_T *Vbat, emxArray_real_T *Vbat0, emxArray_real_T *Ibat,
         emxArray_real32_T *VbatMat, emxArray_real_T *VbatMat0,
         emxArray_real_T *Ishunt, emxArray_real_T *tV, emxArray_real_T *tV1,
         emxArray_real_T *tI, emxArray_real_T *tI1, emxArray_real_T *Iacs758,
         emxArray_real_T *Vdebug0, double errI2C_data[], int errI2C_size[2])
{
  static short c_tmp_data[32767];
  static short d_tmp_data[32767];
  emxArray_real32_T *Rwire0;
  emxArray_real_T *V;
  emxArray_real_T *b_I;
  emxArray_real_T *b_Vbat0;
  emxArray_real_T *r;
  emxArray_real_T *tV1mat;
  emxArray_real_T *tii;
  emxArray_real_T *tvv;
  double b_tii[16];
  double b_tvv[16];
  const double *pIshunt_data;
  double b;
  double *I_data;
  double *Iacs758_data;
  double *Ishunt_data;
  double *V_data;
  double *VbatMat0_data;
  double *Vdebug0_data;
  double *r1;
  double *tI_data;
  double *tV_data;
  double *tii_data;
  double *tvv_data;
  const float *Rwire_data;
  const float *pIacs758_data;
  float *Rwire0_data;
  float *Vbat_data;
  int I_idx_0;
  int b_loop_ub_tmp;
  int c_loop_ub_tmp;
  int i;
  int k_ina219;
  int loop_ub;
  int loop_ub_tmp;
  int trueCount;
  unsigned char b_tmp_data[32767];
  signed char b_VIpacId[16];
  const unsigned char *Pac2Vid0All_data;
  boolean_T tmp_data[32767];
  boolean_T keepMeas;
  Rwire_data = Rwire->data;
  pIshunt_data = pIshunt->data;
  pIacs758_data = pIacs758->data;
  Pac2Vid0All_data = Pac2Vid0All->data;
  keepMeas = true;
  loop_ub_tmp = (int)Nina219;
  i = Vdebug0->size[0] * Vdebug0->size[1] * Vdebug0->size[2];
  Vdebug0->size[0] = (int)Nina219;
  b_loop_ub_tmp = (int)N_bat;
  Vdebug0->size[1] = (int)N_bat;
  Vdebug0->size[2] = 2;
  emxEnsureCapacity_real_T(Vdebug0, i);
  Vdebug0_data = Vdebug0->data;
  c_loop_ub_tmp = (int)Nina219 * (int)N_bat;
  loop_ub = c_loop_ub_tmp << 1;
  for (i = 0; i < loop_ub; i++) {
    Vdebug0_data[i] = 0.0;
  }
  i = VbatMat0->size[0] * VbatMat0->size[1];
  VbatMat0->size[0] = (int)Nina219;
  VbatMat0->size[1] = (int)N_bat;
  emxEnsureCapacity_real_T(VbatMat0, i);
  VbatMat0_data = VbatMat0->data;
  for (i = 0; i < c_loop_ub_tmp; i++) {
    VbatMat0_data[i] = 0.0;
  }
  /*  Ibat   = zeros(Nina219*N_bat); */
  /*  tV1    = zeros(Nina219*N_bat); */
  /*  tI1    = zeros(Nina219*N_bat); */
  i = tV->size[0] * tV->size[1];
  tV->size[0] = (int)Nina219;
  tV->size[1] = (int)N_bat;
  emxEnsureCapacity_real_T(tV, i);
  tV_data = tV->data;
  for (i = 0; i < c_loop_ub_tmp; i++) {
    tV_data[i] = 0.0;
  }
  i = tI->size[0] * tI->size[1];
  tI->size[0] = (int)Nina219;
  tI->size[1] = (int)N_bat;
  emxEnsureCapacity_real_T(tI, i);
  tI_data = tI->data;
  for (i = 0; i < c_loop_ub_tmp; i++) {
    tI_data[i] = 0.0;
  }
  i = Ishunt->size[0] * Ishunt->size[1];
  Ishunt->size[0] = (int)Nina219;
  Ishunt->size[1] = (int)N_bat;
  emxEnsureCapacity_real_T(Ishunt, i);
  Ishunt_data = Ishunt->data;
  for (i = 0; i < c_loop_ub_tmp; i++) {
    Ishunt_data[i] = 0.0;
  }
  emxInit_real_T(&b_I, 1);
  i = b_I->size[0];
  b_I->size[0] = (int)N_bat;
  emxEnsureCapacity_real_T(b_I, i);
  I_data = b_I->data;
  for (i = 0; i < b_loop_ub_tmp; i++) {
    I_data[i] = 0.0;
  }
  emxInit_real_T(&V, 1);
  i = V->size[0];
  V->size[0] = (int)N_bat;
  emxEnsureCapacity_real_T(V, i);
  V_data = V->data;
  for (i = 0; i < b_loop_ub_tmp; i++) {
    V_data[i] = 0.0;
  }
  emxInit_real_T(&tvv, 1);
  i = tvv->size[0];
  tvv->size[0] = (int)N_bat;
  emxEnsureCapacity_real_T(tvv, i);
  tvv_data = tvv->data;
  for (i = 0; i < b_loop_ub_tmp; i++) {
    tvv_data[i] = 0.0;
  }
  emxInit_real_T(&tii, 1);
  i = tii->size[0];
  tii->size[0] = (int)N_bat;
  emxEnsureCapacity_real_T(tii, i);
  tii_data = tii->data;
  for (i = 0; i < b_loop_ub_tmp; i++) {
    tii_data[i] = 0.0;
  }
  i = Iacs758->size[0];
  Iacs758->size[0] = (int)Nina219;
  emxEnsureCapacity_real_T(Iacs758, i);
  Iacs758_data = Iacs758->data;
  for (i = 0; i < loop_ub_tmp; i++) {
    Iacs758_data[i] = 0.0;
  }
  errI2C_size[0] = 0;
  errI2C_size[1] = 0;
  emxInit_real_T(&r, 2);
  for (k_ina219 = 0; k_ina219 < loop_ub_tmp; k_ina219++) {
    switch (ProjectFlag) {
    case 2:
      /* ESP32 */
      /*                  toc0 = toc; */
      i = V->size[0];
      V->size[0] = 16;
      emxEnsureCapacity_real_T(V, i);
      V_data = V->data;
      i = b_I->size[0];
      b_I->size[0] = 16;
      emxEnsureCapacity_real_T(b_I, i);
      I_data = b_I->data;
      i = tvv->size[0];
      tvv->size[0] = 16;
      emxEnsureCapacity_real_T(tvv, i);
      tvv_data = tvv->data;
      i = tii->size[0];
      tii->size[0] = 16;
      emxEnsureCapacity_real_T(tii, i);
      tii_data = tii->data;
      for (i = 0; i < 16; i++) {
        V_data[i] = 0.0;
        I_data[i] = 0.0;
        tvv_data[i] = 0.0;
        tii_data[i] = 0.0;
      }
      errI2C_size[0] = 1;
      errI2C_size[1] = 1;
      errI2C_data[0] = -1.0;
      if (N_bat < 1.0) {
        loop_ub = 0;
      } else {
        loop_ub = (int)N_bat;
      }
      for (i = 0; i < 2; i++) {
        for (trueCount = 0; trueCount < loop_ub; trueCount++) {
          Vdebug0_data[(k_ina219 + Vdebug0->size[0] * trueCount) +
                       Vdebug0->size[0] * Vdebug0->size[1] * i] = 0.0;
        }
      }
      break;
    case 3:
      /* ESP32 ser */
      /*                  toc0 = toc; */
      for (i = 0; i < 16; i++) {
        b_VIpacId[i] = VIpacId[k_ina219 + (i << 1)];
      }
      d_readI2cVIfastTicTocV5_Rratio_(b_VIpacId, N_bat1, V, b_I, b_tvv, b_tii,
                                      errI2C_data, errI2C_size, r);
      r1 = r->data;
      I_data = b_I->data;
      V_data = V->data;
      i = tvv->size[0];
      tvv->size[0] = 16;
      emxEnsureCapacity_real_T(tvv, i);
      tvv_data = tvv->data;
      i = tii->size[0];
      tii->size[0] = 16;
      emxEnsureCapacity_real_T(tii, i);
      tii_data = tii->data;
      for (i = 0; i < 16; i++) {
        tvv_data[i] = b_tvv[i];
        tii_data[i] = b_tii[i];
      }
      if (N_bat < 1.0) {
        loop_ub = 0;
      } else {
        loop_ub = (int)N_bat;
      }
      for (i = 0; i < 2; i++) {
        for (trueCount = 0; trueCount < loop_ub; trueCount++) {
          Vdebug0_data[(k_ina219 + Vdebug0->size[0] * trueCount) +
                       Vdebug0->size[0] * Vdebug0->size[1] * i] =
              r1[trueCount + loop_ub * i];
        }
      }
      break;
    }
    if ((errI2C_size[0] != 0) && (errI2C_size[1] != 0)) {
      keepMeas = false;
    } else {
      if (Iacs758Flag == 1) {
        Iacs758_data[k_ina219] = pIacs758_data[k_ina219] * 0.0F +
                                 pIacs758_data[k_ina219 + pIacs758->size[0]];
        I_idx_0 = b_I->size[0];
        i = b_I->size[0];
        b_I->size[0] = I_idx_0;
        emxEnsureCapacity_real_T(b_I, i);
        I_data = b_I->data;
        for (i = 0; i < I_idx_0; i++) {
          I_data[i] = 0.0;
        }
        /* Pac2Vid0(:,1); */
        loop_ub = Pac2Vid0All->size[1];
        I_idx_0 = Pac2Vid0All->size[1];
        for (i = 0; i < loop_ub; i++) {
          tmp_data[i] =
              (Pac2Vid0All_data[k_ina219 + Pac2Vid0All->size[0] * i] > 0);
        }
        I_idx_0--;
        trueCount = 0;
        loop_ub = 0;
        for (i = 0; i <= I_idx_0; i++) {
          if (tmp_data[i]) {
            trueCount++;
            c_tmp_data[loop_ub] = (short)i;
            loop_ub++;
          }
        }
        for (i = 0; i < trueCount; i++) {
          b_tmp_data[i] =
              Pac2Vid0All_data[k_ina219 + Pac2Vid0All->size[0] * c_tmp_data[i]];
        }
        for (i = 0; i < trueCount; i++) {
          I_data[b_tmp_data[i] - 1] = Iacs758_data[k_ina219];
        }
      } else if (Iacs758Flag == 2) {
        b = 0.0;
        switch (ProjectFlag) {
        case 2:
          /* ESP32 */
          break;
        case 3:
          /* ESP32 ser */
          /* UNTITLED3 Summary of this function goes here */
          /*    Detailed explanation goes here */
          b = randn() * 5.0;
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
          break;
        }
        Iacs758_data[k_ina219] = pIacs758_data[k_ina219] * (float)b +
                                 pIacs758_data[k_ina219 + pIacs758->size[0]];
        I_idx_0 = b_I->size[0];
        i = b_I->size[0];
        b_I->size[0] = I_idx_0;
        emxEnsureCapacity_real_T(b_I, i);
        I_data = b_I->data;
        for (i = 0; i < I_idx_0; i++) {
          I_data[i] = 0.0;
        }
        /* Pac2Vid0(:,1); */
        loop_ub = Pac2Vid0All->size[1];
        I_idx_0 = Pac2Vid0All->size[1];
        for (i = 0; i < loop_ub; i++) {
          tmp_data[i] =
              (Pac2Vid0All_data[k_ina219 + Pac2Vid0All->size[0] * i] > 0);
        }
        I_idx_0--;
        trueCount = 0;
        loop_ub = 0;
        for (i = 0; i <= I_idx_0; i++) {
          if (tmp_data[i]) {
            trueCount++;
            d_tmp_data[loop_ub] = (short)i;
            loop_ub++;
          }
        }
        for (i = 0; i < trueCount; i++) {
          b_tmp_data[i] =
              Pac2Vid0All_data[k_ina219 + Pac2Vid0All->size[0] * d_tmp_data[i]];
        }
        for (i = 0; i < trueCount; i++) {
          I_data[b_tmp_data[i] - 1] = Iacs758_data[k_ina219];
        }
      }
      loop_ub = VbatMat0->size[1];
      for (i = 0; i < loop_ub; i++) {
        VbatMat0_data[k_ina219 + VbatMat0->size[0] * i] = V_data[i];
      }
      /*  + squeeze(Voffset(k_ina219,:,:))'; */
      for (I_idx_0 = 0; I_idx_0 < b_loop_ub_tmp; I_idx_0++) {
        Ishunt_data[k_ina219 + Ishunt->size[0] * I_idx_0] =
            I_data[I_idx_0] *
                pIshunt_data[k_ina219 + pIshunt->size[0] * I_idx_0] +
            pIshunt_data[(k_ina219 + pIshunt->size[0] * I_idx_0) +
                         pIshunt->size[0] * pIshunt->size[1]];
      }
      loop_ub = tV->size[1];
      for (i = 0; i < loop_ub; i++) {
        tV_data[k_ina219 + tV->size[0] * i] = tvv_data[i];
      }
      /* (toc0+toc1)/2; */
      loop_ub = tI->size[1];
      for (i = 0; i < loop_ub; i++) {
        tI_data[k_ina219 + tI->size[0] * i] = tii_data[i];
      }
      /* tV(k_ina219,:,k_t); */
      /*  switch ProjectFlag */
      /*      case 2%ESP32 */
      /*  end */
    }
  }
  emxFree_real_T(&r);
  emxFree_real_T(&tii);
  emxFree_real_T(&tvv);
  emxFree_real_T(&V);
  emxFree_real_T(&b_I);
  emxInit_real_T(&b_Vbat0, 2);
  i = b_Vbat0->size[0] * b_Vbat0->size[1];
  b_Vbat0->size[0] = VbatMat0->size[1];
  b_Vbat0->size[1] = VbatMat0->size[0];
  emxEnsureCapacity_real_T(b_Vbat0, i);
  V_data = b_Vbat0->data;
  loop_ub = VbatMat0->size[0];
  for (i = 0; i < loop_ub; i++) {
    I_idx_0 = VbatMat0->size[1];
    for (trueCount = 0; trueCount < I_idx_0; trueCount++) {
      V_data[trueCount + b_Vbat0->size[0] * i] =
          VbatMat0_data[i + VbatMat0->size[0] * trueCount];
    }
  }
  i = Vbat0->size[0];
  Vbat0->size[0] = c_loop_ub_tmp;
  emxEnsureCapacity_real_T(Vbat0, i);
  Vdebug0_data = Vbat0->data;
  for (i = 0; i < c_loop_ub_tmp; i++) {
    Vdebug0_data[i] = V_data[i];
  }
  emxInit_real_T(&tV1mat, 2);
  i = tV1mat->size[0] * tV1mat->size[1];
  tV1mat->size[0] = tV->size[1];
  tV1mat->size[1] = tV->size[0];
  emxEnsureCapacity_real_T(tV1mat, i);
  I_data = tV1mat->data;
  loop_ub = tV->size[0];
  for (i = 0; i < loop_ub; i++) {
    I_idx_0 = tV->size[1];
    for (trueCount = 0; trueCount < I_idx_0; trueCount++) {
      I_data[trueCount + tV1mat->size[0] * i] =
          tV_data[i + tV->size[0] * trueCount];
    }
  }
  i = tV1->size[0];
  tV1->size[0] = c_loop_ub_tmp;
  emxEnsureCapacity_real_T(tV1, i);
  Vdebug0_data = tV1->data;
  for (i = 0; i < c_loop_ub_tmp; i++) {
    Vdebug0_data[i] = I_data[i];
  }
  i = tV1mat->size[0] * tV1mat->size[1];
  tV1mat->size[0] = tI->size[1];
  tV1mat->size[1] = tI->size[0];
  emxEnsureCapacity_real_T(tV1mat, i);
  I_data = tV1mat->data;
  loop_ub = tI->size[0];
  for (i = 0; i < loop_ub; i++) {
    I_idx_0 = tI->size[1];
    for (trueCount = 0; trueCount < I_idx_0; trueCount++) {
      I_data[trueCount + tV1mat->size[0] * i] =
          tI_data[i + tI->size[0] * trueCount];
    }
  }
  i = tI1->size[0];
  tI1->size[0] = c_loop_ub_tmp;
  emxEnsureCapacity_real_T(tI1, i);
  Vdebug0_data = tI1->data;
  for (i = 0; i < c_loop_ub_tmp; i++) {
    Vdebug0_data[i] = I_data[i];
  }
  i = tV1mat->size[0] * tV1mat->size[1];
  tV1mat->size[0] = Ishunt->size[1];
  tV1mat->size[1] = Ishunt->size[0];
  emxEnsureCapacity_real_T(tV1mat, i);
  I_data = tV1mat->data;
  loop_ub = Ishunt->size[0];
  for (i = 0; i < loop_ub; i++) {
    I_idx_0 = Ishunt->size[1];
    for (trueCount = 0; trueCount < I_idx_0; trueCount++) {
      I_data[trueCount + tV1mat->size[0] * i] =
          Ishunt_data[i + Ishunt->size[0] * trueCount];
    }
  }
  i = Ibat->size[0];
  Ibat->size[0] = c_loop_ub_tmp;
  emxEnsureCapacity_real_T(Ibat, i);
  Vdebug0_data = Ibat->data;
  for (i = 0; i < c_loop_ub_tmp; i++) {
    Vdebug0_data[i] = I_data[i];
  }
  emxInit_real32_T(&Rwire0, 2);
  i = Rwire0->size[0] * Rwire0->size[1];
  Rwire0->size[0] = Rwire->size[1];
  Rwire0->size[1] = Rwire->size[0];
  emxEnsureCapacity_real32_T(Rwire0, i);
  Rwire0_data = Rwire0->data;
  loop_ub = Rwire->size[0];
  for (i = 0; i < loop_ub; i++) {
    I_idx_0 = Rwire->size[1];
    for (trueCount = 0; trueCount < I_idx_0; trueCount++) {
      Rwire0_data[trueCount + Rwire0->size[0] * i] =
          Rwire_data[i + Rwire->size[0] * trueCount];
    }
  }
  I_idx_0 = Rwire0->size[0] * Rwire0->size[1];
  if (I_idx_0 == 1) {
    b_loop_ub_tmp = c_loop_ub_tmp;
  } else {
    b_loop_ub_tmp = I_idx_0;
  }
  if ((I_idx_0 == c_loop_ub_tmp) && (c_loop_ub_tmp == b_loop_ub_tmp)) {
    i = Vbat->size[0];
    Vbat->size[0] = c_loop_ub_tmp;
    emxEnsureCapacity_real32_T(Vbat, i);
    Vbat_data = Vbat->data;
    for (i = 0; i < c_loop_ub_tmp; i++) {
      Vbat_data[i] = (float)V_data[i] + Rwire0_data[i] * (float)I_data[i];
    }
  } else {
    binary_expand_op_8(Vbat, b_Vbat0, Rwire0, tV1mat);
  }
  emxFree_real_T(&b_Vbat0);
  emxFree_real32_T(&Rwire0);
  emxFree_real_T(&tV1mat);
  if (Rwire->size[0] == 1) {
    b_loop_ub_tmp = Ishunt->size[0];
  } else {
    b_loop_ub_tmp = Rwire->size[0];
  }
  if (Rwire->size[1] == 1) {
    loop_ub_tmp = Ishunt->size[1];
  } else {
    loop_ub_tmp = Rwire->size[1];
  }
  if ((Rwire->size[0] == Ishunt->size[0]) &&
      (Rwire->size[1] == Ishunt->size[1]) &&
      (VbatMat0->size[0] == b_loop_ub_tmp) &&
      (VbatMat0->size[1] == loop_ub_tmp)) {
    i = VbatMat->size[0] * VbatMat->size[1];
    VbatMat->size[0] = VbatMat0->size[0];
    VbatMat->size[1] = VbatMat0->size[1];
    emxEnsureCapacity_real32_T(VbatMat, i);
    Rwire0_data = VbatMat->data;
    for (i = 0; i < c_loop_ub_tmp; i++) {
      Rwire0_data[i] =
          (float)VbatMat0_data[i] + Rwire_data[i] * (float)Ishunt_data[i];
    }
  } else {
    binary_expand_op_7(VbatMat, VbatMat0, Rwire, Ishunt);
  }
  return keepMeas;
}

/*
 * File trailer for ReadVI.c
 *
 * [EOF]
 */
