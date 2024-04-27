/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 17-Apr-2024 18:20:59
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
#include "writePortToSpi4RowMask_ser.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* Type Definitions */
#ifndef struct_emxArray_real_T_1x1
#define struct_emxArray_real_T_1x1

struct emxArray_real_T_1x1
{
  double data[1];
  int size[2];
};

#endif                                 /* struct_emxArray_real_T_1x1 */

#ifndef typedef_emxArray_real_T_1x1
#define typedef_emxArray_real_T_1x1

typedef struct emxArray_real_T_1x1 emxArray_real_T_1x1;

#endif                                 /* typedef_emxArray_real_T_1x1 */

/* Function Declarations */
static void binary_expand_op(double in1_data[], int in1_size[2], const
  emxArray_real_T *in2, const signed char in3_data[], const int in3_size[2]);
static void binary_expand_op_1(float in1_data[], const int in1_size[2], int in2,
  const double in3_data[], const int in3_size[2], const struct0_T *in4);
static void binary_expand_op_2(double in1_data[], int in1_size[2], const
  emxArray_real_T *in2, const struct0_T *in3, const int in4_data[], int in5, int
  in6);
static void binary_expand_op_3(emxArray_real_T *in1, const emxArray_real_T *in2,
  const double in3_data[], const signed char in4_data[], const int *in4_size,
  int in5);
static float rt_roundf_snf(float u);

/* Function Definitions */
/*
 * Arguments    : double in1_data[]
 *                int in1_size[2]
 *                const emxArray_real_T *in2
 *                const signed char in3_data[]
 *                const int in3_size[2]
 * Return Type  : void
 */
static void binary_expand_op(double in1_data[], int in1_size[2], const
  emxArray_real_T *in2, const signed char in3_data[], const int in3_size[2])
{
  const double *in2_data;
  int aux_0_1;
  int aux_1_1;
  int b_loop_ub;
  int i;
  int i1;
  int loop_ub;
  int stride_0_1;
  int stride_1_1;
  in2_data = in2->data;
  in1_size[0] = in2->size[0];
  if (in3_size[1] == 1) {
    loop_ub = in2->size[1];
  } else {
    loop_ub = in3_size[1];
  }

  in1_size[1] = loop_ub;
  stride_0_1 = (in2->size[1] != 1);
  stride_1_1 = (in3_size[1] != 1);
  aux_0_1 = 0;
  aux_1_1 = 0;
  for (i = 0; i < loop_ub; i++) {
    b_loop_ub = in2->size[0];
    for (i1 = 0; i1 < b_loop_ub; i1++) {
      in1_data[i1 + in1_size[0] * i] = in2_data[i1 + in2->size[0] * aux_0_1] *
        (double)in3_data[aux_1_1];
    }

    aux_1_1 += stride_1_1;
    aux_0_1 += stride_0_1;
  }
}

/*
 * Arguments    : float in1_data[]
 *                const int in1_size[2]
 *                int in2
 *                const double in3_data[]
 *                const int in3_size[2]
 *                const struct0_T *in4
 * Return Type  : void
 */
static void binary_expand_op_1(float in1_data[], const int in1_size[2], int in2,
  const double in3_data[], const int in3_size[2], const struct0_T *in4)
{
  int i;
  int loop_ub;
  int stride_0_1;
  stride_0_1 = (in3_size[1] != 1);
  loop_ub = in1_size[1];
  for (i = 0; i < loop_ub; i++) {
    in1_data[in2 + in1_size[0] * i] = (float)in3_data[in2 + in3_size[0] * (i *
      stride_0_1)] - in4->bat.Rint[in2 + (i << 1)];
  }
}

/*
 * Arguments    : double in1_data[]
 *                int in1_size[2]
 *                const emxArray_real_T *in2
 *                const struct0_T *in3
 *                const int in4_data[]
 *                int in5
 *                int in6
 * Return Type  : void
 */
static void binary_expand_op_2(double in1_data[], int in1_size[2], const
  emxArray_real_T *in2, const struct0_T *in3, const int in4_data[], int in5, int
  in6)
{
  const double *in2_data;
  int aux_0_1;
  int aux_1_1;
  int b_loop_ub;
  int i;
  int i1;
  int in4;
  int loop_ub;
  int stride_0_1;
  int stride_1_1;
  in2_data = in2->data;
  in4 = in4_data[in5];
  in1_size[0] = in2->size[0];
  if (in6 + 1 == 1) {
    loop_ub = in2->size[1];
  } else {
    loop_ub = in6 + 1;
  }

  in1_size[1] = loop_ub;
  stride_0_1 = (in2->size[1] != 1);
  stride_1_1 = (in6 + 1 != 1);
  aux_0_1 = 0;
  aux_1_1 = 0;
  for (i = 0; i < loop_ub; i++) {
    b_loop_ub = in2->size[0];
    for (i1 = 0; i1 < b_loop_ub; i1++) {
      in1_data[i1 + in1_size[0] * i] = in2_data[i1 + in2->size[0] * aux_0_1] *
        (double)in3->seq.tst.i.NegIflag[(in4 + (aux_1_1 << 3)) - 1];
    }

    aux_1_1 += stride_1_1;
    aux_0_1 += stride_0_1;
  }
}

/*
 * Arguments    : emxArray_real_T *in1
 *                const emxArray_real_T *in2
 *                const double in3_data[]
 *                const signed char in4_data[]
 *                const int *in4_size
 *                int in5
 * Return Type  : void
 */
static void binary_expand_op_3(emxArray_real_T *in1, const emxArray_real_T *in2,
  const double in3_data[], const signed char in4_data[], const int *in4_size,
  int in5)
{
  const double *in2_data;
  double *in1_data;
  int i;
  int loop_ub;
  int stride_0_0;
  int stride_1_0;
  in2_data = in2->data;
  if (*in4_size == 1) {
    loop_ub = in2->size[1];
  } else {
    loop_ub = *in4_size;
  }

  i = in1->size[0];
  in1->size[0] = loop_ub;
  emxEnsureCapacity_real_T(in1, i);
  in1_data = in1->data;
  stride_0_0 = (in2->size[1] != 1);
  stride_1_0 = (*in4_size != 1);
  for (i = 0; i < loop_ub; i++) {
    in1_data[i] = in2_data[i * stride_0_0] + (in3_data[in4_data[i * stride_1_0]]
      - 1.0) * (double)in5;
  }
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
 * const
 *
 * Arguments    : struct0_T *prm
 *                struct30_T *outStruct
 * Return Type  : void
 */
void Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229(struct0_T *prm, struct30_T
  *outStruct)
{
  /*static */unsigned char bitCnfg_data[32768];
  /*static */unsigned char c_y_data[32767];
  /*static */signed char j_tmp_data[32767];
  emxArray_boolean_T b_AllBypassPerGroup_data;
  emxArray_boolean_T b_SelDual_nm1_data;
  emxArray_boolean_T c_SelDual_nm1_data;
  emxArray_boolean_T d_SelDual_nm1_data;
  emxArray_boolean_T e_SelDual_nm1_data;
  emxArray_boolean_T *b_changeConfigFlag;
  emxArray_boolean_T *changeConfigFlag;
  emxArray_boolean_T *switchFlag;
  emxArray_boolean_T *switchFlagHelp;
  emxArray_boolean_T *x;
  emxArray_char_T *IchargeAct;
  emxArray_char_T *Vcharge0;
  emxArray_char_T *b_IchargeAct;
  emxArray_char_T *b_IshuntTest2;
  emxArray_char_T *b_VbusTest2;
  emxArray_char_T *b_VbusTest_data;
  emxArray_char_T *b_Vcharge0;
  emxArray_char_T *b_VmKp184Test_data;
  emxArray_char_T *b_k_Ttest;
  emxArray_char_T *b_k_bat;
  emxArray_char_T *b_prm;
  emxArray_char_T *b_vSumMax;
  emxArray_char_T *c_IchargeAct;
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
  emxArray_char_T *r4;
  emxArray_char_T *r5;
  emxArray_char_T *r6;
  emxArray_char_T *r7;
  emxArray_char_T *r8;
  emxArray_char_T *r9;
  emxArray_char_T *vSumMax;
  emxArray_int8_T *b_Vbat;
  emxArray_int8_T *c_Vbat;
  emxArray_real_T c_BattConfigPerInaHelp_data;
  emxArray_real_T d_BattConfigPerInaHelp_data;
  emxArray_real_T *Iacs758_cal;
  emxArray_real_T *IbatMat;
  emxArray_real_T *Ibrd;
  emxArray_real_T *ImKp184Test2;
  emxArray_real_T *IshuntTest2;
  emxArray_real_T *Ishunt_cal_bat_00;
  emxArray_real_T *SelDual;
  emxArray_real_T *SelDualHelp;
  emxArray_real_T *SelDual_nm1;
  emxArray_real_T *Vbat;
  emxArray_real_T *VbatMat;
  emxArray_real_T *VbatMat0;
  emxArray_real_T *VbitInsMeas;
  emxArray_real_T *Vbrd;
  emxArray_real_T *VbusTest2;
  emxArray_real_T *Vbus_cal_bat_00;
  emxArray_real_T *b_y;
  emxArray_real_T *indVperGrp;
  emxArray_real_T *meanIbrd;
  emxArray_real_T *r;
  emxArray_real_T *r1;
  emxArray_real_T *tLastToggle;
  emxArray_real_T *tV;
  emxArray_real_T *tV1;
  emxArray_real_T *y;
  emxArray_real_T_1x1 r30;
  emxArray_uint8_T c_bitCnfg_data;
  emxArray_uint8_T d_bitCnfg_data;
  emxArray_uint8_T e_bitCnfg_data;
  emxArray_uint8_T *r2;
  emxArray_uint8_T *r3;
  creal_T t2_data;
  double b_IshuntTest2_data[4080];
  double ImKp184Test2_bat_00_data[2048];
  double BattConfigPerInaHelp_data[512];
  double BattConfigPerIna_data[512];
  double n_tmp_data[512];
  double Vdebug_data[256];
  double VmV_data[256];
  double b_BattConfigPerInaHelp_data[256];
  double b_VbusTest2_data[256];
  double VdebugVec_data[64];
  double kEst_data[64];
  double Rtot_data[32];
  double VbusTest_data[32];
  double VmKp184Test_data[32];
  double a__38_data[32];
  double a__39_data[32];
  double a__40_data[32];
  double a__42_data[32];
  double a__72_data[32];
  double b_VdebugVec_data[32];
  double I_data[16];
  double b_VmV_data[16];
  double c_VmKp184Test_data[16];
  double tvv_data[16];
  double a__1_data[8];
  double dateVec[6];
  double VecIna219_data[2];
  double b_k_t0[2];
  double o_tmp_data[2];
  double outVI[2];
  double N0;
  double Nina219_tmp_tmp;
  double V_dis_all;
  double a__16_data;
  double a__4_data;
  double b_dv0;
  double b_meanIbrd0;
  double d;
  double d_tmp_data;
  double g_IchargeAct;
  double k_t;
  double toc00;
  double *Iacs758_cal_data;
  double *ImKp184Test2_data;
  double *IshuntTest2_data;
  double *SelDualHelp_data;
  double *SelDual_data;
  double *VbatMat0_data;
  double *VbatMat_data;
  double *VbusTest2_data;
  double *b_y_data;
  double *tLastToggle_data;
  double *tV1_data;
  double *tV_data;
  double *y_data;
  float varargin_1[1024];
  float f_prm[496];
  float pIacs758_data[64];
  float h_prm[38];
  float Rwire_data[32];
  float a__64_data[32];
  float fv[31];
  float a__36_data[16];
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
  int indVperGrp_data[16];
  int Ibrd_size[4];
  int IshuntTest2_size[4];
  int VbusTest2_size[4];
  int BattConfigPerInaHelp_size[3];
  int Pac2Vid0All_size[3];
  int VdebugVec_size[3];
  int b_VdebugVec_size[3];
  int bitCnfg_size[3];
  int c_tmp_size[3];
  int kEst_size[3];
  int pIacs758_size[3];
  int AllBypassPerGroup_size[2];
  int I_size[2];
  int IbatMat_size[2];
  int ImKp184Test2_bat_00_size[2];
  int Nina219[2];
  int Rtot_size[2];
  int Rwire_size[2];
  int SelDual_nm1_size[2];
  int Vdebug_size[2];
  int VmKp184Test_size[2];
  int VmV_size[2];
  int a__1_size[2];
  int a__3_size[2];
  int a__4_size[2];
  int b_Rwire_size[2];
  int b_VmV_size[2];
  int b_tmp_size[2];
  int d_tmp_size[2];
  int e_tmp_size[2];
  int errI2C_size[2];
  int f_tmp_size[2];
  int id2Bypass_data[2];
  int ipos_data[2];
  int tLastToggle_size[2];
  int tii_size[2];
  int tmp_size[2];
  int tvv_size[2];
  int N_bat_tmp_tmp_tmp;
  int VbusTest_size_idx_0;
  int VdebugVec_size_tmp;
  int VmKp184Test_size_idx_0;
  int VmKp184Test_size_idx_1;
  int b_end;
  int b_loop_ub;
  int b_loop_ub_tmp;
  int b_size;
  int b_trueCount;
  int c_end;
  int c_loop_ub;
  int c_loop_ub_tmp;
  int d_loop_ub;
  int e_loop_ub;
  int end;
  int end_tmp;
  int i;
  int i1;
  int i2;
  int i3;
  int id2Bypass_size;
  int ipos_size;
  int k0;
  int k1;
  int k_ina219_;
  int k_t0;
  int k_tstState;
  int loop_ub;
  int loop_ub_tmp;
  int partialTrueCount;
  int state_k;
  int trueCount;
  short b_data[2];
  short prm_data[2];
  short N_bat1;
  short N_bitCnfg;
  short i4;
  unsigned short size_tmp_idx_1;
  char bitCnfgStr_data[4096];
  unsigned char Pac2Vid0All_data[512];
  unsigned char Pac2Vid0[256];
  unsigned char b_bitCnfg_data[256];
  unsigned char i_tmp_data[256];
  unsigned char id0_data[256];
  unsigned char l_tmp_data[256];
  unsigned char m_tmp_data[256];
  signed char g_tmp_data[255];
  signed char h_tmp_data[255];
  unsigned char SpiPortRowBypass[24];
  unsigned char disConAll[24];
  signed char g_prm[16];
  signed char iv[8];
  unsigned char N_bat[2];
  signed char b_tmp_data[2];
  signed char c_tmp_data[2];
  signed char e_tmp_data[2];
  signed char f_tmp_data[2];
  signed char k_tmp_data[2];
  signed char p_tmp_data[2];
  signed char q_tmp_data[2];
  signed char r_tmp_data[2];
  signed char s_tmp_data[2];
  signed char t_tmp_data[2];
  signed char tmp_data[2];
  signed char Iacs758Flag;
  unsigned char NstTst_tmp_tmp;
  signed char ProjectFlag;
  signed char k_groups_tmp;
  signed char *Vbat_data;
  unsigned char *r31;
  boolean_T VbusTest0_data[256];
  boolean_T ismember0_data[255];
  boolean_T BattConfigAct_data[32];
  boolean_T c_VbusTest_data[32];
  boolean_T AllBypassPerGroup_data[2];
  boolean_T SelDual_nm1_data[2];
  boolean_T b_VecIna219_data[2];
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
  boolean_T *b_changeConfigFlag_data;
  boolean_T *changeConfigFlag_data;
  boolean_T *switchFlagHelp_data;
  boolean_T *switchFlag_data;
  if (!isInitialized_Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229) {
    Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_initialize();
  }

  /* int16(32); */
  /*  NstrColMax= 256; */
  outStruct->VpassFlag = defineOutStruct(prm->Nmax.NbrdMax, prm->Nmax.NbatMax,
    outStruct->VbusTest, outStruct->VmKp184Test, &outStruct->VresetFlag,
    &outStruct->Rint);

  /*  []; */
  /*  init */
  ProjectFlag = prm->ins.ProjectFlag;

  /*  disconnect All */
  CalcPortSpiPrm(prm->brd.spi.disconnect, prm->brd.spi.bypass,
                 prm->brd.spi.PortSpiRow_esp, prm->brd.spi.SwitchMat_esp,
                 prm->brd.spi.PortSpiRow_esp2, prm->brd.spi.SwitchMat_esp2,
                 prm->brd.spi.Pac2Vid, prm->brd.spi.Pac2Vid2,
                 prm->brd.spi.disconnect, disConAll, a__1_data, a__1_size,
                 id0_data);
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
      writePortToSpi4RowMask_ser(disConAll);
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
  pIacs758_size[0] = 2;
  pIacs758_size[1] = 1;
  pIacs758_size[2] = 2;
  pIacs758_data[0] = prm->brd.pac.pIacs758[0];
  pIacs758_data[1] = prm->brd.pac.pIacs758[1];
  pIacs758_data[2] = prm->brd.pac.pIacs758[2];
  pIacs758_data[3] = prm->brd.pac.pIacs758[3];
  if (prm->brd.useRwireFlag) {
    Rwire_size[0] = 2;
    Rwire_size[1] = 16;
    memcpy(&Rwire_data[0], &prm->brd.Rwire[0], 32U * sizeof(float));
  } else {
    Rwire_size[0] = (int)prm->brd.Nina219;
    Rwire_size[1] = prm->brd.N_bat;
    loop_ub = (int)prm->brd.Nina219 * prm->brd.N_bat;
    if (loop_ub - 1 >= 0) {
      memset(&Rwire_data[0], 0, (unsigned int)loop_ub * sizeof(float));
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

  partialTrueCount = 0;
  if (prm->ser.com.grp[0] > 0) {
    tmp_data[0] = 0;
    partialTrueCount = 1;
  }

  if (prm->ser.com.grp[1] > 0) {
    tmp_data[partialTrueCount] = 1;
  }

  end_tmp = trueCount - 1;
  trueCount = 0;
  partialTrueCount = 0;
  for (loop_ub = 0; loop_ub <= end_tmp; loop_ub++) {
    i = prm->ser.com.grp[tmp_data[loop_ub]];
    if (i > 0) {
      trueCount++;
      b_tmp_data[partialTrueCount] = (signed char)loop_ub;
      partialTrueCount++;
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
  for (i = 0; i < loop_ub_tmp; i++) {
    VbusTest_data[i] = 0.0;
    VmKp184Test_data[i] = 0.0;
    BattConfigAct_data[i] = true;
  }

  pause(1.0);

  /* sum(seqTstV.grp>0); */
  outStruct->VpassFlag = true;
  outStruct->VresetFlag = true;
  emxInit_real_T(&VbusTest2, 4);
  emxInit_real_T(&IshuntTest2, 4);
  emxInit_real_T(&ImKp184Test2, 4);
  emxInit_real_T(&Iacs758_cal, 4);
  emxInit_real_T(&Vbrd, 5);
  emxInit_real_T(&Ibrd, 5);
  emxInit_real_T(&meanIbrd, 4);
  emxInit_real_T(&VbitInsMeas, 4);
  emxInit_real_T(&Vbat, 2);
  emxInit_real_T(&IbatMat, 3);
  emxInit_real_T(&VbatMat, 3);
  emxInit_real_T(&VbatMat0, 3);
  emxInit_real_T(&tV1, 2);
  emxInit_real_T(&tV, 3);
  emxInit_real_T(&SelDual, 2);
  emxInit_real_T(&SelDualHelp, 2);
  emxInit_real_T(&tLastToggle, 2);
  emxInit_boolean_T(&switchFlag);
  emxInit_boolean_T(&switchFlagHelp);
  emxInit_boolean_T(&changeConfigFlag);
  emxInit_real_T(&SelDual_nm1, 2);
  emxInit_real_T(&Ishunt_cal_bat_00, 2);
  emxInit_real_T(&Vbus_cal_bat_00, 2);
  emxInit_real_T(&r, 2);
  emxInit_real_T(&indVperGrp, 1);
  emxInit_int8_T(&b_Vbat);
  emxInit_int8_T(&c_Vbat);
  emxInit_real_T(&y, 2);
  y_data = y->data;
  emxInit_boolean_T(&x);
  emxInit_real_T(&r1, 2);
  emxInit_real_T(&b_y, 1);
  emxInit_uint8_T(&r2, 3);
  emxInit_boolean_T(&b_changeConfigFlag);
  emxInit_uint8_T(&r3, 2);
  emxInit_char_T(&r4);
  emxInit_char_T(&r5);
  emxInit_char_T(&k_bat);
  emxInit_char_T(&b_VbusTest_data);
  emxInit_char_T(&r6);
  emxInit_char_T(&b_k_bat);
  emxInit_char_T(&b_VmKp184Test_data);
  emxInit_char_T(&vSumMax);
  emxInit_char_T(&r7);
  emxInit_char_T(&b_prm);
  emxInit_char_T(&r8);
  emxInit_char_T(&r9);
  emxInit_char_T(&c_prm);
  emxInit_char_T(&r10);
  emxInit_char_T(&r11);
  emxInit_char_T(&k_Ttest);
  emxInit_char_T(&b_vSumMax);
  emxInit_char_T(&b_VbusTest2);
  emxInit_char_T(&d_prm);
  emxInit_char_T(&b_k_Ttest);
  emxInit_char_T(&b_IshuntTest2);
  emxInit_char_T(&r12);
  emxInit_char_T(&r13);
  emxInit_char_T(&meanIbrd0);
  emxInit_char_T(&r14);
  emxInit_char_T(&r15);
  emxInit_char_T(&r16);
  emxInit_char_T(&dv0);
  emxInit_char_T(&r17);
  emxInit_char_T(&r18);
  emxInit_char_T(&Vcharge0);
  emxInit_char_T(&b_Vcharge0);
  emxInit_char_T(&r19);
  emxInit_char_T(&IchargeAct);
  emxInit_char_T(&b_IchargeAct);
  emxInit_char_T(&r20);
  emxInit_char_T(&r21);
  emxInit_char_T(&r22);
  emxInit_char_T(&r23);
  emxInit_char_T(&r24);
  emxInit_char_T(&c_IchargeAct);
  emxInit_char_T(&d_IchargeAct);
  emxInit_char_T(&e_IchargeAct);
  emxInit_char_T(&r25);
  emxInit_char_T(&f_IchargeAct);
  emxInit_char_T(&r26);
  emxInit_char_T(&r27);
  emxInit_char_T(&c_Vcharge0);
  emxInit_char_T(&r28);
  emxInit_char_T(&d_Vcharge0);
  emxInit_char_T(&r29);
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
    VdebugVec_size[0] = (int)prm->brd.Nina219;
    VdebugVec_size_tmp = prm->brd.N_bat;
    b_loop_ub_tmp = loop_ub_tmp << 1;
    if (b_loop_ub_tmp - 1 >= 0) {
      memset(&VdebugVec_data[0], 0, (unsigned int)b_loop_ub_tmp * sizeof(double));
    }

    if (prm->brd.Nina219 < 1.0) {
      y->size[0] = 1;
      y->size[1] = 0;
    } else {
      i = y->size[0] * y->size[1];
      y->size[0] = 1;
      y->size[1] = (int)(prm->brd.Nina219 - 1.0) + 1;
      emxEnsureCapacity_real_T(y, i);
      y_data = y->data;
      loop_ub = (int)(prm->brd.Nina219 - 1.0);
      for (i = 0; i <= loop_ub; i++) {
        y_data[i] = (double)i + 1.0;
      }
    }

    i = b_y->size[0];
    b_y->size[0] = y->size[1];
    emxEnsureCapacity_real_T(b_y, i);
    b_y_data = b_y->data;
    loop_ub = y->size[1];
    for (i = 0; i < loop_ub; i++) {
      b_y_data[i] = y_data[i];
    }

    loop_ub = y->size[1];
    for (i = 0; i < loop_ub; i++) {
      VecIna219_data[i] = b_y_data[i];
    }

    kEst_size[0] = (int)prm->brd.Nina219;
    if (b_loop_ub_tmp - 1 >= 0) {
      memset(&kEst_data[0], 0, (unsigned int)b_loop_ub_tmp * sizeof(double));
    }

    /*  [~,~,~,~]          = ControlKp184(s_kp184,'Read',[]); */
    i = prm->seq.tst.v.Nst;
    for (k_tstState = 0; k_tstState < i; k_tstState++) {
      pause(0.1);

      /* 1:length(uGroups) */
      id2Bypass_size = ipos_size - 1;
      trueCount = 0;
      partialTrueCount = 0;

      /* VecIna219_k = find(k_groups == uGroupId); */
      b_trueCount = 0;
      for (loop_ub = 0; loop_ub <= id2Bypass_size; loop_ub++) {
        k_groups_tmp = prm->seq.tst.v.grp[k_tstState];
        i1 = ipos_data[loop_ub];
        if (k_groups_tmp == i1) {
          trueCount++;
          c_tmp_data[partialTrueCount] = (signed char)loop_ub;
          partialTrueCount++;
        }

        if (k_groups_tmp != i1) {
          b_trueCount++;
        }
      }

      for (k0 = 0; k0 < b_trueCount; k0++) {
        /* k_ina219not = VecIna219_notk %diconnect group that not tested */
        switch (ProjectFlag) {
         case 2:
          /* ESP32 */
          break;

         case 3:
          /* ESP32 ser */
          writePortToSpi4RowMask_ser(disConAll);
          break;
        }

        pause(0.1);
      }

      if (trueCount - 1 >= 0) {
        b_loop_ub = trueCount;
        i2 = N_bat_tmp_tmp_tmp;
        if (N_bat_tmp_tmp_tmp < 1) {
          size_tmp_idx_1 = 0U;
          y->size[0] = 1;
          y->size[1] = 0;
        } else {
          size_tmp_idx_1 = (unsigned short)N_bat_tmp_tmp_tmp;
          i1 = y->size[0] * y->size[1];
          y->size[0] = 1;
          y->size[1] = N_bat_tmp_tmp_tmp;
          emxEnsureCapacity_real_T(y, i1);
          y_data = y->data;
          loop_ub = N_bat_tmp_tmp_tmp - 1;
          for (i1 = 0; i1 <= loop_ub; i1++) {
            y_data[i1] = (double)i1 + 1.0;
          }
        }

        c_loop_ub = size_tmp_idx_1;
        i1 = (int)rt_roundd_snf((double)N_bat_tmp_tmp_tmp / 2.0 + 1.0);
        if (i1 < 256) {
          if (i1 >= 0) {
            NstTst_tmp_tmp = (unsigned char)i1;
          } else {
            NstTst_tmp_tmp = 0U;
          }
        } else {
          NstTst_tmp_tmp = MAX_uint8_T;
        }

        N_bat[0] = NstTst_tmp_tmp;
        i1 = N_bat_tmp_tmp_tmp;
        if (N_bat_tmp_tmp_tmp < 0) {
          i1 = 0;
        } else if (N_bat_tmp_tmp_tmp > 255) {
          i1 = 255;
        }

        N_bat[1] = (unsigned char)i1;
      }

      for (k0 = 0; k0 < trueCount; k0++) {
        /* k_ina219 = VecIna219_k */
        b_meanIbrd0 = VecIna219_data[c_tmp_data[k0]];
        for (i1 = 0; i1 < b_loop_ub; i1++) {
          b_VecIna219_data[i1] = (VecIna219_data[c_tmp_data[i1]] != b_meanIbrd0);
        }

        id2Bypass_size = c_eml_find(b_VecIna219_data, trueCount, id2Bypass_data);
        if (id2Bypass_size - 1 >= 0) {
          CalcPortSpiPrm(prm->brd.spi.disconnect, prm->brd.spi.bypass,
                         prm->brd.spi.PortSpiRow_esp, prm->brd.spi.SwitchMat_esp,
                         prm->brd.spi.PortSpiRow_esp2,
                         prm->brd.spi.SwitchMat_esp2, prm->brd.spi.Pac2Vid,
                         prm->brd.spi.Pac2Vid2, prm->brd.spi.bypass,
                         SpiPortRowBypass, a__1_data, a__1_size, id0_data);
        }

        for (k1 = 0; k1 < id2Bypass_size; k1++) {
          /* k_bypass = VecIna219_k(id2Bypass) */
          /*  ina219StateAll(:,k_bypass)   = ina219StateBypass; */
          /*  BattConfig{k_bypass}       = prm.brd.spi.bypass;%-2; */
          /*  BattConfigPerIna{k_bypass} = prm.brd.spi.bypass;%-2; */
          d_tmp_data = VecIna219_data[c_tmp_data[id2Bypass_data[k1] - 1]];
          if (d_tmp_data < 2.147483648E+9) {
            i1 = (int)d_tmp_data;
          } else {
            i1 = MAX_int32_T;
          }

          b_sprintf(i1, r4);
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
            writePortToSpi4RowMask_ser(SpiPortRowBypass);
            break;
          }

          pause(0.1);
        }

        /*  for k_bypass = VecIna219(id2Bypass) */
        if (i2 - 1 >= 0) {
          if (b_meanIbrd0 < 2.147483648E+9) {
            i3 = (int)b_meanIbrd0;
            c_loop_ub_tmp = (int)b_meanIbrd0;
          } else {
            i3 = MAX_int32_T;
            c_loop_ub_tmp = MAX_int32_T;
          }
        }

        for (b_loop_ub_tmp = 0; b_loop_ub_tmp < i2; b_loop_ub_tmp++) {
          if (b_loop_ub_tmp + 1 < 256) {
            NstTst_tmp_tmp = (unsigned char)((double)b_loop_ub_tmp + 1.0);
          } else {
            NstTst_tmp_tmp = MAX_uint8_T;
          }

          padArrUint8(NstTst_tmp_tmp, prm->Nmax.NbatMax, prm->Nmax.NbatMax, r3);
          b_CalcPortSpiPrm(prm->brd.spi.disconnect, prm->brd.spi.bypass,
                           prm->brd.spi.PortSpiRow_esp,
                           prm->brd.spi.SwitchMat_esp,
                           prm->brd.spi.PortSpiRow_esp2,
                           prm->brd.spi.SwitchMat_esp2, prm->brd.spi.Pac2Vid,
                           prm->brd.spi.Pac2Vid2, r3, SpiPortRowBypass,
                           a__1_data, a__1_size, id0_data);

          /*                  ina219State = findSwitch(SwitchCell,k_bat);%8+(k_bat-1); */
          /*  BattConfig{k_ina219}       = k_bat + (k_ina219-1)*N_bat; */
          /*  BattConfigPerIna{k_ina219} = k_bat; */
          /*  ina219StateAll(:,k_ina219)   = ina219State; */
          b_sprintf(i3, r5);
          partialTrueCount = a__1_size[0];
          if (partialTrueCount < 1) {
            partialTrueCount = 1;
          }

          if (a__1_size[0] == 0) {
            k_ina219_ = 0;
          } else {
            k_ina219_ = partialTrueCount;
          }

          for (id2Bypass_size = 0; id2Bypass_size < k_ina219_; id2Bypass_size++)
          {
            d_tmp_data = rt_roundd_snf(a__1_data[id2Bypass_size]);
            if (d_tmp_data < 2.147483648E+9) {
              if (d_tmp_data >= -2.147483648E+9) {
                i1 = (int)d_tmp_data;
              } else {
                i1 = MIN_int32_T;
              }
            } else if (d_tmp_data >= 2.147483648E+9) {
              i1 = MAX_int32_T;
            } else {
              i1 = 0;
            }

            printf("%d \n", i1);
            fflush(stdout);
          }

          switch (ProjectFlag) {
           case 2:
            /* ESP32 */
            break;

           case 3:
            /* ESP32 ser */
            writePortToSpi4RowMask_ser(SpiPortRowBypass);
            break;
          }

          pause(0.5);
          switch (ProjectFlag) {
           case 2:
            /* ESP32 */
            memset(&VmV_data[0], 0, 16U * sizeof(double));
            Vdebug_size[0] = 16;
            Vdebug_size[1] = 2;
            memset(&Vdebug_data[0], 0, 32U * sizeof(double));

            /*  */
            break;

           case 3:
            /* ESP32 ser */
            for (i1 = 0; i1 < 16; i1++) {
              g_prm[i1] = prm->brd.pac.VIpacId[((int)b_meanIbrd0 + (i1 << 1)) -
                1];
            }

            c_readI2cVIfastTicTocV5_Rratio_(g_prm, N_bat1, b_VmV_data,
              b_VmV_size, I_data, I_size, tvv_data, tvv_size, c_VmKp184Test_data,
              tii_size, (double *)&b_dv0, errI2C_size, Vdebug_data, Vdebug_size);
            loop_ub = b_VmV_size[0] * b_VmV_size[1];
            if (loop_ub - 1 >= 0) {
              memcpy(&VmV_data[0], &b_VmV_data[0], (unsigned int)loop_ub *
                     sizeof(double));
            }

            /*  */
            break;

           default:
            Vdebug_size[0] = N_bat_tmp_tmp_tmp;
            Vdebug_size[1] = 1;
            if (N_bat_tmp_tmp_tmp - 1 >= 0) {
              memset(&VmV_data[0], 0, (unsigned int)N_bat_tmp_tmp_tmp * sizeof
                     (double));
              memset(&Vdebug_data[0], 0, (unsigned int)N_bat_tmp_tmp_tmp *
                     sizeof(double));
            }
            break;
          }

          VbusTest_data[((int)b_meanIbrd0 + (int)Nina219_tmp_tmp * b_loop_ub_tmp)
            - 1] = VmV_data[b_loop_ub_tmp];
          b_trueCount = (int)b_meanIbrd0 + VdebugVec_size[0] * b_loop_ub_tmp;
          VdebugVec_data[b_trueCount - 1] = Vdebug_data[b_loop_ub_tmp];
          VdebugVec_data[(b_trueCount + VdebugVec_size[0] * VdebugVec_size_tmp)
            - 1] = Vdebug_data[b_loop_ub_tmp + Vdebug_size[0]];
          c_sprintf(c_loop_ub_tmp, b_loop_ub_tmp + 1, k_bat);
          d_sprintf(VbusTest_data[((int)b_meanIbrd0 + (int)Nina219_tmp_tmp *
                     b_loop_ub_tmp) - 1], b_VbusTest_data);
          pause(0.1);
          if (prm->seq.tst.v.ins[k_tstState] == 1) {
            /* kp184 */
            ControlKp184((double *)&N0, a__3_size, (double *)&a__4_data,
                         a__4_size, (double *)&d_tmp_data, b_tmp_size, (double *)
                         &b_dv0, errI2C_size);
            VmKp184Test_data[((int)b_meanIbrd0 + (int)Nina219_tmp_tmp *
                              b_loop_ub_tmp) - 1] = d_tmp_data;
            if (b_meanIbrd0 < 2.147483648E+9) {
              i1 = (int)b_meanIbrd0;
            } else {
              i1 = MAX_int32_T;
            }

            c_sprintf(i1, b_loop_ub_tmp + 1, b_k_bat);
            d_sprintf(VmKp184Test_data[((int)b_meanIbrd0 + (int)Nina219_tmp_tmp *
                       b_loop_ub_tmp) - 1], b_VmKp184Test_data);
            pause(0.1);
          }
        }

        /*  for k_bat = 1:N_bat */
        for (i1 = 0; i1 < c_loop_ub; i1++) {
          d_tmp_data = y_data[i1];
          if (d_tmp_data < 256.0) {
            NstTst_tmp_tmp = (unsigned char)d_tmp_data;
          } else {
            NstTst_tmp_tmp = MAX_uint8_T;
          }

          c_y_data[i1] = NstTst_tmp_tmp;
        }

        c_padArrUint8(c_y_data, size_tmp_idx_1, prm->Nmax.NbatMax,
                      prm->Nmax.NbatMax, r3);
        b_CalcPortSpiPrm(prm->brd.spi.disconnect, prm->brd.spi.bypass,
                         prm->brd.spi.PortSpiRow_esp, prm->brd.spi.SwitchMat_esp,
                         prm->brd.spi.PortSpiRow_esp2,
                         prm->brd.spi.SwitchMat_esp2, prm->brd.spi.Pac2Vid,
                         prm->brd.spi.Pac2Vid2, r3, SpiPortRowBypass, a__1_data,
                         a__1_size, id0_data);
        switch (ProjectFlag) {
         case 2:
          /* ESP32 */
          break;

         case 3:
          /* ESP32 ser */
          writePortToSpi4RowMask_ser(SpiPortRowBypass);
          break;
        }

        pause(0.1);
        switch (ProjectFlag) {
         case 2:
          /* ESP32 */
          VmV_size[0] = 16;
          VmV_size[1] = 2;
          memset(&VmV_data[0], 0, 32U * sizeof(double));

          /*  */
          break;

         case 3:
          /* ESP32 ser */
          for (i1 = 0; i1 < 16; i1++) {
            g_prm[i1] = prm->brd.pac.VIpacId[((int)b_meanIbrd0 + (i1 << 1)) - 1];
          }

          c_readI2cVIfastTicTocV5_Rratio_(g_prm, N_bat1, b_VmV_data, b_VmV_size,
            I_data, I_size, tvv_data, tvv_size, c_VmKp184Test_data, tii_size,
            (double *)&N0, a__3_size, Vdebug_data, Vdebug_size);
          VmV_size[0] = Vdebug_size[0];
          VmV_size[1] = Vdebug_size[1];
          loop_ub = Vdebug_size[0] * Vdebug_size[1];
          if (loop_ub - 1 >= 0) {
            memcpy(&VmV_data[0], &Vdebug_data[0], (unsigned int)loop_ub * sizeof
                   (double));
          }

          /*  */
          break;

         default:
          VmV_size[0] = N_bat_tmp_tmp_tmp;
          VmV_size[1] = 1;
          if (N_bat_tmp_tmp_tmp - 1 >= 0) {
            memset(&VmV_data[0], 0, (unsigned int)N_bat_tmp_tmp_tmp * sizeof
                   (double));
          }
          break;
        }

        if (prm->seq.tst.v.ins[k_tstState] == 1) {
          /* kp184 */
          ControlKp184((double *)&N0, a__3_size, (double *)&a__4_data, a__4_size,
                       (double *)&b_dv0, errI2C_size, (double *)&a__16_data,
                       id2Bypass_data);
        }

        /*  [SpiPortRow0,ina219State,Pac2Vid0] = CalcPortSpi(SwitchMat_esp,SwitchMat_esp2,PortSpiRow_esp,PortSpiRow_esp2,Pac2Vid,Pac2Vid2,[1+N_bat/2,N_bat]'); */
        d_padArrUint8(N_bat, prm->Nmax.NbatMax, prm->Nmax.NbatMax, r3);
        b_CalcPortSpiPrm(prm->brd.spi.disconnect, prm->brd.spi.bypass,
                         prm->brd.spi.PortSpiRow_esp, prm->brd.spi.SwitchMat_esp,
                         prm->brd.spi.PortSpiRow_esp2,
                         prm->brd.spi.SwitchMat_esp2, prm->brd.spi.Pac2Vid,
                         prm->brd.spi.Pac2Vid2, r3, SpiPortRowBypass, a__1_data,
                         a__1_size, Pac2Vid0);
        switch (ProjectFlag) {
         case 2:
          /* ESP32 */
          break;

         case 3:
          /* ESP32 ser */
          writePortToSpi4RowMask_ser(SpiPortRowBypass);
          break;
        }

        pause(0.1);
        switch (ProjectFlag) {
         case 2:
          /* ESP32 */
          /*  */
          break;

         case 3:
          /* ESP32 ser */
          for (i1 = 0; i1 < 16; i1++) {
            g_prm[i1] = prm->brd.pac.VIpacId[((int)b_meanIbrd0 + (i1 << 1)) - 1];
          }

          c_readI2cVIfastTicTocV5_Rratio_(g_prm, N_bat1, b_VmV_data, b_VmV_size,
            I_data, I_size, tvv_data, tvv_size, c_VmKp184Test_data, tii_size,
            (double *)&N0, a__3_size, Vdebug_data, Vdebug_size);

          /*  */
          break;
        }

        if (prm->seq.tst.v.isPrm) {
          b_VdebugVec_size[0] = 1;
          b_VdebugVec_size[1] = prm->brd.N_bat;
          b_VdebugVec_size[2] = 2;
          for (i1 = 0; i1 < 2; i1++) {
            for (id2Bypass_size = 0; id2Bypass_size < VdebugVec_size_tmp;
                 id2Bypass_size++) {
              b_VdebugVec_data[id2Bypass_size + VdebugVec_size_tmp * i1] =
                VdebugVec_data[(((int)b_meanIbrd0 + VdebugVec_size[0] *
                                 id2Bypass_size) + VdebugVec_size[0] *
                                VdebugVec_size_tmp * i1) - 1];
            }
          }

          squeeze(b_VdebugVec_data, b_VdebugVec_size, Vdebug_data, Vdebug_size);
          for (i1 = 0; i1 < 16; i1++) {
            g_prm[i1] = prm->brd.pac.VIpacId[((int)b_meanIbrd0 + (i1 << 1)) - 1];
          }

          for (i1 = 0; i1 < N_bat_tmp_tmp_tmp; i1++) {
            c_VmKp184Test_data[i1] = VmKp184Test_data[((int)b_meanIbrd0 + (int)
              Nina219_tmp_tmp * i1) - 1];
          }

          CalcK(VmV_data, VmV_size, Vdebug_data, Vdebug_size, g_prm, &id0_data[0],
                c_VmKp184Test_data, N_bat_tmp_tmp_tmp, b_VdebugVec_data,
                b_tmp_size);
          for (i1 = 0; i1 < 2; i1++) {
            for (id2Bypass_size = 0; id2Bypass_size < VdebugVec_size_tmp;
                 id2Bypass_size++) {
              kEst_data[(((int)b_meanIbrd0 + kEst_size[0] * id2Bypass_size) +
                         kEst_size[0] * VdebugVec_size_tmp * i1) - 1] =
                b_VdebugVec_data[id2Bypass_size + b_tmp_size[0] * i1];
            }
          }
        } else {
          for (i1 = 0; i1 < N_bat_tmp_tmp_tmp; i1++) {
            VmKp184Test_data[((int)b_meanIbrd0 + (int)Nina219_tmp_tmp * i1) - 1]
              = VbusTest_data[((int)b_meanIbrd0 + (int)Nina219_tmp_tmp * i1) - 1];
          }
        }

        /*  Kalman */
        if (kalmanFlag) {
          for (b_loop_ub_tmp = 0; b_loop_ub_tmp < N_bat_tmp_tmp_tmp;
               b_loop_ub_tmp++) {
            for (i1 = 0; i1 < 31; i1++) {
              fv[i1] = prm->klm.BatParamsCell[(int)b_meanIbrd0 - 1]
                .BatStateOrg[b_loop_ub_tmp + (i1 << 4)];
            }

            for (i1 = 0; i1 < 38; i1++) {
              h_prm[i1] = prm->klm.BatParamsCell[(int)b_meanIbrd0 - 1]
                .BatParams[b_loop_ub_tmp + (i1 << 4)];
            }

            CalcItByVout(h_prm, fv, Ta, VmKp184Test_data[((int)b_meanIbrd0 +
              (int)Nina219_tmp_tmp * b_loop_ub_tmp) - 1]);
            for (i1 = 0; i1 < 31; i1++) {
              prm->klm.BatParamsCell[(int)b_meanIbrd0 - 1]
                .BatState[b_loop_ub_tmp + (i1 << 4)] = fv[i1];
            }
          }

          VmKp184Test_size[0] = 1;
          VmKp184Test_size[1] = N_bat_tmp_tmp_tmp;
          for (i1 = 0; i1 < N_bat_tmp_tmp_tmp; i1++) {
            c_VmKp184Test_data[i1] = VmKp184Test_data[((int)b_meanIbrd0 + (int)
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
                      VmKp184Test_size), &prm->klm.b_struct[(int)b_meanIbrd0 - 1]);
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
        b_trueCount = 0;
      } else {
        b_trueCount = prm->brd.N_bat;
      }

      for (i = 0; i < 2; i++) {
        for (i1 = 0; i1 < b_trueCount; i1++) {
          for (i2 = 0; i2 < loop_ub; i2++) {
            prm->brd.pac.Rval[(i2 + (i1 << 1)) + (i << 5)] = (float)kEst_data
              [(i2 + loop_ub * i1) + loop_ub * b_trueCount * i];
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
        writePortToSpi4RowMask_ser(disConAll);
        break;
      }
    }

    if (prm->run.testVreset) {
      for (i = 0; i < loop_ub_tmp; i++) {
        c_VbusTest_data[i] = (VbusTest_data[i] > prm->bat.VresetMax);
      }

      if (any(c_VbusTest_data, loop_ub_tmp)) {
        outStruct->VresetFlag = false;
        if (prm->brd.Nina219 < 1.0) {
          loop_ub = 0;
        } else {
          loop_ub = (int)prm->brd.Nina219;
        }

        Nina219[0] = loop_ub;
        Nina219[1] = outStruct->VbusTest->size[1];
        b_loop_ub = outStruct->VbusTest->size[1];
        for (i = 0; i < b_loop_ub; i++) {
          for (i1 = 0; i1 < loop_ub; i1++) {
            outStruct->VbusTest->data[i1 + outStruct->VbusTest->size[0] * i] =
              VbusTest_data[i1 + Nina219[0] * i];
          }
        }

        Nina219[0] = loop_ub;
        Nina219[1] = outStruct->VmKp184Test->size[1];
        b_loop_ub = outStruct->VmKp184Test->size[1];
        for (i = 0; i < b_loop_ub; i++) {
          for (i1 = 0; i1 < loop_ub; i1++) {
            outStruct->VmKp184Test->data[i1 + outStruct->VmKp184Test->size[0] *
              i] = VmKp184Test_data[i1 + Nina219[0] * i];
          }
        }
      } else {
        for (i = 0; i < loop_ub_tmp; i++) {
          c_VbusTest_data[i] = (VbusTest_data[i] < prm->bat.VresetMax);
        }

        if (all(c_VbusTest_data, loop_ub_tmp)) {
          outStruct->VresetFlag = true;
          if (prm->brd.Nina219 < 1.0) {
            loop_ub = 0;
          } else {
            loop_ub = (int)prm->brd.Nina219;
          }

          Nina219[0] = loop_ub;
          Nina219[1] = outStruct->VbusTest->size[1];
          b_loop_ub = outStruct->VbusTest->size[1];
          for (i = 0; i < b_loop_ub; i++) {
            for (i1 = 0; i1 < loop_ub; i1++) {
              outStruct->VbusTest->data[i1 + outStruct->VbusTest->size[0] * i] =
                VbusTest_data[i1 + Nina219[0] * i];
            }
          }

          Nina219[0] = loop_ub;
          Nina219[1] = outStruct->VmKp184Test->size[1];
          b_loop_ub = outStruct->VmKp184Test->size[1];
          for (i = 0; i < b_loop_ub; i++) {
            for (i1 = 0; i1 < loop_ub; i1++) {
              outStruct->VmKp184Test->data[i1 + outStruct->VmKp184Test->size[0] *
                i] = VmKp184Test_data[i1 + Nina219[0] * i];
            }
          }
        } else {
          outStruct->VresetFlag = false;
          if (prm->brd.Nina219 < 1.0) {
            loop_ub = 0;
          } else {
            loop_ub = (int)prm->brd.Nina219;
          }

          Nina219[0] = loop_ub;
          Nina219[1] = outStruct->VbusTest->size[1];
          b_loop_ub = outStruct->VbusTest->size[1];
          for (i = 0; i < b_loop_ub; i++) {
            for (i1 = 0; i1 < loop_ub; i1++) {
              outStruct->VbusTest->data[i1 + outStruct->VbusTest->size[0] * i] =
                VbusTest_data[i1 + Nina219[0] * i];
            }
          }

          Nina219[0] = loop_ub;
          Nina219[1] = outStruct->VmKp184Test->size[1];
          b_loop_ub = outStruct->VmKp184Test->size[1];
          for (i = 0; i < b_loop_ub; i++) {
            for (i1 = 0; i1 < loop_ub; i1++) {
              outStruct->VmKp184Test->data[i1 + outStruct->VmKp184Test->size[0] *
                i] = VmKp184Test_data[i1 + Nina219[0] * i];
            }
          }
        }
      }
    } else {
      for (i = 0; i < loop_ub_tmp; i++) {
        c_VbusTest_data[i] = (VbusTest_data[i] > prm->bat.Vmax);
      }

      if (any(c_VbusTest_data, loop_ub_tmp)) {
        guard2 = true;
      } else {
        for (i = 0; i < loop_ub_tmp; i++) {
          c_VbusTest_data[i] = (VbusTest_data[i] < prm->bat.Vmin);
        }

        if (any(c_VbusTest_data, loop_ub_tmp)) {
          guard2 = true;
        } else {
          for (i = 0; i < loop_ub_tmp; i++) {
            c_VbusTest_data[i] = (VmKp184Test_data[i] > prm->bat.Vmax);
          }

          if (any(c_VbusTest_data, loop_ub_tmp)) {
            guard2 = true;
          } else {
            for (i = 0; i < loop_ub_tmp; i++) {
              c_VbusTest_data[i] = (VmKp184Test_data[i] < prm->bat.Vmin);
            }

            if (any(c_VbusTest_data, loop_ub_tmp)) {
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
        c_loop_ub = prm->brd.N_bat;
      }

      if (VbusTest_size_idx_0 - 1 >= 0) {
        VmKp184Test_size_idx_0 = (int)Nina219_tmp_tmp;
        VmKp184Test_size_idx_1 = N_bat_tmp_tmp_tmp;
        VmKp184Test_size[0] = 1;
        VmKp184Test_size[1] = N_bat_tmp_tmp_tmp;
        for (i = 0; i < b_loop_ub; i++) {
          VmKp184Test_data[i] = 4.0;
        }

        for (i = 0; i < c_loop_ub; i++) {
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
      VdebugVec_size_tmp = prm->seq.tst.i.Nst;
      VbusTest2->size[3] = prm->seq.tst.i.Nst;
      emxEnsureCapacity_real_T(VbusTest2, i);
      VbusTest2_data = VbusTest2->data;
      partialTrueCount = (int)prm->brd.Nina219 * prm->brd.N_bat;
      b_loop_ub_tmp = partialTrueCount * prm->seq.tst.i.NTtest *
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

      Rtot_size[0] = (int)prm->brd.Nina219;
      Rtot_size[1] = prm->brd.N_bat;
      Rwire_size[0] = (int)prm->brd.Nina219;
      Rwire_size[1] = prm->brd.N_bat;
      if (partialTrueCount - 1 >= 0) {
        memset(&Rtot_data[0], 0, (unsigned int)partialTrueCount * sizeof(double));
        memset(&Rwire_data[0], 0, (unsigned int)partialTrueCount * sizeof(float));
      }

      if (prm->brd.Nina219 < 1.0) {
        y->size[0] = 1;
        y->size[1] = 0;
      } else {
        i = y->size[0] * y->size[1];
        y->size[0] = 1;
        y->size[1] = (int)(prm->brd.Nina219 - 1.0) + 1;
        emxEnsureCapacity_real_T(y, i);
        y_data = y->data;
        loop_ub = (int)(prm->brd.Nina219 - 1.0);
        for (i = 0; i <= loop_ub; i++) {
          y_data[i] = (double)i + 1.0;
        }
      }

      i = b_y->size[0];
      b_y->size[0] = y->size[1];
      emxEnsureCapacity_real_T(b_y, i);
      b_y_data = b_y->data;
      loop_ub = y->size[1];
      for (i = 0; i < loop_ub; i++) {
        b_y_data[i] = y_data[i];
      }

      loop_ub = y->size[1];
      for (i = 0; i < loop_ub; i++) {
        VecIna219_data[i] = b_y_data[i];
      }

      for (k_tstState = 0; k_tstState < VdebugVec_size_tmp; k_tstState++) {
        pause(0.1);
        if (Nina219_tmp_tmp < 1.0) {
          i = 0;
        } else {
          i = (int)Nina219_tmp_tmp;
        }

        for (k0 = 0; k0 < i; k0++) {
          k_ina219_ = k_tstState << 3;
          k_groups_tmp = prm->seq.tst.i.grp[k0 + k_ina219_];

          /* 1:length(uGroups) %seqTstI.grp(k_tstState)%1:length(uGroups) */
          /* seqTstI.ItestSwitch{k_groups,k_tstState};%[1:N_bat]'; */
          i1 = ((k_groups_tmp - 1) << 8) + (k_tstState << 11);
          c_CalcPortSpiPrm(prm->brd.spi.disconnect, prm->brd.spi.bypass,
                           prm->brd.spi.PortSpiRow_esp,
                           prm->brd.spi.SwitchMat_esp,
                           prm->brd.spi.PortSpiRow_esp2,
                           prm->brd.spi.SwitchMat_esp2, prm->brd.spi.Pac2Vid,
                           prm->brd.spi.Pac2Vid2, &prm->seq.tst.i.ItestSwitch[i1],
                           SpiPortRowBypass, a__1_data, a__1_size, id0_data);

          /*  I2switchId = ina219State; */
          /*  calc V */
          /* squeeze(Pac2Vid0All(k_ina219,:,:));%Pac2Vid3(:,1); */
          trueCount = 0;
          for (loop_ub = 0; loop_ub < 256; loop_ub++) {
            if (id0_data[loop_ub] > 0) {
              trueCount++;
            }
          }

          c_loop_ub = (k_groups_tmp + k_ina219_) - 1;
          partialTrueCount = (unsigned short)trueCount;
          if (partialTrueCount < 1) {
            partialTrueCount = 1;
          }

          if ((unsigned short)trueCount == 0) {
            partialTrueCount = 0;
          }

          c_vSumMax = prm->bat.Vd + ((prm->bat.CutOffChrV[0] + 0.1F) + maximum
            (prm->seq.tst.i.i_in_test) * prm->seq.tst.i.Rin[c_loop_ub]) * (float)
            partialTrueCount;
          id2Bypass_size = ipos_size - 1;
          trueCount = 0;
          partialTrueCount = 0;

          /* VecIna219_k = find(k_groups == uGroupId); */
          b_trueCount = 0;
          for (loop_ub = 0; loop_ub <= id2Bypass_size; loop_ub++) {
            i2 = ipos_data[loop_ub];
            if (k_groups_tmp == i2) {
              trueCount++;
              e_tmp_data[partialTrueCount] = (signed char)loop_ub;
              partialTrueCount++;
            }

            if (k_groups_tmp != i2) {
              b_trueCount++;
            }
          }

          for (k1 = 0; k1 < b_trueCount; k1++) {
            /* diconnect group that not tested */
            switch (ProjectFlag) {
             case 2:
              /* ESP32 */
              break;

             case 3:
              /* ESP32 ser */
              writePortToSpi4RowMask_ser(disConAll);
              break;
            }

            pause(0.1);
          }

          for (k1 = 0; k1 < trueCount; k1++) {
            b_meanIbrd0 = VecIna219_data[e_tmp_data[k1]];
            if (prm->seq.tst.i.BrdBeforePSflag[c_loop_ub] != 0) {
              for (i2 = 0; i2 < trueCount; i2++) {
                b_VecIna219_data[i2] = (VecIna219_data[e_tmp_data[i2]] !=
                  b_meanIbrd0);
              }

              id2Bypass_size = c_eml_find(b_VecIna219_data, trueCount,
                id2Bypass_data);
              for (k_ina219_ = 0; k_ina219_ < id2Bypass_size; k_ina219_++) {
                /* k_bypass = VecIna219_k(id2Bypass) */
                CalcPortSpiPrm(prm->brd.spi.disconnect, prm->brd.spi.bypass,
                               prm->brd.spi.PortSpiRow_esp,
                               prm->brd.spi.SwitchMat_esp,
                               prm->brd.spi.PortSpiRow_esp2,
                               prm->brd.spi.SwitchMat_esp2, prm->brd.spi.Pac2Vid,
                               prm->brd.spi.Pac2Vid2, prm->brd.spi.bypass,
                               SpiPortRowBypass, a__1_data, a__1_size, id0_data);

                /*  ina219StateAll(:,k_bypass)   = ina219StateBypass; */
                /*  BattConfig{k_bypass}       = prm.brd.spi.bypass;%-2; */
                /*  BattConfigPerIna{k_bypass} = prm.brd.spi.bypass;%-2; */
                d_tmp_data = VecIna219_data[e_tmp_data[id2Bypass_data[k_ina219_]
                  - 1]];
                if (d_tmp_data < 2.147483648E+9) {
                  i2 = (int)d_tmp_data;
                } else {
                  i2 = MAX_int32_T;
                }

                b_sprintf(i2, r7);
                if (prm->seq.tst.i.measRintR) {
                  for (i2 = 0; i2 < 24; i2++) {
                    SpiPortRowBypass[i2] = disConAll[i2];
                  }
                }

                switch (ProjectFlag) {
                 case 2:
                  /* ESP32 */
                  /* TODO bypass */
                  break;

                 case 3:
                  /* ESP32 ser */
                  writePortToSpi4RowMask_ser(SpiPortRowBypass);

                  /* TODO bypass */
                  break;
                }

                pause(0.1);
              }

              c_CalcPortSpiPrm(prm->brd.spi.disconnect, prm->brd.spi.bypass,
                               prm->brd.spi.PortSpiRow_esp,
                               prm->brd.spi.SwitchMat_esp,
                               prm->brd.spi.PortSpiRow_esp2,
                               prm->brd.spi.SwitchMat_esp2, prm->brd.spi.Pac2Vid,
                               prm->brd.spi.Pac2Vid2,
                               &prm->seq.tst.i.ItestSwitch[i1], SpiPortRowBypass,
                               a__1_data, a__1_size, id0_data);

              /*  ina219StateAll(:,k_ina219)   = ina219State; */
              /*  BattConfig{k_ina219}       = iTestSwitchCell_1+(k_ina219-1)*N_bat; */
              /*  BattConfigPerIna{k_ina219} = iTestSwitchCell_1; */
            }

            if (b_meanIbrd0 < 2.147483648E+9) {
              i2 = (int)b_meanIbrd0;
            } else {
              i2 = MAX_int32_T;
            }

            b_sprintf(i2, r6);
            partialTrueCount = a__1_size[0];
            if (partialTrueCount < 1) {
              partialTrueCount = 1;
            }

            if (a__1_size[0] == 0) {
              k_ina219_ = 0;
            } else {
              k_ina219_ = partialTrueCount;
            }

            for (id2Bypass_size = 0; id2Bypass_size < k_ina219_; id2Bypass_size
                 ++) {
              d_tmp_data = rt_roundd_snf(a__1_data[id2Bypass_size]);
              if (d_tmp_data < 2.147483648E+9) {
                if (d_tmp_data >= -2.147483648E+9) {
                  i2 = (int)d_tmp_data;
                } else {
                  i2 = MIN_int32_T;
                }
              } else if (d_tmp_data >= 2.147483648E+9) {
                i2 = MAX_int32_T;
              } else {
                i2 = 0;
              }

              printf("%d \n", i2);
              fflush(stdout);
            }

            if (prm->seq.tst.i.measRintR) {
              for (i2 = 0; i2 < 24; i2++) {
                SpiPortRowBypass[i2] = disConAll[i2];
              }
            }

            switch (ProjectFlag) {
             case 2:
              /* esp32 */
              break;

             case 3:
              /* esp32 ser */
              writePortToSpi4RowMask_ser(SpiPortRowBypass);
              break;
            }

            /*  end */
            i2 = prm->seq.tst.i.NTtest;
            if (i2 - 1 >= 0) {
              if (b_meanIbrd0 < 2.147483648E+9) {
                k_t0 = (int)b_meanIbrd0;
                state_k = (int)b_meanIbrd0;
              } else {
                k_t0 = MAX_int32_T;
                state_k = MAX_int32_T;
              }
            }

            for (b_trueCount = 0; b_trueCount < i2; b_trueCount++) {
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
                f_sprintf(rt_roundf_snf(prm->seq.tst.i.i_in_test[b_trueCount]),
                          b_prm);

                /* ['ISET',num2str(ch),':',num2str(Val)];%[A] */
                pause(0.05);
                pause(1.0);
                break;

               case 3:
                /* juntek+ACDC */
                /* Imax); */
                i_prm[0] = prm->seq.tst.i.i_in_test[b_trueCount];
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

                g_sprintf(i3, r8);

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

                g_sprintf(i3, r9);

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
              if (prm->seq.tst.i.BrdBeforePSflag[c_loop_ub] == 0) {
                for (i3 = 0; i3 < trueCount; i3++) {
                  b_VecIna219_data[i3] = (VecIna219_data[e_tmp_data[i3]] !=
                    b_meanIbrd0);
                }

                id2Bypass_size = c_eml_find(b_VecIna219_data, trueCount,
                  id2Bypass_data);
                for (k_ina219_ = 0; k_ina219_ < id2Bypass_size; k_ina219_++) {
                  CalcPortSpiPrm(prm->brd.spi.disconnect, prm->brd.spi.bypass,
                                 prm->brd.spi.PortSpiRow_esp,
                                 prm->brd.spi.SwitchMat_esp,
                                 prm->brd.spi.PortSpiRow_esp2,
                                 prm->brd.spi.SwitchMat_esp2,
                                 prm->brd.spi.Pac2Vid, prm->brd.spi.Pac2Vid2,
                                 prm->brd.spi.bypass, SpiPortRowBypass,
                                 a__1_data, a__1_size, id0_data);

                  /*  ina219StateAll(:,k_bypass)   = ina219StateBypass; */
                  /*  BattConfig{k_bypass}       = prm.brd.spi.bypass;%-2; */
                  /*  BattConfigPerIna{k_bypass} = prm.brd.spi.bypass;%-2; */
                  d_tmp_data =
                    VecIna219_data[e_tmp_data[id2Bypass_data[k_ina219_] - 1]];
                  if (d_tmp_data < 2.147483648E+9) {
                    i3 = (int)d_tmp_data;
                  } else {
                    i3 = MAX_int32_T;
                  }

                  b_sprintf(i3, r10);
                  if (prm->seq.tst.i.measRintR) {
                    for (i3 = 0; i3 < 24; i3++) {
                      SpiPortRowBypass[i3] = disConAll[i3];
                    }
                  }

                  switch (ProjectFlag) {
                   case 2:
                    /* ESP32 */
                    /* TODO bypass */
                    break;

                   case 3:
                    /* ESP32 ser */
                    writePortToSpi4RowMask_ser(SpiPortRowBypass);

                    /* TODO bypass */
                    break;
                  }

                  pause(0.1);
                }

                c_CalcPortSpiPrm(prm->brd.spi.disconnect, prm->brd.spi.bypass,
                                 prm->brd.spi.PortSpiRow_esp,
                                 prm->brd.spi.SwitchMat_esp,
                                 prm->brd.spi.PortSpiRow_esp2,
                                 prm->brd.spi.SwitchMat_esp2,
                                 prm->brd.spi.Pac2Vid, prm->brd.spi.Pac2Vid2,
                                 &prm->seq.tst.i.ItestSwitch[i1],
                                 SpiPortRowBypass, a__1_data, a__1_size,
                                 id0_data);

                /*  ina219StateAll(:,k_ina219)   = ina219State; */
                /*  BattConfig{k_ina219}       = iTestSwitchCell_1+(k_ina219-1)*N_bat; */
                /*  BattConfigPerIna{k_ina219} = iTestSwitchCell_1; */
              }

              pause(0.5);
              switch (ProjectFlag) {
               case 2:
                /* esp32 */
                I_size[0] = 16;
                I_size[1] = 1;
                tvv_size[0] = 16;
                tvv_size[1] = 1;
                memset(&VmV_data[0], 0, 16U * sizeof(double));
                memset(&I_data[0], 0, 16U * sizeof(double));
                memset(&tvv_data[0], 0, 16U * sizeof(double));
                break;

               case 3:
                /* esp32 ser */
                for (i3 = 0; i3 < 16; i3++) {
                  g_prm[i3] = prm->brd.pac.VIpacId[((int)b_meanIbrd0 + (i3 << 1))
                    - 1];
                }

                c_readI2cVIfastTicTocV5_Rratio_(g_prm, N_bat1, b_VmV_data,
                  b_VmV_size, I_data, I_size, tvv_data, tvv_size,
                  c_VmKp184Test_data, tii_size, (double *)&b_dv0, errI2C_size,
                  Vdebug_data, Vdebug_size);
                loop_ub = b_VmV_size[0] * b_VmV_size[1];
                if (loop_ub - 1 >= 0) {
                  memcpy(&VmV_data[0], &b_VmV_data[0], (unsigned int)loop_ub *
                         sizeof(double));
                }
                break;

               default:
                I_size[0] = N_bat_tmp_tmp_tmp;
                I_size[1] = 1;
                tvv_size[0] = N_bat_tmp_tmp_tmp;
                tvv_size[1] = 1;
                if (N_bat_tmp_tmp_tmp - 1 >= 0) {
                  memset(&VmV_data[0], 0, (unsigned int)N_bat_tmp_tmp_tmp *
                         sizeof(double));
                  memset(&I_data[0], 0, (unsigned int)N_bat_tmp_tmp_tmp * sizeof
                         (double));
                  memset(&tvv_data[0], 0, (unsigned int)N_bat_tmp_tmp_tmp *
                         sizeof(double));
                }
                break;
              }

              Nina219[0] = 1;
              Nina219[1] = VbusTest2->size[1];
              loop_ub = VbusTest2->size[1];
              for (i3 = 0; i3 < loop_ub; i3++) {
                VbusTest2_data[((((int)b_meanIbrd0 + VbusTest2->size[0] * i3) +
                                 VbusTest2->size[0] * VbusTest2->size[1] *
                                 b_trueCount) + VbusTest2->size[0] *
                                VbusTest2->size[1] * VbusTest2->size[2] *
                                k_tstState) - 1] = VmV_data[i3];
              }

              Nina219[0] = 1;
              Nina219[1] = IshuntTest2->size[1];
              loop_ub = IshuntTest2->size[1];
              for (i3 = 0; i3 < loop_ub; i3++) {
                IshuntTest2_data[((((int)b_meanIbrd0 + IshuntTest2->size[0] * i3)
                                   + IshuntTest2->size[0] * IshuntTest2->size[1]
                                   * b_trueCount) + IshuntTest2->size[0] *
                                  IshuntTest2->size[1] * IshuntTest2->size[2] *
                                  k_tstState) - 1] = I_data[i3];
              }

              switch (prm->seq.tst.i.meas[(k_groups_tmp + (k_tstState << 3)) - 1])
              {
               case 1:
                /* kp184 */
                ControlKp184((double *)&N0, a__3_size, (double *)&a__4_data,
                             a__4_size, (double *)&d_tmp_data, b_tmp_size,
                             (double *)&a__16_data, id2Bypass_data);
                ImKp184Test2_data[(((int)b_meanIbrd0 + ImKp184Test2->size[0] *
                                    ImKp184Test2->size[1] * b_trueCount) +
                                   ImKp184Test2->size[0] * ImKp184Test2->size[1]
                                   * ImKp184Test2->size[2] * k_tstState) - 1] =
                  a__16_data;
                break;

               case 2:
                /* ka6005p */
                /* read I */
                d_tmp_data = d_rand();
                ImKp184Test2_data[(((int)b_meanIbrd0 + ImKp184Test2->size[0] *
                                    ImKp184Test2->size[1] * b_trueCount) +
                                   ImKp184Test2->size[0] * ImKp184Test2->size[1]
                                   * ImKp184Test2->size[2] * k_tstState) - 1] =
                  d_tmp_data * 5.0;

                /* read I */
                d_rand();
                break;

               case 3:
                /* juntek+ACDC */
                controlJuntekDPH8920((double *)&d_tmp_data, b_tmp_size);
                ImKp184Test2_data[(((int)b_meanIbrd0 + ImKp184Test2->size[0] *
                                    ImKp184Test2->size[1] * b_trueCount) +
                                   ImKp184Test2->size[0] * ImKp184Test2->size[1]
                                   * ImKp184Test2->size[2] * k_tstState) - 1] =
                  d_tmp_data;
                b_controlJuntekDPH8920(r30.data, r30.size);
                break;

               case 5:
                /*  switch out to load */
                /* []; */
                d_tmp_data = d_rand() * 2.5 + 2.0;
                b_dv0 = d_rand() * 5.0;
                if (prm->ins.prm.swOut.IfromVdivR_flag) {
                  if (prm->seq.tst.i.i_in_test[b_trueCount] == 0.0F) {
                    ImKp184Test2_data[(((int)b_meanIbrd0 + ImKp184Test2->size[0]
                                        * ImKp184Test2->size[1] * b_trueCount) +
                                       ImKp184Test2->size[0] *
                                       ImKp184Test2->size[1] *
                                       ImKp184Test2->size[2] * k_tstState) - 1] =
                      0.0;
                  } else {
                    ImKp184Test2_data[(((int)b_meanIbrd0 + ImKp184Test2->size[0]
                                        * ImKp184Test2->size[1] * b_trueCount) +
                                       ImKp184Test2->size[0] *
                                       ImKp184Test2->size[1] *
                                       ImKp184Test2->size[2] * k_tstState) - 1] =
                      (float)d_tmp_data / prm->seq.tst.i.Rload;

                    /* seqTstI.i_in_test(k_Ttest); */
                  }
                } else {
                  ImKp184Test2_data[(((int)b_meanIbrd0 + ImKp184Test2->size[0] *
                                      ImKp184Test2->size[1] * b_trueCount) +
                                     ImKp184Test2->size[0] * ImKp184Test2->size
                                     [1] * ImKp184Test2->size[2] * k_tstState) -
                    1] = b_dv0;
                }
                break;
              }

              /*  formatSpec0 = '%.3g'; */
              /*  formatSpec1 = '%.5g'; */
              /*                      disp(['V(',pad(num2str([k_ina219,k_Ttest,k_tstState]),'left',padLeft),')=',pad(num2str(VbusTest2(k_ina219,:,k_Ttest,k_tstState)),padRight,'left',padLeft),'V ',newline ,... */
              /*                          'I(',pad(num2str([k_ina219,k_Ttest,k_tstState]),'left',padLeft),')=',pad(num2str(IshuntTest2(k_ina219,:,k_Ttest,k_tstState)),padRight,'left',padLeft),'A ']); */
              h_sprintf(k_t0, b_trueCount + 1, k_tstState + 1, k_Ttest);
              for (b_loop_ub_tmp = 0; b_loop_ub_tmp < N_bat_tmp_tmp_tmp;
                   b_loop_ub_tmp++) {
                i_sprintf(VbusTest2_data[((((int)b_meanIbrd0 + VbusTest2->size[0]
                  * b_loop_ub_tmp) + VbusTest2->size[0] * VbusTest2->size[1] *
                            b_trueCount) + VbusTest2->size[0] * VbusTest2->size
                           [1] * VbusTest2->size[2] * k_tstState) - 1],
                          b_VbusTest2);
              }

              h_sprintf(state_k, b_trueCount + 1, k_tstState + 1, b_k_Ttest);
              for (b_loop_ub_tmp = 0; b_loop_ub_tmp < N_bat_tmp_tmp_tmp;
                   b_loop_ub_tmp++) {
                i_sprintf(IshuntTest2_data[((((int)b_meanIbrd0 +
                  IshuntTest2->size[0] * b_loop_ub_tmp) + IshuntTest2->size[0] *
                            IshuntTest2->size[1] * b_trueCount) +
                           IshuntTest2->size[0] * IshuntTest2->size[1] *
                           IshuntTest2->size[2] * k_tstState) - 1],
                          b_IshuntTest2);

                /* nadav Coder */
              }

              pause(0.1);
              if (Iacs758Flag == 1) {
                if (ProjectFlag == 2) {
                  /* esp32 */
                  Iacs758_cal_data[(((int)b_meanIbrd0 + Iacs758_cal->size[0] *
                                     Iacs758_cal->size[1] * b_trueCount) +
                                    Iacs758_cal->size[0] * Iacs758_cal->size[1] *
                                    Iacs758_cal->size[2] * k_tstState) - 1] =
                    0.0;
                }

                b_dv0 = Iacs758_cal_data[(((int)b_meanIbrd0 + Iacs758_cal->size
                  [0] * Iacs758_cal->size[1] * b_trueCount) + Iacs758_cal->size
                  [0] * Iacs758_cal->size[1] * Iacs758_cal->size[2] * k_tstState)
                  - 1];
                Nina219[0] = I_size[0];
                Nina219[1] = I_size[1];
                loop_ub = Nina219[0] * Nina219[1];
                for (i3 = 0; i3 < loop_ub; i3++) {
                  I_data[i3] = b_dv0;
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
                  Iacs758_cal_data[(((int)b_meanIbrd0 + Iacs758_cal->size[0] *
                                     Iacs758_cal->size[1] * b_trueCount) +
                                    Iacs758_cal->size[0] * Iacs758_cal->size[1] *
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
                  b_dv0 = Iacs758_cal_data[(((int)b_meanIbrd0 +
                    Iacs758_cal->size[0] * Iacs758_cal->size[1] * b_trueCount) +
                    Iacs758_cal->size[0] * Iacs758_cal->size[1] *
                    Iacs758_cal->size[2] * k_tstState) - 1];
                  Nina219[0] = I_size[0];
                  Nina219[1] = I_size[1];
                  loop_ub = Nina219[0] * Nina219[1];
                  for (i3 = 0; i3 < loop_ub; i3++) {
                    I_data[i3] = b_dv0;
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
                CalcStructKalman(&prm->klm.b_struct[(int)b_meanIbrd0 - 1], Ta,
                                 tvv_data, tvv_size, I_data, VmV_data);
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
              writePortToSpi4RowMask_ser(disConAll);
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
      /*      coder.varsize('ImKp184Test2_bat_0','Ishunt_cal_bat_0','Vbus_cal_bat_0',[Nmax.NbitCnfgMax*Nmax.NstateMax 1],[1,1]); */
      i = (int)prm->brd.Nina219;
      kEst_size[0] = (int)prm->brd.Nina219;
      kEst_size[1] = prm->brd.N_bat;
      kEst_size[2] = 2;
      b_loop_ub_tmp = ((int)prm->brd.Nina219 * prm->brd.N_bat) << 1;
      VdebugVec_size[0] = (int)prm->brd.Nina219;
      if (b_loop_ub_tmp - 1 >= 0) {
        memset(&kEst_data[0], 0, (unsigned int)b_loop_ub_tmp * sizeof(double));
        memset(&VdebugVec_data[0], 0, (unsigned int)b_loop_ub_tmp * sizeof
               (double));
      }

      pIacs758_size[0] = (int)prm->brd.Nina219;
      pIacs758_size[1] = 1;
      pIacs758_size[2] = 2;
      loop_ub = (int)prm->brd.Nina219 << 1;
      if (loop_ub - 1 >= 0) {
        memset(&pIacs758_data[0], 0, (unsigned int)loop_ub * sizeof(float));
      }

      if (!prm->seq.tst.i.measRintR) {
        if (i - 1 >= 0) {
          if (N_bat_tmp_tmp_tmp - 1 >= 0) {
            end = NstTst_tmp_tmp - 1;
            tmp_size[0] = 1;
          }

          d_loop_ub = Rtot_size[1];
        }

        for (k_ina219_ = 0; k_ina219_ < i; k_ina219_++) {
          VbusTest2_size[0] = 1;
          VbusTest2_size[1] = 1;
          VbusTest2_size[2] = Iacs758_cal->size[2];
          loop_ub = Iacs758_cal->size[3];
          VbusTest2_size[3] = Iacs758_cal->size[3];
          for (i1 = 0; i1 < loop_ub; i1++) {
            b_loop_ub = Iacs758_cal->size[2];
            for (i2 = 0; i2 < b_loop_ub; i2++) {
              b_VbusTest2_data[i2 + VbusTest2_size[2] * i1] = Iacs758_cal_data
                [(k_ina219_ + Iacs758_cal->size[0] * Iacs758_cal->size[1] * i2)
                + Iacs758_cal->size[0] * Iacs758_cal->size[1] *
                Iacs758_cal->size[2] * i1];
            }
          }

          b_squeeze(b_VbusTest2_data, VbusTest2_size, r1);
          b_y_data = r1->data;
          b_loop_ub_tmp = r1->size[0] * r1->size[1];
          for (i1 = 0; i1 < b_loop_ub_tmp; i1++) {
            VmV_data[i1] = b_y_data[i1];
          }

          VbusTest2_size[0] = 1;
          VbusTest2_size[1] = 1;
          VbusTest2_size[2] = ImKp184Test2->size[2];
          loop_ub = ImKp184Test2->size[3];
          VbusTest2_size[3] = ImKp184Test2->size[3];
          for (i1 = 0; i1 < loop_ub; i1++) {
            b_loop_ub = ImKp184Test2->size[2];
            for (i2 = 0; i2 < b_loop_ub; i2++) {
              b_VbusTest2_data[i2 + VbusTest2_size[2] * i1] = ImKp184Test2_data
                [(k_ina219_ + ImKp184Test2->size[0] * ImKp184Test2->size[1] * i2)
                + ImKp184Test2->size[0] * ImKp184Test2->size[1] *
                ImKp184Test2->size[2] * i1];
            }
          }

          b_squeeze(b_VbusTest2_data, VbusTest2_size, r1);
          b_y_data = r1->data;
          if (r1->size[1] == NstTst_tmp_tmp) {
            Vdebug_size[0] = r1->size[0];
            Vdebug_size[1] = r1->size[1];
            loop_ub = r1->size[1];
            for (i1 = 0; i1 < loop_ub; i1++) {
              b_loop_ub = r1->size[0];
              for (i2 = 0; i2 < b_loop_ub; i2++) {
                Vdebug_data[i2 + Vdebug_size[0] * i1] = b_y_data[i2 + r1->size[0]
                  * i1] * (double)prm->seq.tst.i.NegIflag[(ipos_data[k_ina219_]
                  + (i1 << 3)) - 1];
              }
            }
          } else {
            binary_expand_op_2(Vdebug_data, Vdebug_size, r1, prm, ipos_data,
                               k_ina219_, NstTst_tmp_tmp - 1);
          }

          polyfit(VmV_data, b_loop_ub_tmp, Vdebug_data, Vdebug_size[0] *
                  Vdebug_size[1], outVI);
          pIacs758_data[k_ina219_] = (float)outVI[0];
          pIacs758_data[k_ina219_ + pIacs758_size[0]] = (float)outVI[1];

          /* polyfit(squeeze(IshuntTest2(k_ina219,k_bat,:,k_iTest)),1000*squeeze(ImKp184Test2(k_ina219,1,:,k_iTest)),1); */
          for (b_loop_ub_tmp = 0; b_loop_ub_tmp < N_bat_tmp_tmp_tmp;
               b_loop_ub_tmp++) {
            for (k_tstState = 0; k_tstState < VdebugVec_size_tmp; k_tstState++)
            {
              ismember0_data[k_tstState] = isMember((double)b_loop_ub_tmp + 1.0,
                &prm->seq.tst.i.ItestSwitch[((ipos_data[k_ina219_] - 1) << 8) +
                (k_tstState << 11)]);
            }

            trueCount = 0;
            partialTrueCount = 0;
            for (loop_ub = 0; loop_ub <= end; loop_ub++) {
              if (ismember0_data[loop_ub]) {
                trueCount++;
                g_tmp_data[partialTrueCount] = (signed char)loop_ub;
                partialTrueCount++;
              }
            }

            IshuntTest2_size[0] = 1;
            IshuntTest2_size[1] = 1;
            IshuntTest2_size[2] = IshuntTest2->size[2];
            IshuntTest2_size[3] = trueCount;
            for (i1 = 0; i1 < trueCount; i1++) {
              loop_ub = IshuntTest2->size[2];
              for (i2 = 0; i2 < loop_ub; i2++) {
                b_IshuntTest2_data[i2 + IshuntTest2_size[2] * i1] =
                  IshuntTest2_data[((k_ina219_ + IshuntTest2->size[0] *
                                     b_loop_ub_tmp) + IshuntTest2->size[0] *
                                    IshuntTest2->size[1] * i2) +
                  IshuntTest2->size[0] * IshuntTest2->size[1] *
                  IshuntTest2->size[2] * g_tmp_data[i1]];
              }
            }

            b_squeeze(b_IshuntTest2_data, IshuntTest2_size, Ishunt_cal_bat_00);
            IshuntTest2_size[0] = 1;
            IshuntTest2_size[1] = 1;
            IshuntTest2_size[2] = VbusTest2->size[2];
            IshuntTest2_size[3] = trueCount;
            for (i1 = 0; i1 < trueCount; i1++) {
              loop_ub = VbusTest2->size[2];
              for (i2 = 0; i2 < loop_ub; i2++) {
                b_IshuntTest2_data[i2 + IshuntTest2_size[2] * i1] =
                  VbusTest2_data[((k_ina219_ + VbusTest2->size[0] *
                                   b_loop_ub_tmp) + VbusTest2->size[0] *
                                  VbusTest2->size[1] * i2) + VbusTest2->size[0] *
                  VbusTest2->size[1] * VbusTest2->size[2] * g_tmp_data[i1]];
              }
            }

            b_squeeze(b_IshuntTest2_data, IshuntTest2_size, Vbus_cal_bat_00);
            IshuntTest2_size[0] = 1;
            IshuntTest2_size[1] = 1;
            IshuntTest2_size[2] = ImKp184Test2->size[2];
            IshuntTest2_size[3] = trueCount;
            tmp_size[1] = trueCount;
            for (i1 = 0; i1 < trueCount; i1++) {
              loop_ub = ImKp184Test2->size[2];
              for (i2 = 0; i2 < loop_ub; i2++) {
                b_IshuntTest2_data[i2 + IshuntTest2_size[2] * i1] =
                  ImKp184Test2_data[(k_ina219_ + ImKp184Test2->size[0] *
                                     ImKp184Test2->size[1] * i2) +
                  ImKp184Test2->size[0] * ImKp184Test2->size[1] *
                  ImKp184Test2->size[2] * g_tmp_data[i1]];
              }

              h_tmp_data[i1] = prm->seq.tst.i.NegIflag[(ipos_data[k_ina219_] +
                (g_tmp_data[i1] << 3)) - 1];
            }

            b_squeeze(b_IshuntTest2_data, IshuntTest2_size, r1);
            b_y_data = r1->data;
            if (r1->size[1] == trueCount) {
              ImKp184Test2_bat_00_size[0] = r1->size[0];
              ImKp184Test2_bat_00_size[1] = r1->size[1];
              loop_ub = r1->size[1];
              for (i1 = 0; i1 < loop_ub; i1++) {
                b_loop_ub = r1->size[0];
                for (i2 = 0; i2 < b_loop_ub; i2++) {
                  ImKp184Test2_bat_00_data[i2 + ImKp184Test2_bat_00_size[0] * i1]
                    = b_y_data[i2 + r1->size[0] * i1] * (double)h_tmp_data[i1];
                }
              }
            } else {
              binary_expand_op(ImKp184Test2_bat_00_data,
                               ImKp184Test2_bat_00_size, r1, h_tmp_data,
                               tmp_size);
            }

            id2Bypass_size = ImKp184Test2_bat_00_size[0] *
              ImKp184Test2_bat_00_size[1];
            trueCount = Vbus_cal_bat_00->size[0] * Vbus_cal_bat_00->size[1];
            polyfit(ImKp184Test2_bat_00_data, id2Bypass_size, (double *)
                    Vbus_cal_bat_00->data, trueCount, outVI);
            b_trueCount = k_ina219_ + VdebugVec_size[0] * b_loop_ub_tmp;
            VdebugVec_data[b_trueCount] = outVI[0];
            VdebugVec_data[b_trueCount + VdebugVec_size[0] * prm->brd.N_bat] =
              outVI[1];
            trueCount = Ishunt_cal_bat_00->size[0] * Ishunt_cal_bat_00->size[1];
            polyfit((double *)Ishunt_cal_bat_00->data, trueCount,
                    ImKp184Test2_bat_00_data, id2Bypass_size, outVI);
            b_trueCount = k_ina219_ + kEst_size[0] * b_loop_ub_tmp;
            kEst_data[b_trueCount] = outVI[0];
            kEst_data[b_trueCount + kEst_size[0] * kEst_size[1]] = outVI[1];

            /* polyfit(squeeze(IshuntTest2(k_ina219,k_bat,:,k_iTest)),1000*squeeze(ImKp184Test2(k_ina219,1,:,k_iTest)),1); */
          }

          /*  end */
          for (i1 = 0; i1 < d_loop_ub; i1++) {
            Rtot_data[k_ina219_ + Rtot_size[0] * i1] = -VdebugVec_data[k_ina219_
              + VdebugVec_size[0] * i1];
          }

          if (Rtot_size[1] == 16) {
            loop_ub = Rwire_size[1];
            for (i1 = 0; i1 < loop_ub; i1++) {
              Rwire_data[k_ina219_ + Rwire_size[0] * i1] = (float)
                Rtot_data[k_ina219_ + Rtot_size[0] * i1] - prm->
                bat.Rint[k_ina219_ + (i1 << 1)];
            }
          } else {
            binary_expand_op_1(Rwire_data, Rwire_size, k_ina219_, Rtot_data,
                               Rtot_size, prm);
          }

          if (!prm->seq.tst.i.useRwireFlag) {
            loop_ub = Rwire_size[1];
            for (i1 = 0; i1 < loop_ub; i1++) {
              b_loop_ub = Rwire_size[0];
              for (i2 = 0; i2 < b_loop_ub; i2++) {
                Rwire_data[i2 + Rwire_size[0] * i1] = 0.0F;
              }
            }
          }
        }

        guard3 = true;
      } else if (prm->seq.tst.i.measRintR) {
        loop_ub = ImKp184Test2->size[2];
        for (i = 0; i < loop_ub; i++) {
          tvv_data[i] = ImKp184Test2_data[ImKp184Test2->size[0] *
            ImKp184Test2->size[1] * i];
        }

        if (prm->seq.tst.i.RintBatId < 1) {
          k_groups_tmp = 1;
        } else {
          k_groups_tmp = prm->seq.tst.i.RintBatId;
        }

        loop_ub = VbusTest2->size[2];
        for (i = 0; i < loop_ub; i++) {
          b_VmV_data[i] = VbusTest2_data[VbusTest2->size[0] * (k_groups_tmp - 1)
            + VbusTest2->size[0] * VbusTest2->size[1] * i];
        }

        Nina219[0] = 1;
        Nina219[1] = prm->seq.tst.i.NTtest;
        id2Bypass_data[0] = 1;
        id2Bypass_data[1] = prm->seq.tst.i.NTtest;
        b_polyfit(tvv_data, Nina219, b_VmV_data, id2Bypass_data, outVI);
        outStruct->Rint = -outVI[0];
        VbusTest2_size[2] = VbusTest2->size[2];
        loop_ub = VbusTest2->size[3];
        for (i = 0; i < loop_ub; i++) {
          b_loop_ub = VbusTest2->size[2];
          for (i1 = 0; i1 < b_loop_ub; i1++) {
            b_VbusTest2_data[i1 + VbusTest2_size[2] * i] = VbusTest2_data
              [(VbusTest2->size[0] * (prm->seq.tst.i.RintBatId - 1) +
                VbusTest2->size[0] * VbusTest2->size[1] * i1) + VbusTest2->size
              [0] * VbusTest2->size[1] * VbusTest2->size[2] * i];
          }
        }

        trueCount = VbusTest2->size[2] * VbusTest2->size[3];
        outStruct->VpassFlag = true;
        for (i = 0; i < trueCount; i++) {
          d_tmp_data = b_VbusTest2_data[i];
          VmV_data[i] = d_tmp_data;
          VbusTest0_data[i] = (d_tmp_data > prm->bat.Vmax);
        }

        if (any(VbusTest0_data, trueCount)) {
          outStruct->VpassFlag = false;
        } else {
          for (i = 0; i < trueCount; i++) {
            VbusTest0_data[i] = (VmV_data[i] < prm->bat.Vmin);
          }

          if (any(VbusTest0_data, trueCount)) {
            outStruct->VpassFlag = false;
          } else {
            j_sprintf(prm->seq.tst.i.RintBatId, c_prm);
            Nina219[0] = 1;
            Nina219[1] = prm->seq.tst.i.NTtest;
            id2Bypass_data[0] = 1;
            id2Bypass_data[1] = prm->seq.tst.i.NTtest;
            b_polyfit(tvv_data, Nina219, b_VmV_data, id2Bypass_data, outVI);
            d_sprintf(-outVI[0], r11);
          }
        }
      } else {
        guard3 = true;
      }
    } else {
      b_loop_ub_tmp = (int)prm->brd.Nina219;
      kEst_size[0] = (int)prm->brd.Nina219;
      c_loop_ub_tmp = prm->brd.N_bat;
      kEst_size[1] = prm->brd.N_bat;
      kEst_size[2] = 2;
      loop_ub = ((int)prm->brd.Nina219 * prm->brd.N_bat) << 1;
      if (loop_ub - 1 >= 0) {
        memset(&kEst_data[0], 0, (unsigned int)loop_ub * sizeof(double));
      }

      for (i = 0; i < c_loop_ub_tmp; i++) {
        for (i1 = 0; i1 < b_loop_ub_tmp; i1++) {
          kEst_data[i1 + b_loop_ub_tmp * i] = 1.0;
        }
      }

      /* pIshunt*0; */
      loop_ub = prm->brd.N_bat;
      for (i = 0; i < loop_ub; i++) {
        for (i1 = 0; i1 < b_loop_ub_tmp; i1++) {
          kEst_data[(i1 + kEst_size[0] * i) + kEst_size[0] * c_loop_ub_tmp] =
            0.0;
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
        writePortToSpi4RowMask_ser(disConAll);
        break;
      }
    }

    loop_ub = prm->brd.N_bat;
    for (i = 0; i < loop_ub; i++) {
      b_loop_ub = kEst_size[0];
      for (i1 = 0; i1 < b_loop_ub; i1++) {
        kEst_data[i1 + kEst_size[0] * i] = 1.0;
      }
    }

    /* pIshunt*0; */
    loop_ub = prm->brd.N_bat;
    for (i = 0; i < loop_ub; i++) {
      b_loop_ub = kEst_size[0];
      for (i1 = 0; i1 < b_loop_ub; i1++) {
        kEst_data[(i1 + kEst_size[0] * i) + kEst_size[0] * kEst_size[1]] = 0.0;
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

    Nina219[0] = loop_ub;
    Nina219[1] = outStruct->VbusTest->size[1];
    b_loop_ub = outStruct->VbusTest->size[1];
    for (i = 0; i < b_loop_ub; i++) {
      for (i1 = 0; i1 < loop_ub; i1++) {
        outStruct->VbusTest->data[i1 + outStruct->VbusTest->size[0] * i] =
          VbusTest_data[i1 + Nina219[0] * i];
      }
    }

    Nina219[0] = loop_ub;
    Nina219[1] = outStruct->VmKp184Test->size[1];
    b_loop_ub = outStruct->VmKp184Test->size[1];
    for (i = 0; i < b_loop_ub; i++) {
      for (i1 = 0; i1 < loop_ub; i1++) {
        outStruct->VmKp184Test->data[i1 + outStruct->VmKp184Test->size[0] * i] =
          VmKp184Test_data[i1 + Nina219[0] * i];
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
    Pac2Vid0All_size[0] = (int)prm->brd.Nina219;
    d_loop_ub = prm->brd.N_bat;
    Pac2Vid0All_size[2] = prm->brd.N_bat;
    c_loop_ub_tmp = (int)prm->brd.Nina219 * prm->brd.N_bat;
    end = c_loop_ub_tmp * prm->brd.N_bat;
    if (end - 1 >= 0) {
      memset(&Pac2Vid0All_data[0], 0, (unsigned int)end * sizeof(unsigned char));
    }

    if (prm->seq.bit.bit_flag) {
      /* TODO groups */
      /*  meanVbusTest = mean(VbusTest,3);%(k_ina219,k_bat,k_Ttest) */
      N_bitCnfg = BuildBitCnfg(prm->brd.N_bat1, prm->brd.N_bat2, prm->brd.N_bat,
        prm->Nmax.NbatMax, bitCnfg_data, b_VdebugVec_size, bitCnfgStr_data,
        id2Bypass_data);

      /*      N_bitCnfg = length(bitCnfg); */
      /*  [N_bitI,Ngroups,NstBit] = size(seqBit.IdisChr); */
      /*  N_bitIdis = length(prm.bit.Idis); */
      /*  N_bitIchr = length(prm.bit.Ichr); */
      /* [128,16,32,32,32],[1 1 1 1 1]); */
      i = Vbrd->size[0] * Vbrd->size[1] * Vbrd->size[2] * Vbrd->size[3] *
        Vbrd->size[4];
      Vbrd->size[0] = N_bitCnfg;
      Vbrd->size[1] = (int)prm->brd.Nina219;
      Vbrd->size[2] = 1;
      Vbrd->size[3] = prm->brd.N_bat;
      i1 = prm->seq.bit.Nst;
      Vbrd->size[4] = prm->seq.bit.Nst;
      emxEnsureCapacity_real_T(Vbrd, i);
      VbusTest2_data = Vbrd->data;
      partialTrueCount = N_bitCnfg * (int)prm->brd.Nina219;
      id2Bypass_size = partialTrueCount * prm->brd.N_bat * prm->seq.bit.Nst;
      for (i = 0; i < id2Bypass_size; i++) {
        VbusTest2_data[i] = 0.0;
      }

      i = Ibrd->size[0] * Ibrd->size[1] * Ibrd->size[2] * Ibrd->size[3] *
        Ibrd->size[4];
      Ibrd->size[0] = N_bitCnfg;
      Ibrd->size[1] = (int)prm->brd.Nina219;
      Ibrd->size[2] = 1;
      Ibrd->size[3] = prm->brd.N_bat;
      Ibrd->size[4] = prm->seq.bit.Nst;
      emxEnsureCapacity_real_T(Ibrd, i);
      IshuntTest2_data = Ibrd->data;
      for (i = 0; i < id2Bypass_size; i++) {
        IshuntTest2_data[i] = 0.0;
      }

      /*  VbrdChr = zeros(N_bitCnfg,Nina219,N_bitIchr,N_bat); */
      /*  IbrdChr = zeros(N_bitCnfg,Nina219,N_bitIchr,N_bat); */
      /* [128,16,32,32],[1 1 1 1]); */
      i = meanIbrd->size[0] * meanIbrd->size[1] * meanIbrd->size[2] *
        meanIbrd->size[3];
      meanIbrd->size[0] = N_bitCnfg;
      meanIbrd->size[1] = (int)prm->brd.Nina219;
      meanIbrd->size[2] = 1;
      meanIbrd->size[3] = prm->seq.bit.Nst;
      emxEnsureCapacity_real_T(meanIbrd, i);
      ImKp184Test2_data = meanIbrd->data;
      id2Bypass_size = partialTrueCount * prm->seq.bit.Nst;
      for (i = 0; i < id2Bypass_size; i++) {
        ImKp184Test2_data[i] = 0.0;
      }

      /*  meanIbrdChr = zeros(N_bitCnfg,Nina219,N_bitIchr); */
      i = VbitInsMeas->size[0] * VbitInsMeas->size[1] * VbitInsMeas->size[2] *
        VbitInsMeas->size[3];
      VbitInsMeas->size[0] = N_bitCnfg;
      VbitInsMeas->size[1] = (int)prm->brd.Nina219;
      VbitInsMeas->size[2] = 1;
      VbitInsMeas->size[3] = prm->seq.bit.Nst;
      emxEnsureCapacity_real_T(VbitInsMeas, i);
      Iacs758_cal_data = VbitInsMeas->data;
      for (i = 0; i < id2Bypass_size; i++) {
        Iacs758_cal_data[i] = 0.0;
      }

      /*  VbitInsMeasChr = zeros(N_bitCnfg,Nina219,N_bitIdis); */
      /*  IbitInsMeasChr = zeros(N_bitCnfg,Nina219,N_bitIdis); */
      if (prm->brd.Nina219 < 1.0) {
        y->size[0] = 1;
        y->size[1] = 0;
      } else {
        i = y->size[0] * y->size[1];
        y->size[0] = 1;
        y->size[1] = (int)(prm->brd.Nina219 - 1.0) + 1;
        emxEnsureCapacity_real_T(y, i);
        y_data = y->data;
        loop_ub = (int)(prm->brd.Nina219 - 1.0);
        for (i = 0; i <= loop_ub; i++) {
          y_data[i] = (double)i + 1.0;
        }
      }

      i = b_y->size[0];
      b_y->size[0] = y->size[1];
      emxEnsureCapacity_real_T(b_y, i);
      b_y_data = b_y->data;
      loop_ub = y->size[1];
      for (i = 0; i < loop_ub; i++) {
        b_y_data[i] = y_data[i];
      }

      loop_ub = y->size[1];
      for (i = 0; i < loop_ub; i++) {
        VecIna219_data[i] = b_y_data[i];
      }

      size_tmp_idx_1 = (unsigned short)N_bitCnfg;
      if ((unsigned short)N_bitCnfg > 32767) {
        size_tmp_idx_1 = 32767U;
      }

      i = size_tmp_idx_1;
      for (k_t0 = 0; k_t0 < i; k_t0++) {
        for (state_k = 0; state_k < i1; state_k++) {
          for (k_tstState = 0; k_tstState < b_size; k_tstState++) {
            id2Bypass_size = ipos_size - 1;
            trueCount = 0;
            partialTrueCount = 0;
            for (loop_ub = 0; loop_ub <= id2Bypass_size; loop_ub++) {
              if (k_tstState + 1 == ipos_data[loop_ub]) {
                trueCount++;
                f_tmp_data[partialTrueCount] = (signed char)loop_ub;
                partialTrueCount++;
              }
            }

            /* VecIna219_k = find(k_groups == uGroupId); */
            for (k0 = 0; k0 < trueCount; k0++) {
              /* for k_ina219 = VecIna219_k */
              if (prm->seq.bit.BrdBeforePSflag[state_k] != 0) {
                for (i2 = 0; i2 < trueCount; i2++) {
                  b_VecIna219_data[i2] = (VecIna219_data[f_tmp_data[i2]] !=
                    VecIna219_data[f_tmp_data[k0]]);
                }

                VdebugVec_size_tmp = c_eml_find(b_VecIna219_data, trueCount,
                  id2Bypass_data);
                c_loop_ub = c_eml_find(b_VecIna219_data, trueCount,
                  id2Bypass_data);
                if (c_loop_ub - 1 >= 0) {
                  CalcPortSpiPrm(prm->brd.spi.disconnect, prm->brd.spi.bypass,
                                 prm->brd.spi.PortSpiRow_esp,
                                 prm->brd.spi.SwitchMat_esp,
                                 prm->brd.spi.PortSpiRow_esp2,
                                 prm->brd.spi.SwitchMat_esp2,
                                 prm->brd.spi.Pac2Vid, prm->brd.spi.Pac2Vid2,
                                 prm->brd.spi.bypass, SpiPortRowBypass,
                                 a__1_data, a__1_size, id0_data);
                }

                for (k1 = 0; k1 < VdebugVec_size_tmp; k1++) {
                  /* for k_bypass = VecIna219(id2Bypass) */
                  switch (ProjectFlag) {
                   case 2:
                    /* ESP32 */
                    /* TODO bypass */
                    break;

                   case 3:
                    /* ESP32 ser */
                    writePortToSpi4RowMask_ser(SpiPortRowBypass);

                    /* TODO bypass */
                    break;
                  }

                  pause(0.1);
                }

                bitCnfg_size[1] = b_VdebugVec_size[1];
                loop_ub = b_VdebugVec_size[2];
                for (i2 = 0; i2 < loop_ub; i2++) {
                  b_loop_ub = b_VdebugVec_size[1];
                  for (i3 = 0; i3 < b_loop_ub; i3++) {
                    b_bitCnfg_data[i3 + bitCnfg_size[1] * i2] = bitCnfg_data
                      [(k_t0 + b_VdebugVec_size[0] * i3) + b_VdebugVec_size[0] *
                      b_VdebugVec_size[1] * i2];
                  }
                }

                Nina219[0] = prm->Nmax.NbatMax;
                Nina219[1] = prm->Nmax.NbatMax;
                d_bitCnfg_data.data = &b_bitCnfg_data[0];
                d_bitCnfg_data.size = &Nina219[0];
                d_bitCnfg_data.allocatedSize = 256;
                d_bitCnfg_data.numDimensions = 2;
                d_bitCnfg_data.canFreeData = false;
                b_CalcPortSpiPrm(prm->brd.spi.disconnect, prm->brd.spi.bypass,
                                 prm->brd.spi.PortSpiRow_esp,
                                 prm->brd.spi.SwitchMat_esp,
                                 prm->brd.spi.PortSpiRow_esp2,
                                 prm->brd.spi.SwitchMat_esp2,
                                 prm->brd.spi.Pac2Vid, prm->brd.spi.Pac2Vid2,
                                 &d_bitCnfg_data, SpiPortRowBypass, a__1_data,
                                 a__1_size, id0_data);
                for (i2 = 0; i2 < d_loop_ub; i2++) {
                  for (i3 = 0; i3 < d_loop_ub; i3++) {
                    Pac2Vid0All_data[(((int)VecIna219_data[f_tmp_data[k0]] +
                                       Pac2Vid0All_size[0] * i3) +
                                      Pac2Vid0All_size[0] * d_loop_ub * i2) - 1]
                      = id0_data[i3 + (i2 << 4)];
                  }
                }

                /* sort(bitCnfg{k_bitCnfg})); */
                switch (ProjectFlag) {
                 case 2:
                  /* esp32 */
                  break;

                 case 3:
                  /* esp32 ser */
                  writePortToSpi4RowMask_ser(SpiPortRowBypass);
                  break;
                }
              } else {
                bitCnfg_size[1] = b_VdebugVec_size[1];
                loop_ub = b_VdebugVec_size[2];
                for (i2 = 0; i2 < loop_ub; i2++) {
                  b_loop_ub = b_VdebugVec_size[1];
                  for (i3 = 0; i3 < b_loop_ub; i3++) {
                    b_bitCnfg_data[i3 + bitCnfg_size[1] * i2] = bitCnfg_data
                      [(k_t0 + b_VdebugVec_size[0] * i3) + b_VdebugVec_size[0] *
                      b_VdebugVec_size[1] * i2];
                  }
                }

                Nina219[0] = prm->Nmax.NbatMax;
                Nina219[1] = prm->Nmax.NbatMax;
                c_bitCnfg_data.data = &b_bitCnfg_data[0];
                c_bitCnfg_data.size = &Nina219[0];
                c_bitCnfg_data.allocatedSize = 256;
                c_bitCnfg_data.numDimensions = 2;
                c_bitCnfg_data.canFreeData = false;
                b_CalcPortSpiPrm(prm->brd.spi.disconnect, prm->brd.spi.bypass,
                                 prm->brd.spi.PortSpiRow_esp,
                                 prm->brd.spi.SwitchMat_esp,
                                 prm->brd.spi.PortSpiRow_esp2,
                                 prm->brd.spi.SwitchMat_esp2,
                                 prm->brd.spi.Pac2Vid, prm->brd.spi.Pac2Vid2,
                                 &c_bitCnfg_data, SpiPortRowBypass, a__1_data,
                                 a__1_size, id0_data);
                for (i2 = 0; i2 < d_loop_ub; i2++) {
                  for (i3 = 0; i3 < d_loop_ub; i3++) {
                    Pac2Vid0All_data[(((int)VecIna219_data[f_tmp_data[k0]] +
                                       Pac2Vid0All_size[0] * i3) +
                                      Pac2Vid0All_size[0] * d_loop_ub * i2) - 1]
                      = id0_data[i3 + (i2 << 4)];
                  }
                }

                /* sort(bitCnfg{k_bitCnfg}));sort(bitCnfg{k_bitCnfg})); */
              }

              /*  calc V */
              bitCnfg_size[0] = 1;
              bitCnfg_size[1] = d_loop_ub;
              bitCnfg_size[2] = Pac2Vid0All_size[2];
              for (i2 = 0; i2 < d_loop_ub; i2++) {
                for (i3 = 0; i3 < d_loop_ub; i3++) {
                  b_bitCnfg_data[i3 + d_loop_ub * i2] = Pac2Vid0All_data[(((int)
                    VecIna219_data[f_tmp_data[k0]] + Pac2Vid0All_size[0] * i3) +
                    Pac2Vid0All_size[0] * d_loop_ub * i2) - 1];
                }
              }

              c_squeeze(b_bitCnfg_data, bitCnfg_size, id0_data, id2Bypass_data);

              /* Pac2Vid3(:,1); */
              id2Bypass_size = id2Bypass_data[0] * id2Bypass_data[1] - 1;
              b_trueCount = 0;
              for (loop_ub = 0; loop_ub <= id2Bypass_size; loop_ub++) {
                if (id0_data[loop_ub] > 0) {
                  b_trueCount++;
                }
              }

              partialTrueCount = (unsigned short)b_trueCount;
              if (partialTrueCount < 1) {
                partialTrueCount = 1;
              }

              if ((unsigned short)b_trueCount == 0) {
                partialTrueCount = 0;
              }

              c_vSumMax = prm->bat.Vd + (prm->bat.CutOffChrV[0] + 0.1F) * (float)
                partialTrueCount;

              /*  sumV      = sum(meanVbusTest(k_ina219,Pac2Vid31)); */
              switch (prm->seq.bit.ins[k_tstState + (state_k << 3)]) {
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
                f_sprintf(prm->seq.bit.IdisChr[k_tstState + (state_k << 3)],
                          d_prm);

                /* ['ISET',num2str(ch),':',num2str(Val)];%[A] */
                pause(0.05);
                break;

               case 3:
                /* juntek+ACDC */
                /* Imax); */
                i_prm[0] = prm->seq.bit.IdisChr[k_tstState + (state_k << 3)];
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

                g_sprintf(i2, r12);

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

                g_sprintf(i2, r13);

                /* [':',juntek_address,'w10=',num2str(uint32(Val*100),'%04d'),',',newline];%['VSET',num2str(ch),':',num2str(Val)];%[V] */
                /*  disp(['Juntek VSET=',num2str(vSumMax)]); */
                pause(0.1);

                /*  disp('juntek ON'); */
                break;
              }

              pause(0.2);

              /* switch board */
              if (prm->seq.bit.BrdBeforePSflag[state_k] == 0) {
                for (i2 = 0; i2 < trueCount; i2++) {
                  b_VecIna219_data[i2] = (VecIna219_data[f_tmp_data[i2]] !=
                    VecIna219_data[f_tmp_data[k0]]);
                }

                VdebugVec_size_tmp = c_eml_find(b_VecIna219_data, trueCount,
                  id2Bypass_data);
                c_loop_ub = c_eml_find(b_VecIna219_data, trueCount,
                  id2Bypass_data);
                if (c_loop_ub - 1 >= 0) {
                  CalcPortSpiPrm(prm->brd.spi.disconnect, prm->brd.spi.bypass,
                                 prm->brd.spi.PortSpiRow_esp,
                                 prm->brd.spi.SwitchMat_esp,
                                 prm->brd.spi.PortSpiRow_esp2,
                                 prm->brd.spi.SwitchMat_esp2,
                                 prm->brd.spi.Pac2Vid, prm->brd.spi.Pac2Vid2,
                                 prm->brd.spi.bypass, SpiPortRowBypass,
                                 a__1_data, a__1_size, id0_data);
                }

                for (k1 = 0; k1 < VdebugVec_size_tmp; k1++) {
                  /* for k_bypass = VecIna219(id2Bypass) */
                  /*                                for k_bypass = VecIna219(id2Bypass) */
                  switch (ProjectFlag) {
                   case 2:
                    /* ESP32 */
                    /* TODO bypass */
                    break;

                   case 3:
                    /* ESP32 ser */
                    writePortToSpi4RowMask_ser(SpiPortRowBypass);

                    /* TODO bypass */
                    break;
                  }

                  pause(0.1);
                }

                bitCnfg_size[1] = b_VdebugVec_size[1];
                loop_ub = b_VdebugVec_size[2];
                for (i2 = 0; i2 < loop_ub; i2++) {
                  b_loop_ub = b_VdebugVec_size[1];
                  for (i3 = 0; i3 < b_loop_ub; i3++) {
                    b_bitCnfg_data[i3 + bitCnfg_size[1] * i2] = bitCnfg_data
                      [(k_t0 + b_VdebugVec_size[0] * i3) + b_VdebugVec_size[0] *
                      b_VdebugVec_size[1] * i2];
                  }
                }

                Nina219[0] = prm->Nmax.NbatMax;
                Nina219[1] = prm->Nmax.NbatMax;
                e_bitCnfg_data.data = &b_bitCnfg_data[0];
                e_bitCnfg_data.size = &Nina219[0];
                e_bitCnfg_data.allocatedSize = 256;
                e_bitCnfg_data.numDimensions = 2;
                e_bitCnfg_data.canFreeData = false;
                b_CalcPortSpiPrm(prm->brd.spi.disconnect, prm->brd.spi.bypass,
                                 prm->brd.spi.PortSpiRow_esp,
                                 prm->brd.spi.SwitchMat_esp,
                                 prm->brd.spi.PortSpiRow_esp2,
                                 prm->brd.spi.SwitchMat_esp2,
                                 prm->brd.spi.Pac2Vid, prm->brd.spi.Pac2Vid2,
                                 &e_bitCnfg_data, SpiPortRowBypass, a__1_data,
                                 a__1_size, id0_data);
                for (i2 = 0; i2 < d_loop_ub; i2++) {
                  for (i3 = 0; i3 < d_loop_ub; i3++) {
                    Pac2Vid0All_data[(((int)VecIna219_data[f_tmp_data[k0]] +
                                       Pac2Vid0All_size[0] * i3) +
                                      Pac2Vid0All_size[0] * d_loop_ub * i2) - 1]
                      = id0_data[i3 + (i2 << 4)];
                  }
                }

                /* sort(bitCnfg{k_bitCnfg})); */
                switch (ProjectFlag) {
                 case 2:
                  /* esp32 */
                  break;

                 case 3:
                  /* esp32 ser */
                  writePortToSpi4RowMask_ser(SpiPortRowBypass);
                  break;
                }
              }

              /* switch board - End */
              pause(0.2);
              switch (ProjectFlag) {
               case 2:
                /* esp32 */
                /*  [VbrdChr(k_bitCnfg,k_ina219,k_bitIchr,:),IbrdChr(k_bitCnfg,k_ina219,k_bitIchr,:),~,~,errI2C,~] = readI2cVIfastTicTocV5_Rratio(Esp32_v1(k_ina219),true,true,true,VIpacId(k_ina219,:),Pac2Vid0,squeeze(polyFitPacV(k_ina219,:,:)),squeeze(Rval(k_ina219,:,:)),N_bat1,N_bat2); */
                k_groups_tmp = f_tmp_data[k0];
                for (i2 = 0; i2 < 16; i2++) {
                  g_prm[i2] = prm->brd.pac.VIpacId[((int)
                    VecIna219_data[k_groups_tmp] + (i2 << 1)) - 1];
                }

                bitCnfg_size[0] = 1;
                bitCnfg_size[1] = d_loop_ub;
                bitCnfg_size[2] = Pac2Vid0All_size[2];
                for (i2 = 0; i2 < d_loop_ub; i2++) {
                  for (i3 = 0; i3 < d_loop_ub; i3++) {
                    b_bitCnfg_data[i3 + d_loop_ub * i2] = Pac2Vid0All_data
                      [(((int)VecIna219_data[k_groups_tmp] + Pac2Vid0All_size[0]
                         * i3) + Pac2Vid0All_size[0] * d_loop_ub * i2) - 1];
                  }
                }

                c_loop_ub = (int)VecIna219_data[k_groups_tmp] - 1;
                i_prm[0] = pIacs758_data[c_loop_ub];
                i_prm[1] = pIacs758_data[c_loop_ub + pIacs758_size[0]];
                b_Rwire_size[0] = 1;
                loop_ub = Rwire_size[1];
                b_Rwire_size[1] = Rwire_size[1];
                for (i2 = 0; i2 < loop_ub; i2++) {
                  e_prm[i2] = Rwire_data[c_loop_ub + Rwire_size[0] * i2];
                }

                ReadVI(ProjectFlag, g_prm, b_bitCnfg_data, bitCnfg_size, N_bat1,
                       N_bat_tmp_tmp_tmp, i_prm, Iacs758Flag, kEst_data,
                       kEst_size, e_prm, b_Rwire_size, a__36_data,
                       c_VmKp184Test_data, &partialTrueCount, tvv_data,
                       &VdebugVec_size_tmp, Rtot_data, Rtot_size, a__38_data,
                       id2Bypass_data, a__39_data, b_VmV_size, a__40_data,
                       tii_size, b_VmV_data, &id2Bypass_size, a__42_data,
                       a__4_size, I_data, &k_ina219_, outVI, b_tmp_size,
                       VdebugVec_data, VdebugVec_size, &showVdiff, (double *)
                       &b_dv0, errI2C_size);
                loop_ub = Vbrd->size[3];
                for (i2 = 0; i2 < loop_ub; i2++) {
                  VbusTest2_data[((k_t0 + Vbrd->size[0] * c_loop_ub) +
                                  Vbrd->size[0] * Vbrd->size[1] * Vbrd->size[2] *
                                  i2) + Vbrd->size[0] * Vbrd->size[1] *
                    Vbrd->size[2] * Vbrd->size[3] * state_k] =
                    c_VmKp184Test_data[i2];
                }

                loop_ub = Ibrd->size[3];
                for (i2 = 0; i2 < loop_ub; i2++) {
                  IshuntTest2_data[((k_t0 + Ibrd->size[0] * c_loop_ub) +
                                    Ibrd->size[0] * Ibrd->size[1] * Ibrd->size[2]
                                    * i2) + Ibrd->size[0] * Ibrd->size[1] *
                    Ibrd->size[2] * Ibrd->size[3] * state_k] = tvv_data[i2];
                }

                /*  read V,I */
                break;

               case 3:
                /* esp32 ser */
                /*  [VbrdChr(k_bitCnfg,k_ina219,k_bitIchr,:),IbrdChr(k_bitCnfg,k_ina219,k_bitIchr,:),~,~,errI2C,~] = readI2cVIfastTicTocV5_Rratio_ser(Esp32_v1(k_ina219),true,true,prm.brd.pac.Rshunt(k_ina219),VIpacId(k_ina219,:),Pac2Vid0,squeeze(polyFitPacV(k_ina219,:,:)),squeeze(Rval(k_ina219,:,:)),N_bat1,N_bat2); */
                k_groups_tmp = f_tmp_data[k0];
                for (i2 = 0; i2 < 16; i2++) {
                  g_prm[i2] = prm->brd.pac.VIpacId[((int)
                    VecIna219_data[k_groups_tmp] + (i2 << 1)) - 1];
                }

                bitCnfg_size[0] = 1;
                bitCnfg_size[1] = d_loop_ub;
                bitCnfg_size[2] = Pac2Vid0All_size[2];
                for (i2 = 0; i2 < d_loop_ub; i2++) {
                  for (i3 = 0; i3 < d_loop_ub; i3++) {
                    b_bitCnfg_data[i3 + d_loop_ub * i2] = Pac2Vid0All_data
                      [(((int)VecIna219_data[k_groups_tmp] + Pac2Vid0All_size[0]
                         * i3) + Pac2Vid0All_size[0] * d_loop_ub * i2) - 1];
                  }
                }

                c_loop_ub = (int)VecIna219_data[k_groups_tmp] - 1;
                i_prm[0] = pIacs758_data[c_loop_ub];
                i_prm[1] = pIacs758_data[c_loop_ub + pIacs758_size[0]];
                b_Rwire_size[0] = 1;
                loop_ub = Rwire_size[1];
                b_Rwire_size[1] = Rwire_size[1];
                for (i2 = 0; i2 < loop_ub; i2++) {
                  e_prm[i2] = Rwire_data[c_loop_ub + Rwire_size[0] * i2];
                }

                ReadVI(ProjectFlag, g_prm, b_bitCnfg_data, bitCnfg_size, N_bat1,
                       N_bat_tmp_tmp_tmp, i_prm, Iacs758Flag, kEst_data,
                       kEst_size, e_prm, b_Rwire_size, a__36_data,
                       c_VmKp184Test_data, &partialTrueCount, tvv_data,
                       &VdebugVec_size_tmp, Rtot_data, Rtot_size, a__38_data,
                       id2Bypass_data, a__39_data, b_VmV_size, a__40_data,
                       tii_size, b_VmV_data, &id2Bypass_size, a__42_data,
                       a__4_size, I_data, &k_ina219_, outVI, b_tmp_size,
                       VdebugVec_data, VdebugVec_size, &showVdiff, (double *)
                       &b_dv0, errI2C_size);
                loop_ub = Vbrd->size[3];
                for (i2 = 0; i2 < loop_ub; i2++) {
                  VbusTest2_data[((k_t0 + Vbrd->size[0] * c_loop_ub) +
                                  Vbrd->size[0] * Vbrd->size[1] * Vbrd->size[2] *
                                  i2) + Vbrd->size[0] * Vbrd->size[1] *
                    Vbrd->size[2] * Vbrd->size[3] * state_k] =
                    c_VmKp184Test_data[i2];
                }

                loop_ub = Ibrd->size[3];
                for (i2 = 0; i2 < loop_ub; i2++) {
                  IshuntTest2_data[((k_t0 + Ibrd->size[0] * c_loop_ub) +
                                    Ibrd->size[0] * Ibrd->size[1] * Ibrd->size[2]
                                    * i2) + Ibrd->size[0] * Ibrd->size[1] *
                    Ibrd->size[2] * Ibrd->size[3] * state_k] = tvv_data[i2];
                }

                /*  read V,I */
                break;
              }

              switch (prm->seq.bit.meas.V[k_tstState + (state_k << 3)]) {
               case 1:
                /* kunkin kp184 */
                pause(0.1);
                ControlKp184((double *)&N0, a__3_size, (double *)&a__4_data,
                             a__4_size, (double *)&b_dv0, errI2C_size, (double *)
                             &a__16_data, id2Bypass_data);
                Iacs758_cal_data[(k_t0 + VbitInsMeas->size[0] * ((int)
                  VecIna219_data[f_tmp_data[k0]] - 1)) + VbitInsMeas->size[0] *
                  VbitInsMeas->size[1] * VbitInsMeas->size[2] * state_k] =
                  (float)b_dv0 - prm->seq.bit.meas.Vd[k_tstState + (state_k << 3)];
                break;

               case 2:
                /* ka6005p */
                /* read I */
                d_tmp_data = d_rand();
                Iacs758_cal_data[(k_t0 + VbitInsMeas->size[0] * ((int)
                  VecIna219_data[f_tmp_data[k0]] - 1)) + VbitInsMeas->size[0] *
                  VbitInsMeas->size[1] * VbitInsMeas->size[2] * state_k] =
                  (float)(d_tmp_data * 2.5 + 2.0) - prm->
                  seq.bit.meas.Vd[k_tstState + (state_k << 3)];
                break;

               case 3:
                /* juntek+ACDC */
                pause(0.1);
                b_controlJuntekDPH8920((double *)&d_tmp_data, b_tmp_size);
                d_tmp_data = (float)d_tmp_data - prm->seq.bit.meas.Vd[k_tstState
                  + (state_k << 3)];
                Iacs758_cal_data[(k_t0 + VbitInsMeas->size[0] * ((int)
                  VecIna219_data[f_tmp_data[k0]] - 1)) + VbitInsMeas->size[0] *
                  VbitInsMeas->size[1] * VbitInsMeas->size[2] * state_k] =
                  d_tmp_data;
                break;
              }

              switch (prm->seq.bit.meas.b_I[k_tstState + (state_k << 3)]) {
               case 1:
                /* kunkin kp184 */
                pause(0.1);
                ControlKp184((double *)&N0, a__3_size, (double *)&a__4_data,
                             a__4_size, (double *)&b_dv0, errI2C_size, (double *)
                             &d_tmp_data, b_tmp_size);
                break;

               case 2:
                /* ka6005p */
                /* read I */
                d_rand();
                break;

               case 3:
                /* juntek+ACDC */
                controlJuntekDPH8920(r30.data, r30.size);
                break;
              }

              switch (ProjectFlag) {
               case 2:
                /* esp32 */
                break;

               case 3:
                /* esp32 ser */
                writePortToSpi4RowMask_ser(disConAll);
                break;
              }

              bitCnfg_size[0] = 1;
              bitCnfg_size[1] = d_loop_ub;
              bitCnfg_size[2] = Pac2Vid0All_size[2];
              for (i2 = 0; i2 < d_loop_ub; i2++) {
                for (i3 = 0; i3 < d_loop_ub; i3++) {
                  b_bitCnfg_data[i3 + d_loop_ub * i2] = Pac2Vid0All_data[(((int)
                    VecIna219_data[f_tmp_data[k0]] + Pac2Vid0All_size[0] * i3) +
                    Pac2Vid0All_size[0] * d_loop_ub * i2) - 1];
                }
              }

              c_squeeze(b_bitCnfg_data, bitCnfg_size, id0_data, id2Bypass_data);
              id2Bypass_size = id2Bypass_data[0] * id2Bypass_data[1] - 1;
              b_trueCount = 0;
              partialTrueCount = 0;
              for (loop_ub = 0; loop_ub <= id2Bypass_size; loop_ub++) {
                if (id0_data[loop_ub] > 0) {
                  b_trueCount++;
                  i_tmp_data[partialTrueCount] = (unsigned char)loop_ub;
                  partialTrueCount++;
                }
              }

              Ibrd_size[0] = 1;
              Ibrd_size[1] = 1;
              Ibrd_size[2] = 1;
              Ibrd_size[3] = b_trueCount;
              for (i2 = 0; i2 < b_trueCount; i2++) {
                VmV_data[i2] = IshuntTest2_data[((k_t0 + Ibrd->size[0] * ((int)
                  VecIna219_data[f_tmp_data[k0]] - 1)) + Ibrd->size[0] *
                  Ibrd->size[1] * Ibrd->size[2] * (id0_data[i_tmp_data[i2]] - 1))
                  + Ibrd->size[0] * Ibrd->size[1] * Ibrd->size[2] * Ibrd->size[3]
                  * state_k];
              }

              b_dv0 = VecIna219_data[f_tmp_data[k0]];
              ImKp184Test2_data[(k_t0 + meanIbrd->size[0] * ((int)b_dv0 - 1)) +
                meanIbrd->size[0] * meanIbrd->size[1] * meanIbrd->size[2] *
                state_k] = b_mean(VmV_data, Ibrd_size);
              b_meanIbrd0 = ImKp184Test2_data[(k_t0 + meanIbrd->size[0] * ((int)
                b_dv0 - 1)) + meanIbrd->size[0] * meanIbrd->size[1] *
                meanIbrd->size[2] * state_k];
              i2 = k_tstState + (state_k << 3);
              if (prm->seq.bit.chr[i2] != 0) {
                b_meanIbrd0 = -b_meanIbrd0;
              }

              showVdiff = true;
              f = prm->seq.bit.IdisChr[i2];
              if ((float)fabs((float)b_meanIbrd0 - f) > prm->seq.bit.dIthr) {
                d_sprintf(b_meanIbrd0, meanIbrd0);
                k_sprintf(f, r14);
                k_sprintf(f, r15);
                if (b_dv0 < 2.147483648E+9) {
                  i2 = (int)b_dv0;
                } else {
                  i2 = MAX_int32_T;
                }

                b_sprintf(i2, r16);
                for (id2Bypass_size = 0; id2Bypass_size < b_trueCount;
                     id2Bypass_size++) {
                  printf("%d\n", (int)id0_data[i_tmp_data[id2Bypass_size]]);
                  fflush(stdout);
                }

                showVdiff = false;
              }

              Ibrd_size[0] = 1;
              Ibrd_size[1] = 1;
              Ibrd_size[2] = 1;
              Ibrd_size[3] = b_trueCount;
              for (i2 = 0; i2 < b_trueCount; i2++) {
                VmV_data[i2] = VbusTest2_data[((k_t0 + Vbrd->size[0] * ((int)
                  b_dv0 - 1)) + Vbrd->size[0] * Vbrd->size[1] * Vbrd->size[2] *
                  (id0_data[i_tmp_data[i2]] - 1)) + Vbrd->size[0] * Vbrd->size[1]
                  * Vbrd->size[2] * Vbrd->size[3] * state_k];
              }

              b_dv0 = sum(VmV_data, Ibrd_size) - Iacs758_cal_data[(k_t0 +
                VbitInsMeas->size[0] * ((int)b_dv0 - 1)) + VbitInsMeas->size[0] *
                VbitInsMeas->size[1] * VbitInsMeas->size[2] * state_k];
              if ((fabs(b_dv0) > prm->seq.bit.dVthr) && showVdiff) {
                d_sprintf(b_dv0, dv0);
                d_tmp_data = VecIna219_data[f_tmp_data[k0]];
                if (d_tmp_data < 2.147483648E+9) {
                  i2 = (int)d_tmp_data;
                } else {
                  i2 = MAX_int32_T;
                }

                b_sprintf(i2, r17);
                for (id2Bypass_size = 0; id2Bypass_size < b_trueCount;
                     id2Bypass_size++) {
                  printf("%d\n", (int)id0_data[i_tmp_data[id2Bypass_size]]);
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
    /* ,[8 1024 16],[1 1 1]); */
    /*  SelVsT = zeros(Nt0,1); */
    /*  SelDualVsT = zeros(Nina219,Nt0); */
    /* [16 32 1024],[1 1 1]); */
    /* [512 1024],[1 1]); */
    /* [16 32 1024],[1 1 1]); */
    i = (int)(prm->brd.Nina219 * (double)prm->brd.N_bat);
    i1 = b_Vbat->size[0] * b_Vbat->size[1];
    b_Vbat->size[0] = i;
    b_Vbat->size[1] = prm->run.Nt0;
    emxEnsureCapacity_int8_T(b_Vbat, i1);
    Vbat_data = b_Vbat->data;
    id2Bypass_size = i * prm->run.Nt0;
    for (i1 = 0; i1 < id2Bypass_size; i1++) {
      Vbat_data[i1] = 0;
    }

    i1 = Vbat->size[0] * Vbat->size[1];
    Vbat->size[0] = b_Vbat->size[0];
    Vbat->size[1] = b_Vbat->size[1];
    emxEnsureCapacity_real_T(Vbat, i1);
    ImKp184Test2_data = Vbat->data;
    for (i1 = 0; i1 < id2Bypass_size; i1++) {
      ImKp184Test2_data[i1] = 0.0;
    }

    i1 = IbatMat->size[0] * IbatMat->size[1] * IbatMat->size[2];
    IbatMat->size[0] = (int)prm->brd.Nina219;
    IbatMat->size[1] = prm->brd.N_bat;
    IbatMat->size[2] = prm->run.Nt0;
    emxEnsureCapacity_real_T(IbatMat, i1);
    Iacs758_cal_data = IbatMat->data;
    c_loop_ub_tmp *= prm->run.Nt0;
    for (i1 = 0; i1 < c_loop_ub_tmp; i1++) {
      Iacs758_cal_data[i1] = 0.0;
    }

    i1 = VbatMat->size[0] * VbatMat->size[1] * VbatMat->size[2];
    VbatMat->size[0] = (int)prm->brd.Nina219;
    VbatMat->size[1] = prm->brd.N_bat;
    VbatMat->size[2] = prm->run.Nt0;
    emxEnsureCapacity_real_T(VbatMat, i1);
    VbatMat_data = VbatMat->data;
    for (i1 = 0; i1 < c_loop_ub_tmp; i1++) {
      VbatMat_data[i1] = 0.0;
    }

    i1 = VbatMat0->size[0] * VbatMat0->size[1] * VbatMat0->size[2];
    VbatMat0->size[0] = (int)prm->brd.Nina219;
    VbatMat0->size[1] = prm->brd.N_bat;
    VbatMat0->size[2] = prm->run.Nt0;
    emxEnsureCapacity_real_T(VbatMat0, i1);
    VbatMat0_data = VbatMat0->data;
    for (i1 = 0; i1 < c_loop_ub_tmp; i1++) {
      VbatMat0_data[i1] = 0.0;
    }

    i1 = b_Vbat->size[0] * b_Vbat->size[1];
    b_Vbat->size[0] = i;
    b_Vbat->size[1] = prm->run.Nt0;
    emxEnsureCapacity_int8_T(b_Vbat, i1);
    Vbat_data = b_Vbat->data;
    for (i = 0; i < id2Bypass_size; i++) {
      Vbat_data[i] = 0;
    }

    i = tV1->size[0] * tV1->size[1];
    tV1->size[0] = b_Vbat->size[0];
    tV1->size[1] = b_Vbat->size[1];
    emxEnsureCapacity_real_T(tV1, i);
    tV1_data = tV1->data;
    for (i = 0; i < id2Bypass_size; i++) {
      tV1_data[i] = 0.0;
    }

    i = tV->size[0] * tV->size[1] * tV->size[2];
    tV->size[0] = (int)prm->brd.Nina219;
    tV->size[1] = prm->brd.N_bat;
    tV->size[2] = prm->run.Nt0;
    emxEnsureCapacity_real_T(tV, i);
    tV_data = tV->data;
    for (i = 0; i < c_loop_ub_tmp; i++) {
      tV_data[i] = 0.0;
    }

    /*  Ishunt = zeros(Nina219,N_bat,Nt); */
    /* [1 1024],[1 1]); */
    /* ,[16 1024],[1 1]); */
    /*  coder.varsize('Pac2Vid0All',[16 32 32],[1 1 1]); */
    Pac2Vid0All_size[0] = (int)prm->brd.Nina219;
    Pac2Vid0All_size[1] = prm->brd.N_bat;
    Pac2Vid0All_size[2] = prm->brd.N_bat;
    if (end - 1 >= 0) {
      memset(&Pac2Vid0All_data[0], 0, (unsigned int)end * sizeof(unsigned char));
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
      for (loop_ub = 0; loop_ub <= end_tmp; loop_ub++) {
        if (b_data[id2Bypass_size] == prm->ser.com.grp[tmp_data[loop_ub]]) {
          SelDual_data[loop_ub] = prm->seq.chr[id2Bypass_size];
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
      /*      coder.varsize('outStruct.VbusTest','outStruct.VmKp184Test',[Nmax.NbrdMax,Nmax.NbatMax],[1 1]); */
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
      if (end - 1 >= 0) {
        memset(&BattConfigPerIna_data[0], 0, (unsigned int)end * sizeof(double));
      }

      /* nadav coder */
      if ((prm->seq.pwr.VthFlag[0] == 1) || (prm->seq.pwr.VthFlag[0] == 2)) {
        i = r2->size[0] * r2->size[1] * r2->size[2];
        r2->size[0] = (int)prm->brd.Nina219;
        r2->size[1] = prm->brd.N_bat;
        r2->size[2] = prm->brd.N_bat;
        emxEnsureCapacity_uint8_T(r2, i);
        r31 = r2->data;
        for (i = 0; i < end; i++) {
          r31[i] = 0U;
        }

        b_ReadVI(prm->brd.Nina219, prm->ins.ProjectFlag, prm->brd.pac.VIpacId,
                 (unsigned char *)r2->data, r2->size, prm->brd.N_bat1,
                 prm->brd.N_bat, pIacs758_data, pIacs758_size,
                 prm->brd.pac.Iacs758Flag, kEst_data, kEst_size, Rwire_data,
                 Rwire_size, a__64_data, b_VdebugVec_data, &id2Bypass_size,
                 VbusTest_data, &k_ina219_, Rtot_data, Rtot_size, a__38_data,
                 id2Bypass_data, a__39_data, b_VmV_size, a__40_data, tii_size,
                 VmKp184Test_data, &c_loop_ub, a__42_data, a__4_size, a__72_data,
                 &partialTrueCount, outVI, b_tmp_size, VdebugVec_data,
                 VdebugVec_size, &showVdiff, (double *)&b_dv0, errI2C_size);

        /*  read V,I */
        if ((int)prm->brd.Nina219 - 1 >= 0) {
          if (prm->brd.N_bat < 1) {
            y->size[0] = 1;
            y->size[1] = 0;
          } else {
            i = y->size[0] * y->size[1];
            y->size[0] = 1;
            y->size[1] = prm->brd.N_bat;
            emxEnsureCapacity_real_T(y, i);
            y_data = y->data;
            loop_ub = prm->brd.N_bat - 1;
            for (i = 0; i <= loop_ub; i++) {
              y_data[i] = (double)i + 1.0;
            }
          }

          e_loop_ub = y->size[1];
        }

        for (c_loop_ub = 0; c_loop_ub < b_loop_ub_tmp; c_loop_ub++) {
          b_dv0 = (((double)c_loop_ub + 1.0) - 1.0) * (double)N_bat_tmp_tmp_tmp;
          VdebugVec_size_tmp = y->size[1];
          for (i = 0; i < e_loop_ub; i++) {
            j_tmp_data[i] = (signed char)((signed char)(b_dv0 + y_data[i]) - 1);
          }

          for (i = 0; i < VdebugVec_size_tmp; i++) {
            ImKp184Test2_data[j_tmp_data[i]] = a__38_data[c_loop_ub +
              id2Bypass_data[0] * i];
          }

          i = (int)SelDual_data[c_loop_ub];
          if (i == 0) {
            e_padArrUint8(prm->cnfg.BattConfigDis1, N_bat_tmp_tmp_tmp,
                          N_bat_tmp_tmp_tmp, r3);
            r31 = r3->data;
            i = r->size[0] * r->size[1];
            r->size[0] = r3->size[0];
            r->size[1] = r3->size[1];
            emxEnsureCapacity_real_T(r, i);
            b_y_data = r->data;
            loop_ub = r3->size[0] * r3->size[1];
            for (i = 0; i < loop_ub; i++) {
              b_y_data[i] = r31[i];
            }

            for (i = 0; i < d_loop_ub; i++) {
              for (i1 = 0; i1 < d_loop_ub; i1++) {
                BattConfigPerIna_data[(c_loop_ub + b_loop_ub_tmp * i1) +
                  b_loop_ub_tmp * d_loop_ub * i] = b_y_data[i1 + d_loop_ub * i];
              }
            }

            /* nadav coder */
            /*  BattConfigPerIna{k_ina219} = prm.cnfg.BattConfigDis1;%BattConfigPerInaHelp{k_ina219}; */
            /*  BattConfig{k_ina219}       = BattConfigPerIna{k_ina219} + (k_ina219-1)*N_bat; */
          } else if (i == 1) {
            e_padArrUint8(prm->cnfg.BattConfigChr1, N_bat_tmp_tmp_tmp,
                          N_bat_tmp_tmp_tmp, r3);
            r31 = r3->data;
            i = r->size[0] * r->size[1];
            r->size[0] = r3->size[0];
            r->size[1] = r3->size[1];
            emxEnsureCapacity_real_T(r, i);
            b_y_data = r->data;
            loop_ub = r3->size[0] * r3->size[1];
            for (i = 0; i < loop_ub; i++) {
              b_y_data[i] = r31[i];
            }

            for (i = 0; i < d_loop_ub; i++) {
              for (i1 = 0; i1 < d_loop_ub; i1++) {
                BattConfigPerIna_data[(c_loop_ub + b_loop_ub_tmp * i1) +
                  b_loop_ub_tmp * d_loop_ub * i] = b_y_data[i1 + d_loop_ub * i];
              }
            }

            /* nadav coder */
            /*  BattConfigPerIna{k_ina219} = prm.cnfg.BattConfigChr1;%BattConfigPerInaHelp{k_ina219}; */
            /*  BattConfig{k_ina219}       = BattConfigPerIna{k_ina219} + (k_ina219-1)*N_bat; */
          }
        }
      }

      if (end - 1 >= 0) {
        memcpy(&BattConfigPerInaHelp_data[0], &BattConfigPerIna_data[0],
               (unsigned int)end * sizeof(double));
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
        b_trueCount = (changeConfigFlag->size[1] != 1);
        for (i = 0; i < loop_ub; i++) {
          id2Bypass_size = i * b_trueCount;
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
            y->size[0] = 1;
            y->size[1] = 0;
          } else {
            i = y->size[0] * y->size[1];
            y->size[0] = 1;
            y->size[1] = (int)(Nina219_tmp_tmp - 1.0) + 1;
            emxEnsureCapacity_real_T(y, i);
            y_data = y->data;
            loop_ub = (int)(Nina219_tmp_tmp - 1.0);
            for (i = 0; i <= loop_ub; i++) {
              y_data[i] = (double)i + 1.0;
            }
          }

          i = b_y->size[0];
          b_y->size[0] = y->size[1];
          emxEnsureCapacity_real_T(b_y, i);
          b_y_data = b_y->data;
          loop_ub = y->size[1];
          for (i = 0; i < loop_ub; i++) {
            b_y_data[i] = y_data[i];
          }

          loop_ub = y->size[1];
          for (i = 0; i < loop_ub; i++) {
            VecIna219_data[i] = b_y_data[i];
          }

          if (b_size - 1 >= 0) {
            b_end = ipos_size - 1;
          }

          for (k_tstState = 0; k_tstState < b_size; k_tstState++) {
            trueCount = 0;
            partialTrueCount = 0;
            for (loop_ub = 0; loop_ub <= b_end; loop_ub++) {
              if (k_tstState + 1 == ipos_data[loop_ub]) {
                trueCount++;
                k_tmp_data[partialTrueCount] = (signed char)loop_ub;
                partialTrueCount++;
              }
            }

            /* VecIna219_k = find(k_groups == uGroupId); */
            AllBypassPerGroup_size[0] = 1;
            AllBypassPerGroup_size[1] = trueCount;
            for (i = 0; i < trueCount; i++) {
              AllBypassPerGroup_data[i] = switchFlag_data[(int)
                VecIna219_data[k_tmp_data[i]] - 1];
            }

            b_AllBypassPerGroup_data.data = &AllBypassPerGroup_data[0];
            b_AllBypassPerGroup_data.size = &AllBypassPerGroup_size[0];
            b_AllBypassPerGroup_data.allocatedSize = 2;
            b_AllBypassPerGroup_data.numDimensions = 2;
            b_AllBypassPerGroup_data.canFreeData = false;
            if (b_any(&b_AllBypassPerGroup_data)) {
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
                emxEnsureCapacity_real_T(y, i);
                y_data = y->data;
                loop_ub = N_bat_tmp_tmp_tmp - 1;
                for (i = 0; i <= loop_ub; i++) {
                  y_data[i] = (double)i + 1.0;
                }
              }

              if (trueCount == y->size[1]) {
                i = indVperGrp->size[0];
                indVperGrp->size[0] = y->size[1];
                emxEnsureCapacity_real_T(indVperGrp, i);
                VbusTest2_data = indVperGrp->data;
                loop_ub = y->size[1];
                for (i = 0; i < loop_ub; i++) {
                  VbusTest2_data[i] = y_data[i] + (VecIna219_data[k_tmp_data[i]]
                    - 1.0) * (double)N_bat_tmp_tmp_tmp;
                }
              } else {
                binary_expand_op_3(indVperGrp, y, VecIna219_data, k_tmp_data,
                                   &trueCount, N_bat_tmp_tmp_tmp);
                VbusTest2_data = indVperGrp->data;
              }

              c_tmp_size[0] = trueCount;
              c_tmp_size[1] = d_loop_ub;
              c_tmp_size[2] = d_loop_ub;
              for (i = 0; i < d_loop_ub; i++) {
                for (i1 = 0; i1 < d_loop_ub; i1++) {
                  for (i2 = 0; i2 < trueCount; i2++) {
                    n_tmp_data[(i2 + trueCount * i1) + trueCount * d_loop_ub * i]
                      = BattConfigPerIna_data[(((int)
                      VecIna219_data[k_tmp_data[i2]] + b_loop_ub_tmp * i1) +
                      b_loop_ub_tmp * d_loop_ub * i) - 1];
                  }
                }
              }

              d_tmp_size[0] = 1;
              d_tmp_size[1] = trueCount;
              for (i = 0; i < trueCount; i++) {
                outVI[i] = SelDual_data[(int)VecIna219_data[k_tmp_data[i]] - 1];
              }

              e_tmp_size[0] = 1;
              e_tmp_size[1] = trueCount;
              for (i = 0; i < trueCount; i++) {
                o_tmp_data[i] = tLastToggle_data[(int)
                  VecIna219_data[k_tmp_data[i]] - 1];
              }

              f_tmp_size[0] = N_bat_tmp_tmp_tmp;
              f_tmp_size[1] = trueCount;
              for (i = 0; i < trueCount; i++) {
                for (i1 = 0; i1 < N_bat_tmp_tmp_tmp; i1++) {
                  c_VbusTest_data[i1 + N_bat_tmp_tmp_tmp * i] =
                    BattConfigAct_data[i1 + N_bat_tmp_tmp_tmp * ((int)
                    VecIna219_data[k_tmp_data[i]] - 1)];
                }
              }

              b_k_t0[0] = (double)(k_t0 + 1) - 1.0;
              b_k_t0[1] = 1.0;
              i = (int)b_maximum(b_k_t0);
              partialTrueCount = indVperGrp->size[0];
              loop_ub = indVperGrp->size[0];
              for (i1 = 0; i1 < loop_ub; i1++) {
                c_VmKp184Test_data[i1] = ImKp184Test2_data[((int)
                  VbusTest2_data[i1] + Vbat->size[0] * (i - 1)) - 1];
              }

              tLastToggle_size[0] = 1;
              tLastToggle_size[1] = trueCount;
              for (i = 0; i < trueCount; i++) {
                i_prm[i] = ((float)tLastToggle_data[(int)
                            VecIna219_data[k_tmp_data[i]] - 1] +
                            prm->cnfg.Ttoggle) + 1.0F;
              }

              i = (int)prm->seq.vth[state_k] - 1;
              showVdiff = c_Esp32StepSwitchToggleCombAll_(outVI, d_tmp_size,
                prm->bat.CutOffChrV[i], prm->bat.CutOffDisV[i], n_tmp_data,
                c_tmp_size, c_VmKp184Test_data, partialTrueCount, trueCount,
                prm->brd.Nbat, prm->brd.spi.disconnect, prm->brd.spi.bypass,
                prm->cnfg.Ttoggle, prm->cnfg.NtoggleDrop, prm->cnfg.minLenIna219,
                prm->seq.pwr, prm->seq.pwr.VthFlag[state_k], i_prm,
                tLastToggle_size, o_tmp_data, e_tmp_size, c_VbusTest_data,
                f_tmp_size, AllBypassPerGroup_data, AllBypassPerGroup_size);
              for (i = 0; i < trueCount; i++) {
                id2Bypass_data[i] = (int)VecIna219_data[k_tmp_data[i]] - 1;
              }

              for (i = 0; i < d_loop_ub; i++) {
                for (i1 = 0; i1 < d_loop_ub; i1++) {
                  for (i2 = 0; i2 < trueCount; i2++) {
                    BattConfigPerInaHelp_data[(id2Bypass_data[i2] +
                      b_loop_ub_tmp * i1) + b_loop_ub_tmp * d_loop_ub * i] =
                      n_tmp_data[(i2 + trueCount * i1) + trueCount * d_loop_ub *
                      i];
                  }
                }
              }

              for (i = 0; i < trueCount; i++) {
                id2Bypass_data[i] = (int)VecIna219_data[k_tmp_data[i]];
              }

              c_loop_ub_tmp = trueCount - 1;
              for (i = 0; i <= c_loop_ub_tmp; i++) {
                SelDualHelp_data[id2Bypass_data[i] - 1] = outVI[i];
              }

              for (i = 0; i < trueCount; i++) {
                id2Bypass_data[i] = (int)VecIna219_data[k_tmp_data[i]];
              }

              for (i = 0; i <= c_loop_ub_tmp; i++) {
                switchFlagHelp_data[id2Bypass_data[i] - 1] = showVdiff;
              }

              Nina219[0] = N_bat_tmp_tmp_tmp;
              Nina219[1] = trueCount;
              for (i = 0; i < trueCount; i++) {
                id2Bypass_data[i] = (int)VecIna219_data[k_tmp_data[i]] - 1;
                loop_ub = Nina219[0];
                for (i1 = 0; i1 < loop_ub; i1++) {
                  BattConfigAct_data[i1 + N_bat_tmp_tmp_tmp * id2Bypass_data[i]]
                    = c_VbusTest_data[i1 + Nina219[0] * i];
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
            i = x->size[0] * x->size[1];
            x->size[0] = 1;
            x->size[1] = switchFlagHelp->size[1];
            emxEnsureCapacity_boolean_T(x, i);
            b_changeConfigFlag_data = x->data;
            loop_ub = switchFlagHelp->size[1];
            for (i = 0; i < loop_ub; i++) {
              b_changeConfigFlag_data[i] = !switchFlagHelp_data[i];
            }

            showVdiff = true;
            b_trueCount = 1;
            exitg2 = false;
            while ((!exitg2) && (b_trueCount <= x->size[1])) {
              if (!b_changeConfigFlag_data[b_trueCount - 1]) {
                showVdiff = false;
                exitg2 = true;
              } else {
                b_trueCount++;
              }
            }

            if (showVdiff) {
              for (c_loop_ub = 0; c_loop_ub < VbusTest_size_idx_0; c_loop_ub++)
              {
                /* Start new mode */
                if (switchFlag_data[c_loop_ub]) {
                  /* nadav Coder */
                  d_tmp_data = SelDual_data[c_loop_ub];
                  if (d_tmp_data == 0.0) {
                    for (i = 0; i < d_loop_ub; i++) {
                      for (i1 = 0; i1 < d_loop_ub; i1++) {
                        b_BattConfigPerInaHelp_data[i1 + d_loop_ub * i] =
                          BattConfigPerInaHelp_data[(c_loop_ub + b_loop_ub_tmp *
                          i1) + b_loop_ub_tmp * d_loop_ub * i];
                      }
                    }

                    for (i = 0; i < d_loop_ub; i++) {
                      for (i1 = 0; i1 < d_loop_ub; i1++) {
                        BattConfigPerIna_data[(c_loop_ub + b_loop_ub_tmp * i1) +
                          b_loop_ub_tmp * d_loop_ub * i] =
                          b_BattConfigPerInaHelp_data[i1 + d_loop_ub * i];
                      }
                    }
                  } else if (d_tmp_data == 1.0) {
                    for (i = 0; i < d_loop_ub; i++) {
                      for (i1 = 0; i1 < d_loop_ub; i1++) {
                        b_BattConfigPerInaHelp_data[i1 + d_loop_ub * i] =
                          BattConfigPerInaHelp_data[(c_loop_ub + b_loop_ub_tmp *
                          i1) + b_loop_ub_tmp * d_loop_ub * i];
                      }
                    }

                    for (i = 0; i < d_loop_ub; i++) {
                      for (i1 = 0; i1 < d_loop_ub; i1++) {
                        BattConfigPerIna_data[(c_loop_ub + b_loop_ub_tmp * i1) +
                          b_loop_ub_tmp * d_loop_ub * i] =
                          b_BattConfigPerInaHelp_data[i1 + d_loop_ub * i];
                      }
                    }
                  } else if (d_tmp_data == 1.5) {
                    for (i = 0; i < d_loop_ub; i++) {
                      for (i1 = 0; i1 < d_loop_ub; i1++) {
                        b_BattConfigPerInaHelp_data[i1 + d_loop_ub * i] =
                          BattConfigPerInaHelp_data[(c_loop_ub + b_loop_ub_tmp *
                          i1) + b_loop_ub_tmp * d_loop_ub * i];
                      }
                    }

                    for (i = 0; i < d_loop_ub; i++) {
                      for (i1 = 0; i1 < d_loop_ub; i1++) {
                        BattConfigPerIna_data[(c_loop_ub + b_loop_ub_tmp * i1) +
                          b_loop_ub_tmp * d_loop_ub * i] =
                          b_BattConfigPerInaHelp_data[i1 + d_loop_ub * i];
                      }
                    }
                  } else if (d_tmp_data == 1.6) {
                    for (i = 0; i < d_loop_ub; i++) {
                      for (i1 = 0; i1 < d_loop_ub; i1++) {
                        b_BattConfigPerInaHelp_data[i1 + d_loop_ub * i] =
                          BattConfigPerInaHelp_data[(c_loop_ub + b_loop_ub_tmp *
                          i1) + b_loop_ub_tmp * d_loop_ub * i];
                      }
                    }

                    for (i = 0; i < d_loop_ub; i++) {
                      for (i1 = 0; i1 < d_loop_ub; i1++) {
                        BattConfigPerIna_data[(c_loop_ub + b_loop_ub_tmp * i1) +
                          b_loop_ub_tmp * d_loop_ub * i] =
                          b_BattConfigPerInaHelp_data[i1 + d_loop_ub * i];
                      }
                    }
                  } else if (d_tmp_data == prm->brd.spi.disconnect) {
                    b_padArr(prm->brd.spi.disconnect, N_bat_tmp_tmp_tmp,
                             N_bat_tmp_tmp_tmp, r);
                    b_y_data = r->data;
                    for (i = 0; i < d_loop_ub; i++) {
                      for (i1 = 0; i1 < d_loop_ub; i1++) {
                        BattConfigPerIna_data[(c_loop_ub + b_loop_ub_tmp * i1) +
                          b_loop_ub_tmp * d_loop_ub * i] = b_y_data[i1 +
                          d_loop_ub * i];
                      }
                    }
                  } else if (d_tmp_data == prm->brd.spi.bypass) {
                    b_padArr(prm->brd.spi.bypass, N_bat_tmp_tmp_tmp,
                             N_bat_tmp_tmp_tmp, r);
                    b_y_data = r->data;
                    for (i = 0; i < d_loop_ub; i++) {
                      for (i1 = 0; i1 < d_loop_ub; i1++) {
                        BattConfigPerIna_data[(c_loop_ub + b_loop_ub_tmp * i1) +
                          b_loop_ub_tmp * d_loop_ub * i] = b_y_data[i1 +
                          d_loop_ub * i];
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
          for (c_loop_ub = 0; c_loop_ub < VbusTest_size_idx_0; c_loop_ub++) {
            /* Start new mode */
            if (switchFlag_data[c_loop_ub]) {
              d_tmp_data = SelDual_data[c_loop_ub];
              if (d_tmp_data == 0.0) {
                e_padArrUint8(prm->cnfg.BattConfigDis1, N_bat_tmp_tmp_tmp,
                              N_bat_tmp_tmp_tmp, r3);
                r31 = r3->data;
                i = r->size[0] * r->size[1];
                r->size[0] = r3->size[0];
                r->size[1] = r3->size[1];
                emxEnsureCapacity_real_T(r, i);
                b_y_data = r->data;
                loop_ub = r3->size[0] * r3->size[1];
                for (i = 0; i < loop_ub; i++) {
                  b_y_data[i] = r31[i];
                }

                for (i = 0; i < d_loop_ub; i++) {
                  for (i1 = 0; i1 < d_loop_ub; i1++) {
                    BattConfigPerIna_data[(c_loop_ub + b_loop_ub_tmp * i1) +
                      b_loop_ub_tmp * d_loop_ub * i] = b_y_data[i1 + d_loop_ub *
                      i];
                  }
                }
              } else if (d_tmp_data == 1.0) {
                e_padArrUint8(prm->cnfg.BattConfigChr1, N_bat_tmp_tmp_tmp,
                              N_bat_tmp_tmp_tmp, r3);
                r31 = r3->data;
                i = r->size[0] * r->size[1];
                r->size[0] = r3->size[0];
                r->size[1] = r3->size[1];
                emxEnsureCapacity_real_T(r, i);
                b_y_data = r->data;
                loop_ub = r3->size[0] * r3->size[1];
                for (i = 0; i < loop_ub; i++) {
                  b_y_data[i] = r31[i];
                }

                for (i = 0; i < d_loop_ub; i++) {
                  for (i1 = 0; i1 < d_loop_ub; i1++) {
                    BattConfigPerIna_data[(c_loop_ub + b_loop_ub_tmp * i1) +
                      b_loop_ub_tmp * d_loop_ub * i] = b_y_data[i1 + d_loop_ub *
                      i];
                  }
                }
              } else if (d_tmp_data == 1.5) {
                e_padArrUint8(prm->cnfg.BattConfigChr2, N_bat_tmp_tmp_tmp,
                              N_bat_tmp_tmp_tmp, r3);
                r31 = r3->data;
                i = r->size[0] * r->size[1];
                r->size[0] = r3->size[0];
                r->size[1] = r3->size[1];
                emxEnsureCapacity_real_T(r, i);
                b_y_data = r->data;
                loop_ub = r3->size[0] * r3->size[1];
                for (i = 0; i < loop_ub; i++) {
                  b_y_data[i] = r31[i];
                }

                for (i = 0; i < d_loop_ub; i++) {
                  for (i1 = 0; i1 < d_loop_ub; i1++) {
                    BattConfigPerIna_data[(c_loop_ub + b_loop_ub_tmp * i1) +
                      b_loop_ub_tmp * d_loop_ub * i] = b_y_data[i1 + d_loop_ub *
                      i];
                  }
                }
              } else if (d_tmp_data == 1.6) {
                e_padArrUint8(prm->cnfg.BattConfigChr3, N_bat_tmp_tmp_tmp,
                              N_bat_tmp_tmp_tmp, r3);
                r31 = r3->data;
                i = r->size[0] * r->size[1];
                r->size[0] = r3->size[0];
                r->size[1] = r3->size[1];
                emxEnsureCapacity_real_T(r, i);
                b_y_data = r->data;
                loop_ub = r3->size[0] * r3->size[1];
                for (i = 0; i < loop_ub; i++) {
                  b_y_data[i] = r31[i];
                }

                for (i = 0; i < d_loop_ub; i++) {
                  for (i1 = 0; i1 < d_loop_ub; i1++) {
                    BattConfigPerIna_data[(c_loop_ub + b_loop_ub_tmp * i1) +
                      b_loop_ub_tmp * d_loop_ub * i] = b_y_data[i1 + d_loop_ub *
                      i];
                  }
                }
              } else if (d_tmp_data == prm->brd.spi.disconnect) {
                f_padArrUint8(prm->brd.spi.disconnect, N_bat_tmp_tmp_tmp,
                              N_bat_tmp_tmp_tmp, r3);
                r31 = r3->data;
                i = r->size[0] * r->size[1];
                r->size[0] = r3->size[0];
                r->size[1] = r3->size[1];
                emxEnsureCapacity_real_T(r, i);
                b_y_data = r->data;
                loop_ub = r3->size[0] * r3->size[1];
                for (i = 0; i < loop_ub; i++) {
                  b_y_data[i] = r31[i];
                }

                for (i = 0; i < d_loop_ub; i++) {
                  for (i1 = 0; i1 < d_loop_ub; i1++) {
                    BattConfigPerIna_data[(c_loop_ub + b_loop_ub_tmp * i1) +
                      b_loop_ub_tmp * d_loop_ub * i] = b_y_data[i1 + d_loop_ub *
                      i];
                  }
                }
              } else if (d_tmp_data == prm->brd.spi.bypass) {
                f_padArrUint8(prm->brd.spi.bypass, N_bat_tmp_tmp_tmp,
                              N_bat_tmp_tmp_tmp, r3);
                r31 = r3->data;
                i = r->size[0] * r->size[1];
                r->size[0] = r3->size[0];
                r->size[1] = r3->size[1];
                emxEnsureCapacity_real_T(r, i);
                b_y_data = r->data;
                loop_ub = r3->size[0] * r3->size[1];
                for (i = 0; i < loop_ub; i++) {
                  b_y_data[i] = r31[i];
                }

                for (i = 0; i < d_loop_ub; i++) {
                  for (i1 = 0; i1 < d_loop_ub; i1++) {
                    BattConfigPerIna_data[(c_loop_ub + b_loop_ub_tmp * i1) +
                      b_loop_ub_tmp * d_loop_ub * i] = b_y_data[i1 + d_loop_ub *
                      i];
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
          for (c_loop_ub = 0; c_loop_ub < VbusTest_size_idx_0; c_loop_ub++) {
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
            if (changeConfigFlag_data[c_loop_ub] || switchFlag_data[c_loop_ub])
            {
              /* [SpiPortRow0,ina219State,Pac2Vid0] = CalcPortSpi(SwitchMat_esp,SwitchMat_esp2,PortSpiRow_esp,PortSpiRow_esp2,Pac2Vid,Pac2Vid2,BattConfigPerIna{k_ina219}); */
              for (i = 0; i < d_loop_ub; i++) {
                for (i1 = 0; i1 < d_loop_ub; i1++) {
                  b_BattConfigPerInaHelp_data[i1 + d_loop_ub * i] =
                    BattConfigPerIna_data[(c_loop_ub + b_loop_ub_tmp * i1) +
                    b_loop_ub_tmp * d_loop_ub * i];
                }
              }

              b_trueCount = prm->Nmax.NbatMax;
              Nina219[0] = prm->Nmax.NbatMax;
              Nina219[1] = prm->Nmax.NbatMax;
              c_BattConfigPerInaHelp_data.data = &b_BattConfigPerInaHelp_data[0];
              c_BattConfigPerInaHelp_data.size = &Nina219[0];
              c_BattConfigPerInaHelp_data.allocatedSize = 256;
              c_BattConfigPerInaHelp_data.numDimensions = 2;
              c_BattConfigPerInaHelp_data.canFreeData = false;
              d_CalcPortSpiPrm(prm->brd.spi.disconnect, prm->brd.spi.bypass,
                               prm->brd.spi.PortSpiRow_esp,
                               prm->brd.spi.SwitchMat_esp,
                               prm->brd.spi.PortSpiRow_esp2,
                               prm->brd.spi.SwitchMat_esp2, prm->brd.spi.Pac2Vid,
                               prm->brd.spi.Pac2Vid2,
                               &c_BattConfigPerInaHelp_data, SpiPortRowBypass,
                               a__1_data, a__1_size, id0_data);
              for (i = 0; i < d_loop_ub; i++) {
                for (i1 = 0; i1 < d_loop_ub; i1++) {
                  Pac2Vid0All_data[(c_loop_ub + Pac2Vid0All_size[0] * i1) +
                    Pac2Vid0All_size[0] * d_loop_ub * i] = id0_data[i1 + (i << 4)];
                }
              }

              if (prm->seq.BrdBeforePSflag[state_k]) {
                switch (ProjectFlag) {
                 case 2:
                  break;

                 case 3:
                  writePortToSpi4RowMask_ser(SpiPortRowBypass);
                  break;
                }

                /*                      disp([sprintf('%d',int32(k_ina219)) ':',num2str(Pac2Vid0(:,1)')]); */
                if ((unsigned int)c_loop_ub + 1U < 2147483648U) {
                  i = c_loop_ub + 1;
                } else {
                  i = MAX_int32_T;
                }

                b_sprintf(i, r19);
                for (id2Bypass_size = 0; id2Bypass_size < b_trueCount;
                     id2Bypass_size++) {
                  NstTst_tmp_tmp = id0_data[id2Bypass_size];
                  if (NstTst_tmp_tmp > 127) {
                    NstTst_tmp_tmp = 127U;
                  }

                  printf(" %d", (signed char)NstTst_tmp_tmp);
                  fflush(stdout);
                }

                changeConfigFlag_data[c_loop_ub] = false;
                switchFlag_data[c_loop_ub] = false;
                pause(0.05);
              }
            }
          }

          /* for k_ina219 */
          /*  change switch - End */
          b_dv0 = 0.0;
          V_dis_all = 0.0;

          /*  IchrPerPhase = zeros(1,sum(SelDual)); */
          /*  k0 = 1; */
          b_meanIbrd0 = 0.0;
          N0 = 0.0;
          for (c_loop_ub = 0; c_loop_ub < VbusTest_size_idx_0; c_loop_ub++) {
            d_tmp_data = SelDual_data[c_loop_ub];
            if ((d_tmp_data >= 1.0) && (d_tmp_data < 2.0)) {
              bitCnfg_size[0] = 1;
              bitCnfg_size[1] = d_loop_ub;
              bitCnfg_size[2] = Pac2Vid0All_size[2];
              for (i = 0; i < d_loop_ub; i++) {
                for (i1 = 0; i1 < d_loop_ub; i1++) {
                  b_bitCnfg_data[i1 + d_loop_ub * i] = Pac2Vid0All_data
                    [(c_loop_ub + Pac2Vid0All_size[0] * i1) + Pac2Vid0All_size[0]
                    * d_loop_ub * i];
                }
              }

              c_squeeze(b_bitCnfg_data, bitCnfg_size, id0_data, id2Bypass_data);

              /* Pac2Vid3(:,1); */
              if (k_t0 >= 1) {
                k_ina219_ = k_t0;
              } else {
                k_ina219_ = 1;
              }

              /* ins */
              id2Bypass_size = id2Bypass_data[0] * id2Bypass_data[1] - 1;
              trueCount = 0;
              partialTrueCount = 0;
              for (loop_ub = 0; loop_ub <= id2Bypass_size; loop_ub++) {
                if (id0_data[loop_ub] > 0) {
                  trueCount++;
                  l_tmp_data[partialTrueCount] = (unsigned char)loop_ub;
                  partialTrueCount++;
                }
              }

              VmKp184Test_size[0] = 1;
              VmKp184Test_size[1] = trueCount;
              for (i = 0; i < trueCount; i++) {
                c_VmKp184Test_data[i] = VbatMat0_data[(c_loop_ub +
                  VbatMat0->size[0] * (id0_data[l_tmp_data[i]] - 1)) +
                  VbatMat0->size[0] * VbatMat0->size[1] * (k_ina219_ - 1)];
              }

              b_dv0 += c_sum(c_VmKp184Test_data, VmKp184Test_size);

              /* +I0'*Rwire(k_ina219,:);%prm.brd.Rint; */
              /*  limit Vchr calculation */
              /*  if SelDual(k_ina219)>=1&&SelDual(k_ina219)<2 */
              BattConfigPerInaHelp_size[0] = 1;
              BattConfigPerInaHelp_size[1] = d_loop_ub;
              BattConfigPerInaHelp_size[2] = d_loop_ub;
              for (i = 0; i < d_loop_ub; i++) {
                for (i1 = 0; i1 < d_loop_ub; i1++) {
                  b_BattConfigPerInaHelp_data[i1 + d_loop_ub * i] =
                    BattConfigPerIna_data[(c_loop_ub + b_loop_ub_tmp * i1) +
                    b_loop_ub_tmp * d_loop_ub * i];
                }
              }

              remPadArr(b_BattConfigPerInaHelp_data, BattConfigPerInaHelp_size,
                        VmV_data, b_tmp_size);
              N0 += (double)(unsigned short)b_tmp_size[0];

              /* size(BattConfigPerIna{k_ina0},1);%N_row(ina219StateAll(k_ina0)); */
              /*  end */
              IbatMat_size[0] = 1;
              IbatMat_size[1] = IbatMat->size[1];
              loop_ub = IbatMat->size[1];
              for (i = 0; i < loop_ub; i++) {
                b_VmV_data[i] = Iacs758_cal_data[(c_loop_ub + IbatMat->size[0] *
                  i) + IbatMat->size[0] * IbatMat->size[1] * (k_ina219_ - 1)];
              }

              b_abs(b_VmV_data, IbatMat_size, c_VmKp184Test_data,
                    VmKp184Test_size);
              d_tmp_data = 0.0;
              loop_ub = VmKp184Test_size[1];
              for (i = 0; i < loop_ub; i++) {
                a__4_data = c_VmKp184Test_data[i];
                if ((g_IchargeAct >= a__4_data) || rtIsNaN(a__4_data)) {
                  d = g_IchargeAct;
                } else {
                  d = a__4_data;
                }

                d_tmp_data += d * Rwire_data[c_loop_ub + Rwire_size[0] * i];
              }

              b_meanIbrd0 += d_tmp_data;

              /* prm.brd.Rint; */
            }

            d_tmp_data = SelDual_data[c_loop_ub];
            if ((d_tmp_data >= 0.0) && (d_tmp_data < 1.0)) {
              bitCnfg_size[0] = 1;
              bitCnfg_size[1] = d_loop_ub;
              bitCnfg_size[2] = Pac2Vid0All_size[2];
              for (i = 0; i < d_loop_ub; i++) {
                for (i1 = 0; i1 < d_loop_ub; i1++) {
                  b_bitCnfg_data[i1 + d_loop_ub * i] = Pac2Vid0All_data
                    [(c_loop_ub + Pac2Vid0All_size[0] * i1) + Pac2Vid0All_size[0]
                    * d_loop_ub * i];
                }
              }

              c_squeeze(b_bitCnfg_data, bitCnfg_size, id0_data, id2Bypass_data);

              /* Pac2Vid3(:,1); */
              if (k_t0 >= 1) {
                b_trueCount = k_t0;
              } else {
                b_trueCount = 1;
              }

              id2Bypass_size = id2Bypass_data[0] * id2Bypass_data[1] - 1;
              trueCount = 0;
              partialTrueCount = 0;
              for (loop_ub = 0; loop_ub <= id2Bypass_size; loop_ub++) {
                if (id0_data[loop_ub] > 0) {
                  trueCount++;
                  m_tmp_data[partialTrueCount] = (unsigned char)loop_ub;
                  partialTrueCount++;
                }
              }

              VmKp184Test_size[0] = 1;
              VmKp184Test_size[1] = trueCount;
              for (i = 0; i < trueCount; i++) {
                c_VmKp184Test_data[i] = VbatMat0_data[(c_loop_ub +
                  VbatMat0->size[0] * (id0_data[m_tmp_data[i]] - 1)) +
                  VbatMat0->size[0] * VbatMat0->size[1] * (b_trueCount - 1)];
              }

              V_dis_all += c_sum(c_VmKp184Test_data, VmKp184Test_size);

              /* -I0'*Rwire(k_ina219,:);%prm.brd.Rint; */
            }
          }

          e_Vcharge0 = (prm->bat.Vd + (c_maximum(prm->bat.CutOffChrV) + 0.1F) *
                        (float)N0) + (float)b_meanIbrd0;
          if (b_dv0 > 0.0) {
            ImaxChrB2B = (float)(V_dis_all / b_dv0) * ImaxDis *
              juntekEfficencyFactor;
          } else {
            /* k_t = 1; */
            V_dis_all = prm->ins.prm.jun.minVjuntekInput + 1.0;
            ImaxChrB2B = ImaxDis * juntekEfficencyFactor;
          }

          /* calc max I allowed & V discharge - End */
          b_dv0 = toc();
          f = b_mod(b_dv0, prm->bat.T);
          for (id2Bypass_size = 0; id2Bypass_size < 1024; id2Bypass_size++) {
            varargin_1[id2Bypass_size] = (float)fabs(prm->bat.t[id2Bypass_size]
              - f);
          }

          c_minimum(varargin_1, &b_trueCount);
          c_vSumMax = rt_roundf_snf(1000.0F * prm->bat.i_in[b_trueCount - 1]) /
            1000.0F;

          /* calc I to discharge- End */
          if (onPowerFlag) {
            /*  if prm.ins.juntek */
            /*      controlJuntekDPH8920(prm.ser.s_juntek,'ON'); */
            /*  end */
            /*  if prm.ins.ka6005p */
            /*      ControlKa6005P(prm.ser.s_ka6005p,'On'); */
            /*  end */
            for (i = 0; i < 8; i++) {
              VbusTest0_data[i] = (prm->seq.ins[i + (state_k << 3)] == 1);
            }

            if (vectorAny(VbusTest0_data)) {
              /* prm.ins.kp184 */
              pause(0.05);
            }

            for (i = 0; i < 8; i++) {
              VbusTest0_data[i] = (prm->seq.ins[i + (state_k << 3)] == 2);
            }

            if (vectorAny(VbusTest0_data)) {
              /* prm.ins.ka6005p */
              c_loop_ub = 0;
              exitg2 = false;
              while ((!exitg2) && (c_loop_ub <= (int)Nina219_tmp_tmp - 1)) {
                if (SelDual_data[c_loop_ub] == 1.0) {
                  b_trueCount = 0;
                } else if (SelDual_data[c_loop_ub] == 1.5) {
                  b_trueCount = 1;
                } else if (SelDual_data[c_loop_ub] == 1.6) {
                  b_trueCount = 2;
                } else {
                  b_trueCount = -1;
                }

                switch (b_trueCount) {
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
                  c_loop_ub++;
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
              VbusTest0_data[i] = (prm->seq.ins[i + (state_k << 3)] == 3);
            }

            if (vectorAny(VbusTest0_data)) {
              /*  3-juntek+ACDC */
              c_loop_ub = 0;
              exitg2 = false;
              while ((!exitg2) && (c_loop_ub <= (int)Nina219_tmp_tmp - 1)) {
                if (SelDual_data[c_loop_ub] == 1.0) {
                  b_trueCount = 0;
                } else if (SelDual_data[c_loop_ub] == 1.5) {
                  b_trueCount = 1;
                } else if (SelDual_data[c_loop_ub] == 1.6) {
                  b_trueCount = 2;
                } else {
                  b_trueCount = -1;
                }

                switch (b_trueCount) {
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
                  c_loop_ub++;
                  break;
                }
              }

              /* Imax); */
              b_k_t0[0] = g_IchargeAct;
              b_k_t0[1] = ImaxAcDC;
              g_IchargeAct = rt_roundd_snf(1000.0 * d_minimum(b_k_t0)) / 1000.0;
              d_tmp_data = rt_roundd_snf(g_IchargeAct * 1000.0);
              if (d_tmp_data < 2.147483648E+9) {
                if (d_tmp_data >= -2.147483648E+9) {
                  i = (int)d_tmp_data;
                } else {
                  i = MIN_int32_T;
                }
              } else if (d_tmp_data >= 2.147483648E+9) {
                i = MAX_int32_T;
              } else {
                i = 0;
              }

              g_sprintf(i, r23);

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

              g_sprintf(i, r26);

              /* [':',juntek_address,'w10=',num2str(uint32(Val*100),'%04d'),',',newline];%['VSET',num2str(ch),':',num2str(Val)];%[V] */
              k_sprintf(e_Vcharge0, c_Vcharge0);
              pause(0.1);
            }

            for (i = 0; i < 8; i++) {
              VbusTest0_data[i] = (prm->seq.ins[i + (state_k << 3)] == 4);
            }

            if (vectorAny(VbusTest0_data)) {
              /*  4-juntek B2B */
              c_loop_ub = 0;
              exitg2 = false;
              while ((!exitg2) && (c_loop_ub <= (int)Nina219_tmp_tmp - 1)) {
                if (SelDual_data[c_loop_ub] == 1.0) {
                  b_trueCount = 0;
                } else if (SelDual_data[c_loop_ub] == 1.5) {
                  b_trueCount = 1;
                } else if (SelDual_data[c_loop_ub] == 1.6) {
                  b_trueCount = 2;
                } else {
                  b_trueCount = -1;
                }

                switch (b_trueCount) {
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
                  c_loop_ub++;
                  break;
                }
              }

              /* Imax); */
              b_k_t0[0] = g_IchargeAct;
              b_k_t0[1] = ImaxChrB2B;
              g_IchargeAct = rt_roundd_snf(1000.0 * d_minimum(b_k_t0)) / 1000.0;
              d_tmp_data = rt_roundd_snf(g_IchargeAct * 1000.0);
              if (d_tmp_data < 2.147483648E+9) {
                if (d_tmp_data >= -2.147483648E+9) {
                  i = (int)d_tmp_data;
                } else {
                  i = MIN_int32_T;
                }
              } else if (d_tmp_data >= 2.147483648E+9) {
                i = MAX_int32_T;
              } else {
                i = 0;
              }

              g_sprintf(i, r25);

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

              g_sprintf(i, r28);

              /* [':',juntek_address,'w10=',num2str(uint32(Val*100),'%04d'),',',newline];%['VSET',num2str(ch),':',num2str(Val)];%[V] */
              k_sprintf(e_Vcharge0, d_Vcharge0);
              pause(0.1);
            }
          } else {
            /*  if onPowerFlag == true; */
            for (i = 0; i < 8; i++) {
              k_groups_tmp = prm->seq.ins[i + (state_k << 3)];
              iv[i] = k_groups_tmp;
              VbusTest0_data[i] = (k_groups_tmp == 3);
            }

            if (vectorAny(VbusTest0_data)) {
              /*  3-juntek ACDC */
              i = b_changeConfigFlag->size[0] * b_changeConfigFlag->size[1];
              b_changeConfigFlag->size[0] = 1;
              b_changeConfigFlag->size[1] = SelDual->size[1];
              emxEnsureCapacity_boolean_T(b_changeConfigFlag, i);
              b_changeConfigFlag_data = b_changeConfigFlag->data;
              loop_ub = SelDual->size[1];
              for (i = 0; i < loop_ub; i++) {
                d_tmp_data = SelDual_data[i];
                b_changeConfigFlag_data[i] = ((d_tmp_data >= 1.0) && (d_tmp_data
                  < 2.0));
              }

              if (b_any(b_changeConfigFlag)) {
                c_loop_ub = 0;
                exitg2 = false;
                while ((!exitg2) && (c_loop_ub <= (int)Nina219_tmp_tmp - 1)) {
                  if (SelDual_data[c_loop_ub] == 1.0) {
                    b_trueCount = 0;
                  } else if (SelDual_data[c_loop_ub] == 1.5) {
                    b_trueCount = 1;
                  } else if (SelDual_data[c_loop_ub] == 1.6) {
                    b_trueCount = 2;
                  } else {
                    b_trueCount = -1;
                  }

                  switch (b_trueCount) {
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
                    c_loop_ub++;
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

                g_sprintf(i, r20);

                /* [':',juntek_address,'w10=',num2str(uint32(Val*100),'%04d'),',',newline];%['VSET',num2str(ch),':',num2str(Val)];%[V] */
                d_tmp_data = rt_roundd_snf(g_IchargeAct * 1000.0);
                if (d_tmp_data < 2.147483648E+9) {
                  if (d_tmp_data >= -2.147483648E+9) {
                    i = (int)d_tmp_data;
                  } else {
                    i = MIN_int32_T;
                  }
                } else if (d_tmp_data >= 2.147483648E+9) {
                  i = MAX_int32_T;
                } else {
                  i = 0;
                }

                g_sprintf(i, r22);

                /* [':',juntek_address,'w11=',num2str(uint32(Val*1000),'%04d'),',',newline];%['VSET',num2str(ch),':',num2str(Val)];%[V] */
                d_sprintf(g_IchargeAct, c_IchargeAct);
                pause(0.1);
              }
            }

            for (i = 0; i < 8; i++) {
              VbusTest0_data[i] = (iv[i] == 4);
            }

            if (vectorAny(VbusTest0_data)) {
              /*  4-juntek B2B */
              i = b_changeConfigFlag->size[0] * b_changeConfigFlag->size[1];
              b_changeConfigFlag->size[0] = 1;
              b_changeConfigFlag->size[1] = SelDual->size[1];
              emxEnsureCapacity_boolean_T(b_changeConfigFlag, i);
              b_changeConfigFlag_data = b_changeConfigFlag->data;
              loop_ub = SelDual->size[1];
              for (i = 0; i < loop_ub; i++) {
                d_tmp_data = SelDual_data[i];
                b_changeConfigFlag_data[i] = ((d_tmp_data >= 1.0) && (d_tmp_data
                  < 2.0));
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

                g_sprintf(i, r21);

                /* [':',juntek_address,'w10=',num2str(uint32(Val*100),'%04d'),',',newline];%['VSET',num2str(ch),':',num2str(Val)];%[V] */
                d_tmp_data = rt_roundd_snf(g_IchargeAct * 1000.0);
                if (d_tmp_data < 2.147483648E+9) {
                  if (d_tmp_data >= -2.147483648E+9) {
                    i = (int)d_tmp_data;
                  } else {
                    i = MIN_int32_T;
                  }
                } else if (d_tmp_data >= 2.147483648E+9) {
                  i = MAX_int32_T;
                } else {
                  i = 0;
                }

                g_sprintf(i, r24);

                /* [':',juntek_address,'w11=',num2str(uint32(Val*1000),'%04d'),',',newline];%['VSET',num2str(ch),':',num2str(Val)];%[V] */
                d_sprintf(g_IchargeAct, e_IchargeAct);
                pause(0.1);
              }
            }

            for (i = 0; i < 8; i++) {
              VbusTest0_data[i] = (iv[i] == 1);
            }

            if (vectorAny(VbusTest0_data)) {
              /* prm.ins.kp184 */
              pause(0.05);

              /*  ControlKp184(prm.ser.s_kp184,'On'); */
            }
          }

          onPowerFlag = false;
          pause(0.1);
          for (c_loop_ub = 0; c_loop_ub < VbusTest_size_idx_0; c_loop_ub++) {
            if (!prm->seq.BrdBeforePSflag[state_k]) {
              for (i = 0; i < d_loop_ub; i++) {
                for (i1 = 0; i1 < d_loop_ub; i1++) {
                  b_BattConfigPerInaHelp_data[i1 + d_loop_ub * i] =
                    BattConfigPerIna_data[(c_loop_ub + b_loop_ub_tmp * i1) +
                    b_loop_ub_tmp * d_loop_ub * i];
                }
              }

              b_trueCount = prm->Nmax.NbatMax;
              Nina219[0] = prm->Nmax.NbatMax;
              Nina219[1] = prm->Nmax.NbatMax;
              d_BattConfigPerInaHelp_data.data = &b_BattConfigPerInaHelp_data[0];
              d_BattConfigPerInaHelp_data.size = &Nina219[0];
              d_BattConfigPerInaHelp_data.allocatedSize = 256;
              d_BattConfigPerInaHelp_data.numDimensions = 2;
              d_BattConfigPerInaHelp_data.canFreeData = false;
              d_CalcPortSpiPrm(prm->brd.spi.disconnect, prm->brd.spi.bypass,
                               prm->brd.spi.PortSpiRow_esp,
                               prm->brd.spi.SwitchMat_esp,
                               prm->brd.spi.PortSpiRow_esp2,
                               prm->brd.spi.SwitchMat_esp2, prm->brd.spi.Pac2Vid,
                               prm->brd.spi.Pac2Vid2,
                               &d_BattConfigPerInaHelp_data, SpiPortRowBypass,
                               a__1_data, a__1_size, id0_data);
              for (i = 0; i < d_loop_ub; i++) {
                for (i1 = 0; i1 < d_loop_ub; i1++) {
                  Pac2Vid0All_data[(c_loop_ub + Pac2Vid0All_size[0] * i1) +
                    Pac2Vid0All_size[0] * d_loop_ub * i] = id0_data[i1 + (i << 4)];
                }
              }

              if (changeConfigFlag_data[c_loop_ub] || switchFlag_data[c_loop_ub])
              {
                switch (ProjectFlag) {
                 case 2:
                  break;

                 case 3:
                  writePortToSpi4RowMask_ser(SpiPortRowBypass);
                  break;
                }

                /*  disp([num2str(k_ina219) ':',num2str(Pac2Vid0(:,1)')]); */
                if ((unsigned int)c_loop_ub + 1U < 2147483648U) {
                  i = c_loop_ub + 1;
                } else {
                  i = MAX_int32_T;
                }

                b_sprintf(i, r27);
                for (id2Bypass_size = 0; id2Bypass_size < b_trueCount;
                     id2Bypass_size++) {
                  NstTst_tmp_tmp = id0_data[id2Bypass_size];
                  if (NstTst_tmp_tmp > 127) {
                    NstTst_tmp_tmp = 127U;
                  }

                  printf(" %d", (signed char)NstTst_tmp_tmp);
                  fflush(stdout);
                }

                changeConfigFlag_data[c_loop_ub] = false;
                switchFlag_data[c_loop_ub] = false;
                pause(0.05);
              }
            }
          }

          /* On power and load - End */
          b_ReadVI(Nina219_tmp_tmp, ProjectFlag, prm->brd.pac.VIpacId,
                   Pac2Vid0All_data, Pac2Vid0All_size, N_bat1, N_bat_tmp_tmp_tmp,
                   pIacs758_data, pIacs758_size, Iacs758Flag, kEst_data,
                   kEst_size, Rwire_data, Rwire_size, a__64_data,
                   b_VdebugVec_data, &id2Bypass_size, VbusTest_data, &k_ina219_,
                   Rtot_data, Rtot_size, a__38_data, id2Bypass_data, a__39_data,
                   b_VmV_size, a__40_data, tii_size, VmKp184Test_data,
                   &c_loop_ub, a__42_data, a__4_size, a__72_data,
                   &partialTrueCount, outVI, b_tmp_size, VdebugVec_data,
                   VdebugVec_size, &keepMeas, (double *)&b_dv0, errI2C_size);
          loop_ub = Vbat->size[0];
          for (i = 0; i < loop_ub; i++) {
            ImKp184Test2_data[i + Vbat->size[0] * k_t0] = a__64_data[i];
          }

          Nina219[0] = VbatMat->size[0];
          Nina219[1] = VbatMat->size[1];
          loop_ub = VbatMat->size[1];
          for (i = 0; i < loop_ub; i++) {
            b_loop_ub = Nina219[0];
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              VbatMat_data[(i1 + VbatMat->size[0] * i) + VbatMat->size[0] *
                VbatMat->size[1] * k_t0] = Rtot_data[i1 + Nina219[0] * i];
            }
          }

          Nina219[0] = VbatMat0->size[0];
          Nina219[1] = VbatMat0->size[1];
          loop_ub = VbatMat0->size[1];
          for (i = 0; i < loop_ub; i++) {
            b_loop_ub = Nina219[0];
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              VbatMat0_data[(i1 + VbatMat0->size[0] * i) + VbatMat0->size[0] *
                VbatMat0->size[1] * k_t0] = a__38_data[i1 + Nina219[0] * i];
            }
          }

          Nina219[0] = IbatMat->size[0];
          Nina219[1] = IbatMat->size[1];
          loop_ub = IbatMat->size[1];
          for (i = 0; i < loop_ub; i++) {
            b_loop_ub = Nina219[0];
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              Iacs758_cal_data[(i1 + IbatMat->size[0] * i) + IbatMat->size[0] *
                IbatMat->size[1] * k_t0] = a__39_data[i1 + Nina219[0] * i];
            }
          }

          Nina219[0] = tV->size[0];
          Nina219[1] = tV->size[1];
          loop_ub = tV->size[1];
          for (i = 0; i < loop_ub; i++) {
            b_loop_ub = Nina219[0];
            for (i1 = 0; i1 < b_loop_ub; i1++) {
              tV_data[(i1 + tV->size[0] * i) + tV->size[0] * tV->size[1] * k_t0]
                = a__40_data[i1 + Nina219[0] * i];
            }
          }

          loop_ub = tV1->size[0];
          for (i = 0; i < loop_ub; i++) {
            tV1_data[i + tV1->size[0] * k_t0] = VmKp184Test_data[i];
          }

          /*  read V,I */
          for (c_loop_ub = 0; c_loop_ub < VbusTest_size_idx_0; c_loop_ub++) {
            if (kalmanFlag) {
              VmKp184Test_size[0] = 1;
              loop_ub = tV->size[1];
              VmKp184Test_size[1] = tV->size[1];
              for (i = 0; i < loop_ub; i++) {
                c_VmKp184Test_data[i] = tV_data[(c_loop_ub + tV->size[0] * i) +
                  tV->size[0] * tV->size[1] * k_t0];
              }

              loop_ub = IbatMat->size[1];
              for (i = 0; i < loop_ub; i++) {
                b_VmV_data[i] = Iacs758_cal_data[(c_loop_ub + IbatMat->size[0] *
                  i) + IbatMat->size[0] * IbatMat->size[1] * k_t0];
              }

              loop_ub = VbatMat->size[1];
              for (i = 0; i < loop_ub; i++) {
                I_data[i] = VbatMat_data[(c_loop_ub + VbatMat->size[0] * i) +
                  VbatMat->size[0] * VbatMat->size[1] * k_t0];
              }

              b_CalcStructKalman(&prm->klm.b_struct[c_loop_ub], Ta,
                                 c_VmKp184Test_data, VmKp184Test_size,
                                 b_VmV_data, I_data);
            }
          }

          /* kalman - End */
          for (i = 0; i < 8; i++) {
            VbusTest0_data[i] = (prm->seq.ins[i + (state_k << 3)] == 3);
          }

          guard5 = false;
          if (vectorAny(VbusTest0_data)) {
            guard5 = true;
          } else {
            for (i = 0; i < 8; i++) {
              VbusTest0_data[i] = (prm->seq.ins[i + (state_k << 3)] == 4);
            }

            if (vectorAny(VbusTest0_data)) {
              guard5 = true;
            } else {
              for (i = 0; i < 8; i++) {
                VbusTest0_data[i] = (prm->seq.ins[i + (state_k << 3)] == 1);
              }

              if (vectorAny(VbusTest0_data)) {
                ControlKp184((double *)&N0, a__3_size, (double *)&a__4_data,
                             a__4_size, (double *)&d_tmp_data, b_tmp_size,
                             (double *)&a__16_data, id2Bypass_data);
              } else {
                for (i = 0; i < 8; i++) {
                  VbusTest0_data[i] = (prm->seq.ins[i + (state_k << 3)] == 2);
                }

                if (vectorAny(VbusTest0_data)) {
                  /* read I */
                  d_rand();

                  /* read I */
                  d_rand();
                }
              }
            }
          }

          if (guard5) {
            controlJuntekDPH8920(r30.data, r30.size);
            pause(0.1);
            b_controlJuntekDPH8920(r30.data, r30.size);
          }

          i = SelDual_nm1->size[0] * SelDual_nm1->size[1];
          SelDual_nm1->size[0] = 1;
          SelDual_nm1->size[1] = SelDual->size[1];
          emxEnsureCapacity_real_T(SelDual_nm1, i);
          IshuntTest2_data = SelDual_nm1->data;
          loop_ub = SelDual->size[1];
          for (i = 0; i < loop_ub; i++) {
            IshuntTest2_data[i] = SelDual_data[i];
          }

          /*  BattConfigAct = BattConfigAct(:); */
          if (Nina219_tmp_tmp < 1.0) {
            y->size[0] = 1;
            y->size[1] = 0;
          } else {
            i = y->size[0] * y->size[1];
            y->size[0] = 1;
            y->size[1] = (int)(Nina219_tmp_tmp - 1.0) + 1;
            emxEnsureCapacity_real_T(y, i);
            y_data = y->data;
            loop_ub = (int)(Nina219_tmp_tmp - 1.0);
            for (i = 0; i <= loop_ub; i++) {
              y_data[i] = (double)i + 1.0;
            }
          }

          i = b_y->size[0];
          b_y->size[0] = y->size[1];
          emxEnsureCapacity_real_T(b_y, i);
          b_y_data = b_y->data;
          loop_ub = y->size[1];
          for (i = 0; i < loop_ub; i++) {
            b_y_data[i] = y_data[i];
          }

          loop_ub = y->size[1];
          for (i = 0; i < loop_ub; i++) {
            VecIna219_data[i] = b_y_data[i];
          }

          if (b_size - 1 >= 0) {
            c_end = ipos_size - 1;
          }

          for (k_tstState = 0; k_tstState < b_size; k_tstState++) {
            trueCount = 0;
            partialTrueCount = 0;
            for (loop_ub = 0; loop_ub <= c_end; loop_ub++) {
              if (k_tstState + 1 == ipos_data[loop_ub]) {
                trueCount++;
                p_tmp_data[partialTrueCount] = (signed char)loop_ub;
                partialTrueCount++;
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
              emxEnsureCapacity_real_T(y, i);
              y_data = y->data;
              loop_ub = N_bat_tmp_tmp_tmp - 1;
              for (i = 0; i <= loop_ub; i++) {
                y_data[i] = (double)i + 1.0;
              }
            }

            if (trueCount == y->size[1]) {
              i = indVperGrp->size[0];
              indVperGrp->size[0] = y->size[1];
              emxEnsureCapacity_real_T(indVperGrp, i);
              VbusTest2_data = indVperGrp->data;
              loop_ub = y->size[1];
              for (i = 0; i < loop_ub; i++) {
                VbusTest2_data[i] = y_data[i] + (VecIna219_data[p_tmp_data[i]] -
                  1.0) * (double)N_bat_tmp_tmp_tmp;
              }
            } else {
              binary_expand_op_3(indVperGrp, y, VecIna219_data, p_tmp_data,
                                 &trueCount, N_bat_tmp_tmp_tmp);
              VbusTest2_data = indVperGrp->data;
            }

            /*          for k_ina219 = 1:Nina219 */
            /*              if SelDual(k_ina219) ~= prm.brd.spi.bypass&&SelDual(k_ina219) ~= prm.brd.spi.disconnect */
            c_tmp_size[0] = trueCount;
            c_tmp_size[1] = d_loop_ub;
            c_tmp_size[2] = d_loop_ub;
            for (i = 0; i < d_loop_ub; i++) {
              for (i1 = 0; i1 < d_loop_ub; i1++) {
                for (i2 = 0; i2 < trueCount; i2++) {
                  n_tmp_data[(i2 + trueCount * i1) + trueCount * d_loop_ub * i] =
                    BattConfigPerIna_data[(((int)VecIna219_data[p_tmp_data[i2]]
                    + b_loop_ub_tmp * i1) + b_loop_ub_tmp * d_loop_ub * i) - 1];
                }
              }
            }

            d_tmp_size[0] = 1;
            d_tmp_size[1] = trueCount;
            for (i = 0; i < trueCount; i++) {
              outVI[i] = SelDual_data[(int)VecIna219_data[p_tmp_data[i]] - 1];
            }

            e_tmp_size[0] = 1;
            e_tmp_size[1] = trueCount;
            for (i = 0; i < trueCount; i++) {
              o_tmp_data[i] = tLastToggle_data[(int)VecIna219_data[p_tmp_data[i]]
                - 1];
            }

            f_tmp_size[0] = N_bat_tmp_tmp_tmp;
            f_tmp_size[1] = trueCount;
            for (i = 0; i < trueCount; i++) {
              for (i1 = 0; i1 < N_bat_tmp_tmp_tmp; i1++) {
                c_VbusTest_data[i1 + N_bat_tmp_tmp_tmp * i] =
                  BattConfigAct_data[i1 + N_bat_tmp_tmp_tmp * ((int)
                  VecIna219_data[p_tmp_data[i]] - 1)];
              }
            }

            loop_ub = indVperGrp->size[0];
            for (i = 0; i < loop_ub; i++) {
              indVperGrp_data[i] = (int)VbusTest2_data[i];
            }

            partialTrueCount = indVperGrp->size[0];
            loop_ub = indVperGrp->size[0];
            for (i = 0; i < loop_ub; i++) {
              c_VmKp184Test_data[i] = ImKp184Test2_data[(indVperGrp_data[i] +
                Vbat->size[0] * k_t0) - 1];
            }

            i = (int)prm->seq.vth[state_k] - 1;
            showVdiff = d_Esp32StepSwitchToggleCombAll_(outVI, d_tmp_size,
              prm->bat.CutOffChrV[i], prm->bat.CutOffDisV[i], n_tmp_data,
              c_tmp_size, c_VmKp184Test_data, partialTrueCount, trueCount,
              prm->brd.Nbat, prm->brd.spi.disconnect, prm->brd.spi.bypass,
              prm->cnfg.Ttoggle, prm->cnfg.NtoggleDrop, prm->cnfg.minLenIna219,
              prm->seq.pwr, prm->seq.pwr.VthFlag[state_k], tV_data[tV->size[0] *
              tV->size[1] * k_t0], o_tmp_data, e_tmp_size, c_VbusTest_data,
              f_tmp_size, AllBypassPerGroup_data, AllBypassPerGroup_size);
            for (i = 0; i < trueCount; i++) {
              id2Bypass_data[i] = (int)VecIna219_data[p_tmp_data[i]];
            }

            c_loop_ub_tmp = trueCount - 1;
            for (i = 0; i <= c_loop_ub_tmp; i++) {
              changeConfigFlag_data[id2Bypass_data[i] - 1] =
                AllBypassPerGroup_data[i];
            }

            for (i = 0; i < trueCount; i++) {
              id2Bypass_data[i] = (int)VecIna219_data[p_tmp_data[i]] - 1;
            }

            for (i = 0; i < d_loop_ub; i++) {
              for (i1 = 0; i1 < d_loop_ub; i1++) {
                for (i2 = 0; i2 < trueCount; i2++) {
                  BattConfigPerIna_data[(id2Bypass_data[i2] + b_loop_ub_tmp * i1)
                    + b_loop_ub_tmp * d_loop_ub * i] = n_tmp_data[(i2 +
                    trueCount * i1) + trueCount * d_loop_ub * i];
                }
              }
            }

            for (i = 0; i < trueCount; i++) {
              id2Bypass_data[i] = (int)VecIna219_data[p_tmp_data[i]];
            }

            for (i = 0; i <= c_loop_ub_tmp; i++) {
              SelDual_data[id2Bypass_data[i] - 1] = outVI[i];
            }

            for (i = 0; i < trueCount; i++) {
              id2Bypass_data[i] = (int)VecIna219_data[p_tmp_data[i]];
            }

            for (i = 0; i <= c_loop_ub_tmp; i++) {
              switchFlag_data[id2Bypass_data[i] - 1] = showVdiff;
            }

            for (i = 0; i < trueCount; i++) {
              id2Bypass_data[i] = (int)VecIna219_data[p_tmp_data[i]];
            }

            for (i = 0; i <= c_loop_ub_tmp; i++) {
              tLastToggle_data[id2Bypass_data[i] - 1] = o_tmp_data[i];
            }

            Nina219[0] = N_bat_tmp_tmp_tmp;
            Nina219[1] = trueCount;
            for (i = 0; i < trueCount; i++) {
              id2Bypass_data[i] = (int)VecIna219_data[p_tmp_data[i]] - 1;
              loop_ub = Nina219[0];
              for (i1 = 0; i1 < loop_ub; i1++) {
                BattConfigAct_data[i1 + N_bat_tmp_tmp_tmp * id2Bypass_data[i]] =
                  c_VbusTest_data[i1 + Nina219[0] * i];
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
          for (c_loop_ub = 0; c_loop_ub < VbusTest_size_idx_0; c_loop_ub++) {
            if (switchFlag_data[c_loop_ub]) {
              SelDual_data[c_loop_ub] = prm->brd.spi.bypass;

              /* bypass (short) */
              /*  BattConfigPerIna{k_ina219} = prm.brd.spi.bypass; */
              /*  BattConfig{k_ina219}       = prm.brd.spi.bypass; */
            }
          }

          /*  test all groups */
          AllBypassPerGroup_size[0] = 1;
          AllBypassPerGroup_size[1] = b_size;
          for (id2Bypass_size = 0; id2Bypass_size < b_size; id2Bypass_size++) {
            AllBypassPerGroup_data[id2Bypass_size] = false;
            trueCount = 0;
            for (loop_ub = 0; loop_ub <= end_tmp; loop_ub++) {
              if (b_data[id2Bypass_size] == prm->ser.com.grp[tmp_data[loop_ub]])
              {
                trueCount++;
              }
            }

            i = x->size[0] * x->size[1];
            x->size[0] = 1;
            x->size[1] = trueCount;
            emxEnsureCapacity_boolean_T(x, i);
            b_changeConfigFlag_data = x->data;
            partialTrueCount = 0;
            for (loop_ub = 0; loop_ub <= end_tmp; loop_ub++) {
              if (b_data[id2Bypass_size] == prm->ser.com.grp[tmp_data[loop_ub]])
              {
                b_changeConfigFlag_data[partialTrueCount] =
                  (SelDual_data[loop_ub] == prm->brd.spi.bypass);
                partialTrueCount++;
              }
            }

            showVdiff = true;
            b_trueCount = 1;
            exitg2 = false;
            while ((!exitg2) && (b_trueCount <= x->size[1])) {
              if (!b_changeConfigFlag_data[b_trueCount - 1]) {
                showVdiff = false;
                exitg2 = true;
              } else {
                b_trueCount++;
              }
            }

            if (showVdiff) {
              trueCount = 0;
              partialTrueCount = 0;
              for (loop_ub = 0; loop_ub <= end_tmp; loop_ub++) {
                i4 = b_data[id2Bypass_size];
                i = prm->ser.com.grp[tmp_data[loop_ub]];
                if (i4 == i) {
                  trueCount++;
                  q_tmp_data[partialTrueCount] = (signed char)loop_ub;
                  partialTrueCount++;
                }
              }

              SelDual_nm1_size[0] = 1;
              SelDual_nm1_size[1] = trueCount;
              for (i = 0; i < trueCount; i++) {
                SelDual_nm1_data[i] = (IshuntTest2_data[q_tmp_data[i]] == 0.0);
              }

              b_SelDual_nm1_data.data = &SelDual_nm1_data[0];
              b_SelDual_nm1_data.size = &SelDual_nm1_size[0];
              b_SelDual_nm1_data.allocatedSize = 2;
              b_SelDual_nm1_data.numDimensions = 2;
              b_SelDual_nm1_data.canFreeData = false;
              if (b_any(&b_SelDual_nm1_data)) {
                AllBypassPerGroup_data[id2Bypass_size] = true;
                prm->seq.ins[id2Bypass_size + (state_k << 3)] = 0;
                for (loop_ub = 0; loop_ub <= end_tmp; loop_ub++) {
                  if (b_data[id2Bypass_size] == prm->
                      ser.com.grp[tmp_data[loop_ub]]) {
                    SelDual_data[loop_ub] = prm->brd.spi.disconnect;
                  }
                }
              } else {
                trueCount = 0;
                partialTrueCount = 0;
                for (loop_ub = 0; loop_ub <= end_tmp; loop_ub++) {
                  i4 = b_data[id2Bypass_size];
                  i = prm->ser.com.grp[tmp_data[loop_ub]];
                  if (i4 == i) {
                    trueCount++;
                    r_tmp_data[partialTrueCount] = (signed char)loop_ub;
                    partialTrueCount++;
                  }
                }

                SelDual_nm1_size[0] = 1;
                SelDual_nm1_size[1] = trueCount;
                for (i = 0; i < trueCount; i++) {
                  SelDual_nm1_data[i] = (IshuntTest2_data[r_tmp_data[i]] == 1.0);
                }

                c_SelDual_nm1_data.data = &SelDual_nm1_data[0];
                c_SelDual_nm1_data.size = &SelDual_nm1_size[0];
                c_SelDual_nm1_data.allocatedSize = 2;
                c_SelDual_nm1_data.numDimensions = 2;
                c_SelDual_nm1_data.canFreeData = false;
                if (b_any(&c_SelDual_nm1_data)) {
                  for (loop_ub = 0; loop_ub <= end_tmp; loop_ub++) {
                    if (b_data[id2Bypass_size] == prm->
                        ser.com.grp[tmp_data[loop_ub]]) {
                      SelDual_data[loop_ub] = 1.5;
                    }
                  }

                  IchargePhase2t = IchargePhase2_tmp_tmp;
                } else {
                  trueCount = 0;
                  partialTrueCount = 0;
                  for (loop_ub = 0; loop_ub <= end_tmp; loop_ub++) {
                    i4 = b_data[id2Bypass_size];
                    i = prm->ser.com.grp[tmp_data[loop_ub]];
                    if (i4 == i) {
                      trueCount++;
                      s_tmp_data[partialTrueCount] = (signed char)loop_ub;
                      partialTrueCount++;
                    }
                  }

                  SelDual_nm1_size[0] = 1;
                  SelDual_nm1_size[1] = trueCount;
                  for (i = 0; i < trueCount; i++) {
                    SelDual_nm1_data[i] = (IshuntTest2_data[s_tmp_data[i]] ==
                      1.5);
                  }

                  d_SelDual_nm1_data.data = &SelDual_nm1_data[0];
                  d_SelDual_nm1_data.size = &SelDual_nm1_size[0];
                  d_SelDual_nm1_data.allocatedSize = 2;
                  d_SelDual_nm1_data.numDimensions = 2;
                  d_SelDual_nm1_data.canFreeData = false;
                  if (b_any(&d_SelDual_nm1_data)) {
                    IchargePhase2t -= prm->bat.dIphase2;
                    if (IchargePhase2t < prm->bat.minIphase2) {
                      for (loop_ub = 0; loop_ub <= end_tmp; loop_ub++) {
                        if (b_data[id2Bypass_size] == prm->
                            ser.com.grp[tmp_data[loop_ub]]) {
                          SelDual_data[loop_ub] = 1.6;
                        }
                      }

                      IchargePhase2t = IchargePhase2_tmp_tmp;
                    } else {
                      for (loop_ub = 0; loop_ub <= end_tmp; loop_ub++) {
                        if (b_data[id2Bypass_size] == prm->
                            ser.com.grp[tmp_data[loop_ub]]) {
                          SelDual_data[loop_ub] = 1.5;
                        }
                      }
                    }
                  } else {
                    trueCount = 0;
                    partialTrueCount = 0;
                    for (loop_ub = 0; loop_ub <= end_tmp; loop_ub++) {
                      i4 = b_data[id2Bypass_size];
                      i = prm->ser.com.grp[tmp_data[loop_ub]];
                      if (i4 == i) {
                        trueCount++;
                        t_tmp_data[partialTrueCount] = (signed char)loop_ub;
                        partialTrueCount++;
                      }
                    }

                    SelDual_nm1_size[0] = 1;
                    SelDual_nm1_size[1] = trueCount;
                    for (i = 0; i < trueCount; i++) {
                      SelDual_nm1_data[i] = (IshuntTest2_data[t_tmp_data[i]] ==
                        1.6);
                    }

                    e_SelDual_nm1_data.data = &SelDual_nm1_data[0];
                    e_SelDual_nm1_data.size = &SelDual_nm1_size[0];
                    e_SelDual_nm1_data.allocatedSize = 2;
                    e_SelDual_nm1_data.numDimensions = 2;
                    e_SelDual_nm1_data.canFreeData = false;
                    if (b_any(&e_SelDual_nm1_data)) {
                      AllBypassPerGroup_data[id2Bypass_size] = true;
                      prm->seq.ins[id2Bypass_size + (state_k << 3)] = 0;
                      for (loop_ub = 0; loop_ub <= end_tmp; loop_ub++) {
                        if (b_data[id2Bypass_size] == prm->
                            ser.com.grp[tmp_data[loop_ub]]) {
                          SelDual_data[loop_ub] = prm->brd.spi.disconnect;
                        }
                      }
                    }
                  }
                }
              }
            }
          }

          showVdiff = true;
          b_trueCount = 1;
          exitg2 = false;
          while ((!exitg2) && (b_trueCount <= AllBypassPerGroup_size[1])) {
            if (!AllBypassPerGroup_data[b_trueCount - 1]) {
              showVdiff = false;
              exitg2 = true;
            } else {
              b_trueCount++;
            }
          }

          guard5 = false;
          if (showVdiff || ((V_dis_all < prm->ins.prm.jun.minVjuntekInput) &&
                            (prm->seq.VminDisFlag[state_k] != 0.0F))) {
            guard5 = true;
          } else {
            i = x->size[0] * x->size[1];
            x->size[0] = 1;
            x->size[1] = SelDual->size[1];
            emxEnsureCapacity_boolean_T(x, i);
            b_changeConfigFlag_data = x->data;
            loop_ub = SelDual->size[1];
            for (i = 0; i < loop_ub; i++) {
              b_changeConfigFlag_data[i] = (SelDual_data[i] ==
                prm->brd.spi.disconnect);
            }

            showVdiff = true;
            b_trueCount = 1;
            exitg2 = false;
            while ((!exitg2) && (b_trueCount <= x->size[1])) {
              if (!b_changeConfigFlag_data[b_trueCount - 1]) {
                showVdiff = false;
                exitg2 = true;
              } else {
                b_trueCount++;
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
              for (loop_ub = 0; loop_ub <= end_tmp; loop_ub++) {
                if (b_data[id2Bypass_size] == prm->ser.com.grp[tmp_data[loop_ub]])
                {
                  SelDual_data[loop_ub] = prm->seq.chr[id2Bypass_size + (state_k
                    << 3)];
                }
              }

              for (loop_ub = 0; loop_ub <= end_tmp; loop_ub++) {
                if (b_data[id2Bypass_size] == prm->ser.com.grp[tmp_data[loop_ub]])
                {
                  switchFlag_data[loop_ub] = true;
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
            b_trueCount = tV1->size[0];
            loop_ub = tV1->size[0];
            for (i = 0; i < loop_ub; i++) {
              c_VbusTest_data[i] = (tV1_data[i + tV1->size[0] * k_t0] >
                                    prm->run.MaxTime);
            }

            if (ifWhileCond(c_VbusTest_data, b_trueCount)) {
              keepMeas = false;
            }
          }

          if (k_t0 + 1 == N_bitCnfg) {
            t2_data = datetime_datetime();
            datetime_datevec(t2_data, dateVec);
            d_tmp_data = rt_roundd_snf(dateVec[0]);
            if (d_tmp_data < 2.147483648E+9) {
              if (d_tmp_data >= -2.147483648E+9) {
                i = (int)d_tmp_data;
              } else {
                i = MIN_int32_T;
              }
            } else if (d_tmp_data >= 2.147483648E+9) {
              i = MAX_int32_T;
            } else {
              i = 0;
            }

            d_tmp_data = rt_roundd_snf(dateVec[1]);
            if (d_tmp_data < 2.147483648E+9) {
              if (d_tmp_data >= -2.147483648E+9) {
                i1 = (int)d_tmp_data;
              } else {
                i1 = MIN_int32_T;
              }
            } else if (d_tmp_data >= 2.147483648E+9) {
              i1 = MAX_int32_T;
            } else {
              i1 = 0;
            }

            d_tmp_data = rt_roundd_snf(dateVec[2]);
            if (d_tmp_data < 2.147483648E+9) {
              if (d_tmp_data >= -2.147483648E+9) {
                i2 = (int)d_tmp_data;
              } else {
                i2 = MIN_int32_T;
              }
            } else if (d_tmp_data >= 2.147483648E+9) {
              i2 = MAX_int32_T;
            } else {
              i2 = 0;
            }

            d_tmp_data = rt_roundd_snf(dateVec[3]);
            if (d_tmp_data < 2.147483648E+9) {
              if (d_tmp_data >= -2.147483648E+9) {
                i3 = (int)d_tmp_data;
              } else {
                i3 = MIN_int32_T;
              }
            } else if (d_tmp_data >= 2.147483648E+9) {
              i3 = MAX_int32_T;
            } else {
              i3 = 0;
            }

            d_tmp_data = rt_roundd_snf(dateVec[4]);
            if (d_tmp_data < 2.147483648E+9) {
              if (d_tmp_data >= -2.147483648E+9) {
                c_loop_ub_tmp = (int)d_tmp_data;
              } else {
                c_loop_ub_tmp = MIN_int32_T;
              }
            } else if (d_tmp_data >= 2.147483648E+9) {
              c_loop_ub_tmp = MAX_int32_T;
            } else {
              c_loop_ub_tmp = 0;
            }

            d_tmp_data = rt_roundd_snf(dateVec[5]);
            if (d_tmp_data < 2.147483648E+9) {
              if (d_tmp_data >= -2.147483648E+9) {
                id2Bypass_size = (int)d_tmp_data;
              } else {
                id2Bypass_size = MIN_int32_T;
              }
            } else if (d_tmp_data >= 2.147483648E+9) {
              id2Bypass_size = MAX_int32_T;
            } else {
              id2Bypass_size = 0;
            }

            m_sprintf(i, i1, i2, i3, c_loop_ub_tmp, id2Bypass_size, r29);

            /* char(dateStr); */
            i = (int)(Nina219_tmp_tmp * (double)N_bat_tmp_tmp_tmp);
            i1 = c_Vbat->size[0] * c_Vbat->size[1];
            c_Vbat->size[0] = i;
            c_Vbat->size[1] = N_bitCnfg;
            emxEnsureCapacity_int8_T(c_Vbat, i1);
            Vbat_data = c_Vbat->data;
            c_loop_ub_tmp = i * N_bitCnfg;
            for (i1 = 0; i1 < c_loop_ub_tmp; i1++) {
              Vbat_data[i1] = 0;
            }

            i1 = Vbat->size[0] * Vbat->size[1];
            Vbat->size[0] = c_Vbat->size[0];
            Vbat->size[1] = c_Vbat->size[1];
            emxEnsureCapacity_real_T(Vbat, i1);
            ImKp184Test2_data = Vbat->data;
            for (i1 = 0; i1 < c_loop_ub_tmp; i1++) {
              ImKp184Test2_data[i1] = 0.0;
            }

            i1 = IbatMat->size[0] * IbatMat->size[1] * IbatMat->size[2];
            IbatMat->size[0] = (int)Nina219_tmp_tmp;
            IbatMat->size[1] = N_bat_tmp_tmp_tmp;
            IbatMat->size[2] = N_bitCnfg;
            emxEnsureCapacity_real_T(IbatMat, i1);
            Iacs758_cal_data = IbatMat->data;
            end = loop_ub_tmp * N_bitCnfg;
            i1 = VbatMat->size[0] * VbatMat->size[1] * VbatMat->size[2];
            VbatMat->size[0] = (int)Nina219_tmp_tmp;
            VbatMat->size[1] = N_bat_tmp_tmp_tmp;
            VbatMat->size[2] = N_bitCnfg;
            emxEnsureCapacity_real_T(VbatMat, i1);
            VbatMat_data = VbatMat->data;
            i1 = VbatMat0->size[0] * VbatMat0->size[1] * VbatMat0->size[2];
            VbatMat0->size[0] = (int)Nina219_tmp_tmp;
            VbatMat0->size[1] = N_bat_tmp_tmp_tmp;
            VbatMat0->size[2] = N_bitCnfg;
            emxEnsureCapacity_real_T(VbatMat0, i1);
            VbatMat0_data = VbatMat0->data;
            for (i1 = 0; i1 < end; i1++) {
              Iacs758_cal_data[i1] = 0.0;
              VbatMat_data[i1] = 0.0;
              VbatMat0_data[i1] = 0.0;
            }

            i1 = c_Vbat->size[0] * c_Vbat->size[1];
            c_Vbat->size[0] = i;
            c_Vbat->size[1] = N_bitCnfg;
            emxEnsureCapacity_int8_T(c_Vbat, i1);
            Vbat_data = c_Vbat->data;
            for (i = 0; i < c_loop_ub_tmp; i++) {
              Vbat_data[i] = 0;
            }

            i = tV1->size[0] * tV1->size[1];
            tV1->size[0] = c_Vbat->size[0];
            tV1->size[1] = c_Vbat->size[1];
            emxEnsureCapacity_real_T(tV1, i);
            tV1_data = tV1->data;
            for (i = 0; i < c_loop_ub_tmp; i++) {
              tV1_data[i] = 0.0;
            }

            i = tV->size[0] * tV->size[1] * tV->size[2];
            tV->size[0] = (int)Nina219_tmp_tmp;
            tV->size[1] = N_bat_tmp_tmp_tmp;
            tV->size[2] = N_bitCnfg;
            emxEnsureCapacity_real_T(tV, i);
            tV_data = tV->data;
            for (i = 0; i < end; i++) {
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
      for (c_loop_ub = 0; c_loop_ub < i; c_loop_ub++) {
        switch (ProjectFlag) {
         case 2:
          /* esp32 */
          break;

         case 3:
          /* esp32 ser */
          writePortToSpi4RowMask_ser(disConAll);
          break;
        }
      }

      t2_data = datetime_datetime();
      datetime_datevec(t2_data, dateVec);
      d_tmp_data = rt_roundd_snf(dateVec[0]);
      if (d_tmp_data < 2.147483648E+9) {
        if (d_tmp_data >= -2.147483648E+9) {
          i = (int)d_tmp_data;
        } else {
          i = MIN_int32_T;
        }
      } else if (d_tmp_data >= 2.147483648E+9) {
        i = MAX_int32_T;
      } else {
        i = 0;
      }

      d_tmp_data = rt_roundd_snf(dateVec[1]);
      if (d_tmp_data < 2.147483648E+9) {
        if (d_tmp_data >= -2.147483648E+9) {
          i1 = (int)d_tmp_data;
        } else {
          i1 = MIN_int32_T;
        }
      } else if (d_tmp_data >= 2.147483648E+9) {
        i1 = MAX_int32_T;
      } else {
        i1 = 0;
      }

      d_tmp_data = rt_roundd_snf(dateVec[2]);
      if (d_tmp_data < 2.147483648E+9) {
        if (d_tmp_data >= -2.147483648E+9) {
          i2 = (int)d_tmp_data;
        } else {
          i2 = MIN_int32_T;
        }
      } else if (d_tmp_data >= 2.147483648E+9) {
        i2 = MAX_int32_T;
      } else {
        i2 = 0;
      }

      d_tmp_data = rt_roundd_snf(dateVec[3]);
      if (d_tmp_data < 2.147483648E+9) {
        if (d_tmp_data >= -2.147483648E+9) {
          i3 = (int)d_tmp_data;
        } else {
          i3 = MIN_int32_T;
        }
      } else if (d_tmp_data >= 2.147483648E+9) {
        i3 = MAX_int32_T;
      } else {
        i3 = 0;
      }

      d_tmp_data = rt_roundd_snf(dateVec[4]);
      if (d_tmp_data < 2.147483648E+9) {
        if (d_tmp_data >= -2.147483648E+9) {
          c_loop_ub_tmp = (int)d_tmp_data;
        } else {
          c_loop_ub_tmp = MIN_int32_T;
        }
      } else if (d_tmp_data >= 2.147483648E+9) {
        c_loop_ub_tmp = MAX_int32_T;
      } else {
        c_loop_ub_tmp = 0;
      }

      d_tmp_data = rt_roundd_snf(dateVec[5]);
      if (d_tmp_data < 2.147483648E+9) {
        if (d_tmp_data >= -2.147483648E+9) {
          id2Bypass_size = (int)d_tmp_data;
        } else {
          id2Bypass_size = MIN_int32_T;
        }
      } else if (d_tmp_data >= 2.147483648E+9) {
        id2Bypass_size = MAX_int32_T;
      } else {
        id2Bypass_size = 0;
      }

      m_sprintf(i, i1, i2, i3, c_loop_ub_tmp, id2Bypass_size, r18);

      /* char(dateStr); */
      /*  ind0 = strfind((dateStr),':'); */
      /*  dateStr(ind0) = '-'; */
    }
  } else {
    /* CalibrateBaordFlag */
  }

  emxFree_char_T(&r29);
  emxFree_char_T(&d_Vcharge0);
  emxFree_char_T(&r28);
  emxFree_char_T(&c_Vcharge0);
  emxFree_char_T(&r27);
  emxFree_char_T(&r26);
  emxFree_char_T(&f_IchargeAct);
  emxFree_char_T(&r25);
  emxFree_char_T(&e_IchargeAct);
  emxFree_char_T(&d_IchargeAct);
  emxFree_char_T(&c_IchargeAct);
  emxFree_char_T(&r24);
  emxFree_char_T(&r23);
  emxFree_char_T(&r22);
  emxFree_char_T(&r21);
  emxFree_char_T(&r20);
  emxFree_char_T(&b_IchargeAct);
  emxFree_char_T(&IchargeAct);
  emxFree_char_T(&r19);
  emxFree_char_T(&b_Vcharge0);
  emxFree_char_T(&Vcharge0);
  emxFree_char_T(&r18);
  emxFree_char_T(&r17);
  emxFree_char_T(&dv0);
  emxFree_char_T(&r16);
  emxFree_char_T(&r15);
  emxFree_char_T(&r14);
  emxFree_char_T(&meanIbrd0);
  emxFree_char_T(&r13);
  emxFree_char_T(&r12);
  emxFree_char_T(&b_IshuntTest2);
  emxFree_char_T(&b_k_Ttest);
  emxFree_char_T(&d_prm);
  emxFree_char_T(&b_VbusTest2);
  emxFree_char_T(&b_vSumMax);
  emxFree_char_T(&k_Ttest);
  emxFree_char_T(&r11);
  emxFree_char_T(&r10);
  emxFree_char_T(&c_prm);
  emxFree_char_T(&r9);
  emxFree_char_T(&r8);
  emxFree_char_T(&b_prm);
  emxFree_char_T(&r7);
  emxFree_char_T(&vSumMax);
  emxFree_char_T(&b_VmKp184Test_data);
  emxFree_char_T(&b_k_bat);
  emxFree_char_T(&r6);
  emxFree_char_T(&b_VbusTest_data);
  emxFree_char_T(&k_bat);
  emxFree_char_T(&r5);
  emxFree_char_T(&r4);
  emxFree_uint8_T(&r3);
  emxFree_boolean_T(&b_changeConfigFlag);
  emxFree_uint8_T(&r2);
  emxFree_real_T(&b_y);
  emxFree_real_T(&r1);
  emxFree_boolean_T(&x);
  emxFree_real_T(&y);
  emxFree_int8_T(&c_Vbat);
  emxFree_int8_T(&b_Vbat);
  emxFree_real_T(&indVperGrp);
  emxFree_real_T(&r);
  emxFree_real_T(&Vbus_cal_bat_00);
  emxFree_real_T(&Ishunt_cal_bat_00);
  emxFree_real_T(&SelDual_nm1);
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
  emxFree_real_T(&VbitInsMeas);
  emxFree_real_T(&meanIbrd);
  emxFree_real_T(&Ibrd);
  emxFree_real_T(&Vbrd);
  emxFree_real_T(&Iacs758_cal);
  emxFree_real_T(&ImKp184Test2);
  emxFree_real_T(&IshuntTest2);
  emxFree_real_T(&VbusTest2);
}

/*
 * File trailer for Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229.c
 *
 * [EOF]
 */
