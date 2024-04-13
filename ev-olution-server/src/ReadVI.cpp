/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: ReadVI.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 10-Apr-2024 21:46:28
 */

/* Include Files */
#include "ReadVI.h"
#include "randn.h"
#include "readI2cVIfastTicTocV5_Rratio_ser.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Declarations */
static void binary_expand_op_5(double in1_data[], int in1_size[2],
                               const float in2_data[], const int in2_size[2],
                               const double in3_data[], const int in3_size[2]);

static int binary_expand_op_6(float in1_data[], const double in2_data[],
                              const int in2_size[2], const float in3_data[],
                              const int in3_size[2], const double in4_data[],
                              const int in4_size[2]);

static void binary_expand_op_7(double in1_data[], int in1_size[2],
                               const float in2_data[], const int in2_size[2],
                               const double in3_data[], const int in3_size[2]);

static int binary_expand_op_8(float in1_data[], const double in2_data[],
                              const int in2_size[2], const float in3_data[],
                              const int in3_size[2], const double in4_data[],
                              const int in4_size[2]);

/* Function Definitions */
/*
 * Arguments    : double in1_data[]
 *                int in1_size[2]
 *                const float in2_data[]
 *                const int in2_size[2]
 *                const double in3_data[]
 *                const int in3_size[2]
 * Return Type  : void
 */
static void binary_expand_op_5(double in1_data[], int in1_size[2],
                               const float in2_data[], const int in2_size[2],
                               const double in3_data[], const int in3_size[2])
{
  double b_in1_data[32];
  int aux_0_1;
  int aux_1_1;
  int aux_2_1;
  int b_loop_ub;
  int i;
  int i1;
  int in1_size_idx_0;
  int loop_ub;
  int stride_0_1;
  int stride_1_1;
  int stride_2_1;
  in1_size_idx_0 = in1_size[0];
  if (in3_size[1] == 1) {
    i = in2_size[1];
  } else {
    i = in3_size[1];
  }
  if (i == 1) {
    loop_ub = in1_size[1];
  } else {
    loop_ub = i;
  }
  stride_0_1 = (in1_size[1] != 1);
  stride_1_1 = (in2_size[1] != 1);
  stride_2_1 = (in3_size[1] != 1);
  aux_0_1 = 0;
  aux_1_1 = 0;
  aux_2_1 = 0;
  for (i = 0; i < loop_ub; i++) {
    b_loop_ub = in1_size[0];
    for (i1 = 0; i1 < b_loop_ub; i1++) {
      b_in1_data[i1 + in1_size_idx_0 * i] =
          in1_data[i1 + in1_size[0] * aux_0_1] +
          in2_data[i1 + aux_1_1] * in3_data[i1 + in3_size[0] * aux_2_1];
    }
    aux_2_1 += stride_2_1;
    aux_1_1 += stride_1_1;
    aux_0_1 += stride_0_1;
  }
  in1_size[1] = loop_ub;
  for (i = 0; i < loop_ub; i++) {
    for (i1 = 0; i1 < in1_size_idx_0; i1++) {
      in1_data[i1 + in1_size[0] * i] = b_in1_data[i1 + in1_size_idx_0 * i];
    }
  }
}

/*
 * Arguments    : float in1_data[]
 *                const double in2_data[]
 *                const int in2_size[2]
 *                const float in3_data[]
 *                const int in3_size[2]
 *                const double in4_data[]
 *                const int in4_size[2]
 * Return Type  : int
 */
static int binary_expand_op_6(float in1_data[], const double in2_data[],
                              const int in2_size[2], const float in3_data[],
                              const int in3_size[2], const double in4_data[],
                              const int in4_size[2])
{
  int i;
  int in1_size;
  int in2_idx_0;
  int in4_idx_0;
  int stride_0_0;
  int stride_1_0;
  in2_idx_0 = in2_size[0];
  in4_idx_0 = in4_size[0];
  if (in4_idx_0 == 1) {
    i = in3_size[1];
  } else {
    i = in4_idx_0;
  }
  if (i == 1) {
    in1_size = in2_idx_0;
  } else {
    in1_size = i;
  }
  stride_0_0 = (in2_idx_0 != 1);
  stride_1_0 = (in3_size[1] != 1);
  in2_idx_0 = (in4_idx_0 != 1);
  for (i = 0; i < in1_size; i++) {
    in1_data[i] = (float)in2_data[i * stride_0_0] +
                  in3_data[i * stride_1_0] * (float)in4_data[i * in2_idx_0];
  }
  return in1_size;
}

/*
 * Arguments    : double in1_data[]
 *                int in1_size[2]
 *                const float in2_data[]
 *                const int in2_size[2]
 *                const double in3_data[]
 *                const int in3_size[2]
 * Return Type  : void
 */
static void binary_expand_op_7(double in1_data[], int in1_size[2],
                               const float in2_data[], const int in2_size[2],
                               const double in3_data[], const int in3_size[2])
{
  double b_in1_data[32];
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
  if (in3_size[0] == 1) {
    i = in2_size[0];
  } else {
    i = in3_size[0];
  }
  if (i == 1) {
    loop_ub = in1_size[0];
  } else {
    loop_ub = i;
  }
  if (in3_size[1] == 1) {
    i = in2_size[1];
  } else {
    i = in3_size[1];
  }
  if (i == 1) {
    b_loop_ub = in1_size[1];
  } else {
    b_loop_ub = i;
  }
  stride_0_0 = (in1_size[0] != 1);
  stride_0_1 = (in1_size[1] != 1);
  stride_1_0 = (in2_size[0] != 1);
  stride_1_1 = (in2_size[1] != 1);
  stride_2_0 = (in3_size[0] != 1);
  stride_2_1 = (in3_size[1] != 1);
  aux_0_1 = 0;
  aux_1_1 = 0;
  aux_2_1 = 0;
  for (i = 0; i < b_loop_ub; i++) {
    for (i1 = 0; i1 < loop_ub; i1++) {
      b_in1_data[i1 + loop_ub * i] =
          in1_data[i1 * stride_0_0 + in1_size[0] * aux_0_1] +
          in2_data[i1 * stride_1_0 + in2_size[0] * aux_1_1] *
              in3_data[i1 * stride_2_0 + in3_size[0] * aux_2_1];
    }
    aux_2_1 += stride_2_1;
    aux_1_1 += stride_1_1;
    aux_0_1 += stride_0_1;
  }
  in1_size[0] = loop_ub;
  in1_size[1] = b_loop_ub;
  for (i = 0; i < b_loop_ub; i++) {
    for (i1 = 0; i1 < loop_ub; i1++) {
      in1_data[i1 + in1_size[0] * i] = b_in1_data[i1 + loop_ub * i];
    }
  }
}

/*
 * Arguments    : float in1_data[]
 *                const double in2_data[]
 *                const int in2_size[2]
 *                const float in3_data[]
 *                const int in3_size[2]
 *                const double in4_data[]
 *                const int in4_size[2]
 * Return Type  : int
 */
static int binary_expand_op_8(float in1_data[], const double in2_data[],
                              const int in2_size[2], const float in3_data[],
                              const int in3_size[2], const double in4_data[],
                              const int in4_size[2])
{
  int i;
  int in1_size;
  int in2;
  int in3;
  int in4;
  int stride_0_0;
  in2 = in2_size[0] * in2_size[1];
  in3 = in3_size[0] * in3_size[1];
  in4 = in4_size[0] * in4_size[1];
  if (in4 == 1) {
    i = in3;
  } else {
    i = in4;
  }
  if (i == 1) {
    in1_size = in2;
  } else {
    in1_size = i;
  }
  stride_0_0 = (in2 != 1);
  in3 = (in3 != 1);
  in2 = (in4 != 1);
  for (i = 0; i < in1_size; i++) {
    in1_data[i] = (float)in2_data[i * stride_0_0] +
                  in3_data[i * in3] * (float)in4_data[i * in2];
  }
  return in1_size;
}

/*
 * read V,I
 *
 * Arguments    : signed char ProjectFlag
 *                const signed char VIpacId[16]
 *                const unsigned char Pac2Vid0All_data[]
 *                const int Pac2Vid0All_size[3]
 *                short N_bat1
 *                double N_bat
 *                const float pIacs758_data[]
 *                signed char Iacs758Flag
 *                const double pIshunt_data[]
 *                const int pIshunt_size[3]
 *                const float Rwire_data[]
 *                const int Rwire_size[2]
 *                float Vbat_data[]
 *                double Vbat0_data[]
 *                int *Vbat0_size
 *                double Ibat_data[]
 *                int *Ibat_size
 *                double VbatMat_data[]
 *                int VbatMat_size[2]
 *                double VbatMat0_data[]
 *                int VbatMat0_size[2]
 *                double Ishunt_data[]
 *                int Ishunt_size[2]
 *                double tV_data[]
 *                int tV_size[2]
 *                double tV1_data[]
 *                int *tV1_size
 *                double tI_data[]
 *                int tI_size[2]
 *                double tI1_data[]
 *                int *tI1_size
 *                double Iacs758_data[]
 *                int Iacs758_size[2]
 *                double Vdebug0_data[]
 *                int Vdebug0_size[3]
 *                boolean_T *keepMeas
 *                double errI2C_data[]
 *                int errI2C_size[2]
 * Return Type  : int
 */
int ReadVI(signed char ProjectFlag, const signed char VIpacId[16],
           const unsigned char Pac2Vid0All_data[],
           const int Pac2Vid0All_size[3], short N_bat1, double N_bat,
           const float pIacs758_data[], signed char Iacs758Flag,
           const double pIshunt_data[], const int pIshunt_size[3],
           const float Rwire_data[], const int Rwire_size[2], float Vbat_data[],
           double Vbat0_data[], int *Vbat0_size, double Ibat_data[],
           int *Ibat_size, double VbatMat_data[], int VbatMat_size[2],
           double VbatMat0_data[], int VbatMat0_size[2], double Ishunt_data[],
           int Ishunt_size[2], double tV_data[], int tV_size[2],
           double tV1_data[], int *tV1_size, double tI_data[], int tI_size[2],
           double tI1_data[], int *tI1_size, double Iacs758_data[],
           int Iacs758_size[2], double Vdebug0_data[], int Vdebug0_size[3],
           boolean_T *keepMeas, double errI2C_data[], int errI2C_size[2])
{
  double b_VbatMat_data[32];
  double I_data[16];
  double V_data[16];
  double tii_data[16];
  double tvv_data[16];
  double b;
  int I_size[2];
  int V_size[2];
  int b_Vbat0_size[2];
  int b_VbatMat_size[2];
  int tV1mat_size[2];
  int tii_size[2];
  int tvv_size[2];
  int Vbat_size;
  int end;
  int i;
  int loop_ub;
  int loop_ub_tmp;
  int trueCount;
  signed char b_tmp_data[16];
  signed char c_tmp_data[16];
  boolean_T tmp_data[16];
  boolean_T b_b;
  *keepMeas = true;
  /* int16(32); */
  /*  Nmax.NgrpMax = int16(8); */
  /*  Nmax.NseqMax   = int16(32); */
  /*  Nmax.NstateMax = int16(16); */
  /*  Nmax.NitstMax  = int16(32); */
  /*  Nmax.NstrRowMax= int16(32); */
  /*  NstrColMax= 256; */
  /*  Nmax.NtimeMax         = int16(2^10); */
  /*  Nmax.NkalmanBatState  = int16(31); */
  /*  Nmax.NkalmanBatParams = int16(38); */
  /*  Vbat   = zeros(Nina219*N_bat); */
  Vdebug0_size[0] = 1;
  loop_ub_tmp = (int)N_bat;
  Vdebug0_size[1] = (int)N_bat;
  Vdebug0_size[2] = 2;
  loop_ub = (int)N_bat << 1;
  if (loop_ub - 1 >= 0) {
    memset(&Vdebug0_data[0], 0, (unsigned int)loop_ub * sizeof(double));
  }
  VbatMat_size[0] = 1;
  VbatMat_size[1] = (int)N_bat;
  /*  Ibat   = zeros(Nina219*N_bat); */
  /*  tV1    = zeros(Nina219*N_bat); */
  /*  tI1    = zeros(Nina219*N_bat); */
  tV_size[0] = 1;
  tV_size[1] = (int)N_bat;
  tI_size[0] = 1;
  tI_size[1] = (int)N_bat;
  Ishunt_size[0] = 1;
  Ishunt_size[1] = (int)N_bat;
  I_size[0] = (int)N_bat;
  I_size[1] = 1;
  if (loop_ub_tmp - 1 >= 0) {
    memset(&VbatMat_data[0], 0, (unsigned int)loop_ub_tmp * sizeof(double));
    memset(&tV_data[0], 0, (unsigned int)loop_ub_tmp * sizeof(double));
    memset(&tI_data[0], 0, (unsigned int)loop_ub_tmp * sizeof(double));
    memset(&Ishunt_data[0], 0, (unsigned int)loop_ub_tmp * sizeof(double));
    memset(&I_data[0], 0, (unsigned int)loop_ub_tmp * sizeof(double));
    memset(&V_data[0], 0, (unsigned int)loop_ub_tmp * sizeof(double));
    memset(&tvv_data[0], 0, (unsigned int)loop_ub_tmp * sizeof(double));
    memset(&tii_data[0], 0, (unsigned int)loop_ub_tmp * sizeof(double));
  }
  Iacs758_size[0] = 1;
  Iacs758_size[1] = 1;
  Iacs758_data[0] = 0.0;
  errI2C_size[0] = 0;
  errI2C_size[1] = 0;
  switch (ProjectFlag) {
  case 2:
    /* ESP32 */
    /*                  toc0 = toc; */
    I_size[0] = 16;
    I_size[1] = 1;
    memset(&V_data[0], 0, 16U * sizeof(double));
    memset(&I_data[0], 0, 16U * sizeof(double));
    memset(&tvv_data[0], 0, 16U * sizeof(double));
    memset(&tii_data[0], 0, 16U * sizeof(double));
    errI2C_size[0] = 1;
    errI2C_size[1] = 1;
    errI2C_data[0] = -1.0;
    if (N_bat < 1.0) {
      loop_ub = 0;
    } else {
      loop_ub = (int)N_bat;
    }
    for (i = 0; i < 2; i++) {
      for (end = 0; end < loop_ub; end++) {
        Vdebug0_data[end + (int)N_bat * i] = 0.0;
      }
    }
    break;
  case 3:
    /* ESP32 ser */
    /*                  toc0 = toc; */
    d_readI2cVIfastTicTocV5_Rratio_(VIpacId, N_bat1, V_data, V_size, I_data,
                                    I_size, tvv_data, tvv_size, tii_data,
                                    tii_size, errI2C_data, errI2C_size,
                                    b_VbatMat_data, b_VbatMat_size);
    if (N_bat < 1.0) {
      loop_ub = 0;
    } else {
      loop_ub = (int)N_bat;
    }
    for (i = 0; i < 2; i++) {
      for (end = 0; end < loop_ub; end++) {
        Vdebug0_data[end + (int)N_bat * i] = b_VbatMat_data[end + loop_ub * i];
      }
    }
    break;
  }
  if ((errI2C_size[0] != 0) && (errI2C_size[1] != 0)) {
    *keepMeas = false;
  } else {
    if (Iacs758Flag == 1) {
      Iacs758_data[0] = pIacs758_data[0] * 0.0F + pIacs758_data[1];
      loop_ub = I_size[0] * I_size[1];
      if (loop_ub - 1 >= 0) {
        memset(&I_data[0], 0, (unsigned int)loop_ub * sizeof(double));
      }
      /* Pac2Vid0(:,1); */
      loop_ub = Pac2Vid0All_size[1];
      for (i = 0; i < loop_ub; i++) {
        tmp_data[i] = (Pac2Vid0All_data[i] > 0);
      }
      end = Pac2Vid0All_size[1] - 1;
      trueCount = 0;
      loop_ub = 0;
      for (i = 0; i <= end; i++) {
        if (tmp_data[i]) {
          trueCount++;
          c_tmp_data[loop_ub] = (signed char)i;
          loop_ub++;
        }
      }
      for (i = 0; i < trueCount; i++) {
        b_tmp_data[i] = (signed char)Pac2Vid0All_data[c_tmp_data[i]];
      }
      loop_ub = trueCount - 1;
      for (i = 0; i <= loop_ub; i++) {
        I_data[b_tmp_data[i] - 1] = Iacs758_data[0];
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
      Iacs758_data[0] = pIacs758_data[0] * (float)b + pIacs758_data[1];
      loop_ub = I_size[0] * I_size[1];
      if (loop_ub - 1 >= 0) {
        memset(&I_data[0], 0, (unsigned int)loop_ub * sizeof(double));
      }
      /* Pac2Vid0(:,1); */
      loop_ub = Pac2Vid0All_size[1];
      for (i = 0; i < loop_ub; i++) {
        tmp_data[i] = (Pac2Vid0All_data[i] > 0);
      }
      end = Pac2Vid0All_size[1] - 1;
      trueCount = 0;
      loop_ub = 0;
      for (i = 0; i <= end; i++) {
        if (tmp_data[i]) {
          trueCount++;
          c_tmp_data[loop_ub] = (signed char)i;
          loop_ub++;
        }
      }
      for (i = 0; i < trueCount; i++) {
        b_tmp_data[i] = (signed char)Pac2Vid0All_data[c_tmp_data[i]];
      }
      loop_ub = trueCount - 1;
      for (i = 0; i <= loop_ub; i++) {
        I_data[b_tmp_data[i] - 1] = Iacs758_data[0];
      }
    }
    /*  + squeeze(Voffset(k_ina219,:,:))'; */
    /* (toc0+toc1)/2; */
    for (end = 0; end < loop_ub_tmp; end++) {
      VbatMat_data[end] = V_data[end];
      Ishunt_data[end] =
          I_data[end] * pIshunt_data[pIshunt_size[0] * end] +
          pIshunt_data[(end + pIshunt_size[1]) * pIshunt_size[0]];
      tV_data[end] = tvv_data[end];
      tI_data[end] = tii_data[end];
    }
    /* tV(k_ina219,:,k_t); */
    /*  switch ProjectFlag */
    /*      case 2%ESP32 */
    /*  end */
  }
  loop_ub = (int)N_bat;
  b_Vbat0_size[0] = (int)N_bat;
  b_Vbat0_size[1] = 1;
  *Vbat0_size = (int)N_bat;
  for (i = 0; i < loop_ub; i++) {
    b = VbatMat_data[i];
    I_data[i] = b;
    Vbat0_data[i] = b;
  }
  loop_ub = (int)N_bat;
  *tV1_size = (int)N_bat;
  if (loop_ub - 1 >= 0) {
    memcpy(&tV1_data[0], &tV_data[0], (unsigned int)loop_ub * sizeof(double));
  }
  loop_ub = (int)N_bat;
  *tI1_size = (int)N_bat;
  if (loop_ub - 1 >= 0) {
    memcpy(&tI1_data[0], &tI_data[0], (unsigned int)loop_ub * sizeof(double));
  }
  loop_ub = (int)N_bat;
  tV1mat_size[0] = (int)N_bat;
  tV1mat_size[1] = 1;
  *Ibat_size = (int)N_bat;
  for (i = 0; i < loop_ub; i++) {
    b = Ishunt_data[i];
    V_data[i] = b;
    Ibat_data[i] = b;
  }
  if (Rwire_size[1] == 1) {
    loop_ub_tmp = (int)N_bat;
  } else {
    loop_ub_tmp = Rwire_size[1];
  }
  b_b = (((int)N_bat == Rwire_size[1]) && ((int)N_bat == loop_ub_tmp));
  if (b_b) {
    Vbat_size = (int)N_bat;
    loop_ub = (int)N_bat;
    for (i = 0; i < loop_ub; i++) {
      Vbat_data[i] = (float)I_data[i] + Rwire_data[i] * (float)V_data[i];
    }
  } else {
    Vbat_size = binary_expand_op_6(Vbat_data, I_data, b_Vbat0_size, Rwire_data,
                                   Rwire_size, V_data, tV1mat_size);
  }
  VbatMat0_size[0] = 1;
  VbatMat0_size[1] = (int)N_bat;
  loop_ub = (int)N_bat;
  if (loop_ub - 1 >= 0) {
    memcpy(&VbatMat0_data[0], &VbatMat_data[0],
           (unsigned int)loop_ub * sizeof(double));
  }
  if (b_b) {
    loop_ub = (int)N_bat;
    for (i = 0; i < loop_ub; i++) {
      VbatMat_data[i] += Rwire_data[i] * Ishunt_data[i];
    }
  } else {
    binary_expand_op_5(VbatMat_data, VbatMat_size, Rwire_data, Rwire_size,
                       Ishunt_data, Ishunt_size);
  }
  return Vbat_size;
}

/*
 * read V,I
 *
 * Arguments    : double Nina219
 *                signed char ProjectFlag
 *                const signed char VIpacId[32]
 *                const unsigned char Pac2Vid0All_data[]
 *                const int Pac2Vid0All_size[3]
 *                short N_bat1
 *                double N_bat
 *                const float pIacs758_data[]
 *                const int pIacs758_size[3]
 *                signed char Iacs758Flag
 *                const double pIshunt_data[]
 *                const int pIshunt_size[3]
 *                const float Rwire_data[]
 *                const int Rwire_size[2]
 *                float Vbat_data[]
 *                double Vbat0_data[]
 *                int *Vbat0_size
 *                double Ibat_data[]
 *                int *Ibat_size
 *                double VbatMat_data[]
 *                int VbatMat_size[2]
 *                double VbatMat0_data[]
 *                int VbatMat0_size[2]
 *                double Ishunt_data[]
 *                int Ishunt_size[2]
 *                double tV_data[]
 *                int tV_size[2]
 *                double tV1_data[]
 *                int *tV1_size
 *                double tI_data[]
 *                int tI_size[2]
 *                double tI1_data[]
 *                int *tI1_size
 *                double Iacs758_data[]
 *                int Iacs758_size[2]
 *                double Vdebug0_data[]
 *                int Vdebug0_size[3]
 *                boolean_T *keepMeas
 *                double errI2C_data[]
 *                int errI2C_size[2]
 * Return Type  : int
 */
int b_ReadVI(double Nina219, signed char ProjectFlag,
             const signed char VIpacId[32],
             const unsigned char Pac2Vid0All_data[],
             const int Pac2Vid0All_size[3], short N_bat1, double N_bat,
             const float pIacs758_data[], const int pIacs758_size[3],
             signed char Iacs758Flag, const double pIshunt_data[],
             const int pIshunt_size[3], const float Rwire_data[],
             const int Rwire_size[2], float Vbat_data[], double Vbat0_data[],
             int *Vbat0_size, double Ibat_data[], int *Ibat_size,
             double VbatMat_data[], int VbatMat_size[2], double VbatMat0_data[],
             int VbatMat0_size[2], double Ishunt_data[], int Ishunt_size[2],
             double tV_data[], int tV_size[2], double tV1_data[], int *tV1_size,
             double tI_data[], int tI_size[2], double tI1_data[], int *tI1_size,
             double Iacs758_data[], int Iacs758_size[2], double Vdebug0_data[],
             int Vdebug0_size[3], boolean_T *keepMeas, double errI2C_data[],
             int errI2C_size[2])
{
  double b_Vbat0_data[32];
  double tV1mat_data[32];
  double I_data[16];
  double V_data[16];
  double tii_data[16];
  double tvv_data[16];
  double b;
  float Rwire0_data[32];
  int I_size[2];
  int Rwire0_size[2];
  int V_size[2];
  int b_Vbat0_size[2];
  int tV1mat_size[2];
  int tii_size[2];
  int tvv_size[2];
  int I_idx_0;
  int I_idx_1;
  int Vbat_size;
  int b_loop_ub_tmp;
  int i;
  int k_ina219;
  int loop_ub;
  int loop_ub_tmp;
  signed char b_VIpacId[16];
  signed char b_tmp_data[16];
  signed char tmp_data[16];
  *keepMeas = true;
  /* int16(32); */
  /*  Nmax.NgrpMax = int16(8); */
  /*  Nmax.NseqMax   = int16(32); */
  /*  Nmax.NstateMax = int16(16); */
  /*  Nmax.NitstMax  = int16(32); */
  /*  Nmax.NstrRowMax= int16(32); */
  /*  NstrColMax= 256; */
  /*  Nmax.NtimeMax         = int16(2^10); */
  /*  Nmax.NkalmanBatState  = int16(31); */
  /*  Nmax.NkalmanBatParams = int16(38); */
  /*  Vbat   = zeros(Nina219*N_bat); */
  loop_ub_tmp = (int)Nina219;
  Vdebug0_size[0] = (int)Nina219;
  b_loop_ub_tmp = (int)N_bat;
  Vdebug0_size[1] = (int)N_bat;
  Vdebug0_size[2] = 2;
  *Vbat0_size = (int)Nina219 * (int)N_bat;
  loop_ub = *Vbat0_size << 1;
  if (loop_ub - 1 >= 0) {
    memset(&Vdebug0_data[0], 0, (unsigned int)loop_ub * sizeof(double));
  }
  VbatMat_size[0] = (int)Nina219;
  VbatMat_size[1] = (int)N_bat;
  /*  Ibat   = zeros(Nina219*N_bat); */
  /*  tV1    = zeros(Nina219*N_bat); */
  /*  tI1    = zeros(Nina219*N_bat); */
  tV_size[0] = (int)Nina219;
  tV_size[1] = (int)N_bat;
  tI_size[0] = (int)Nina219;
  tI_size[1] = (int)N_bat;
  Ishunt_size[0] = (int)Nina219;
  Ishunt_size[1] = (int)N_bat;
  if (*Vbat0_size - 1 >= 0) {
    memset(&VbatMat_data[0], 0, (unsigned int)*Vbat0_size * sizeof(double));
    memset(&tV_data[0], 0, (unsigned int)*Vbat0_size * sizeof(double));
    memset(&tI_data[0], 0, (unsigned int)*Vbat0_size * sizeof(double));
    memset(&Ishunt_data[0], 0, (unsigned int)*Vbat0_size * sizeof(double));
  }
  I_size[0] = (int)N_bat;
  I_size[1] = 1;
  if (b_loop_ub_tmp - 1 >= 0) {
    memset(&I_data[0], 0, (unsigned int)b_loop_ub_tmp * sizeof(double));
    memset(&V_data[0], 0, (unsigned int)b_loop_ub_tmp * sizeof(double));
    memset(&tvv_data[0], 0, (unsigned int)b_loop_ub_tmp * sizeof(double));
    memset(&tii_data[0], 0, (unsigned int)b_loop_ub_tmp * sizeof(double));
  }
  Iacs758_size[0] = (int)Nina219;
  Iacs758_size[1] = 1;
  if (loop_ub_tmp - 1 >= 0) {
    memset(&Iacs758_data[0], 0, (unsigned int)loop_ub_tmp * sizeof(double));
  }
  errI2C_size[0] = 0;
  errI2C_size[1] = 0;
  for (k_ina219 = 0; k_ina219 < loop_ub_tmp; k_ina219++) {
    switch (ProjectFlag) {
    case 2:
      /* ESP32 */
      /*                  toc0 = toc; */
      I_size[0] = 16;
      I_size[1] = 1;
      memset(&V_data[0], 0, 16U * sizeof(double));
      memset(&I_data[0], 0, 16U * sizeof(double));
      memset(&tvv_data[0], 0, 16U * sizeof(double));
      memset(&tii_data[0], 0, 16U * sizeof(double));
      errI2C_size[0] = 1;
      errI2C_size[1] = 1;
      errI2C_data[0] = -1.0;
      if (N_bat < 1.0) {
        loop_ub = 0;
      } else {
        loop_ub = (int)N_bat;
      }
      for (i = 0; i < 2; i++) {
        for (I_idx_1 = 0; I_idx_1 < loop_ub; I_idx_1++) {
          Vdebug0_data[(k_ina219 + (int)Nina219 * I_idx_1) +
                       (int)Nina219 * (int)N_bat * i] = 0.0;
        }
      }
      break;
    case 3:
      /* ESP32 ser */
      /*                  toc0 = toc; */
      for (i = 0; i < 16; i++) {
        b_VIpacId[i] = VIpacId[k_ina219 + (i << 1)];
      }
      d_readI2cVIfastTicTocV5_Rratio_(b_VIpacId, N_bat1, V_data, V_size, I_data,
                                      I_size, tvv_data, tvv_size, tii_data,
                                      tii_size, errI2C_data, errI2C_size,
                                      tV1mat_data, tV1mat_size);
      if (N_bat < 1.0) {
        loop_ub = 0;
      } else {
        loop_ub = (int)N_bat;
      }
      for (i = 0; i < 2; i++) {
        for (I_idx_1 = 0; I_idx_1 < loop_ub; I_idx_1++) {
          Vdebug0_data[(k_ina219 + (int)Nina219 * I_idx_1) +
                       (int)Nina219 * (int)N_bat * i] =
              tV1mat_data[I_idx_1 + loop_ub * i];
        }
      }
      break;
    }
    if ((errI2C_size[0] != 0) && (errI2C_size[1] != 0)) {
      *keepMeas = false;
    } else {
      if (Iacs758Flag == 1) {
        Iacs758_data[k_ina219] = pIacs758_data[k_ina219] * 0.0F +
                                 pIacs758_data[k_ina219 + pIacs758_size[0]];
        I_idx_0 = I_size[0];
        I_idx_1 = I_size[1];
        loop_ub = I_idx_0 * I_idx_1;
        if (loop_ub - 1 >= 0) {
          memset(&I_data[0], 0, (unsigned int)loop_ub * sizeof(double));
        }
        /* Pac2Vid0(:,1); */
        loop_ub = Pac2Vid0All_size[1];
        I_idx_1 = 0;
        I_idx_0 = 0;
        for (i = 0; i < loop_ub; i++) {
          if (Pac2Vid0All_data[k_ina219 + Pac2Vid0All_size[0] * i] > 0) {
            I_idx_1++;
            tmp_data[I_idx_0] = (signed char)i;
            I_idx_0++;
          }
        }
        for (i = 0; i < I_idx_1; i++) {
          b_VIpacId[i] = (signed char)
              Pac2Vid0All_data[k_ina219 + Pac2Vid0All_size[0] * tmp_data[i]];
        }
        loop_ub = I_idx_1 - 1;
        for (i = 0; i <= loop_ub; i++) {
          I_data[b_VIpacId[i] - 1] = Iacs758_data[k_ina219];
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
                                 pIacs758_data[k_ina219 + pIacs758_size[0]];
        I_idx_0 = I_size[0];
        I_idx_1 = I_size[1];
        loop_ub = I_idx_0 * I_idx_1;
        if (loop_ub - 1 >= 0) {
          memset(&I_data[0], 0, (unsigned int)loop_ub * sizeof(double));
        }
        /* Pac2Vid0(:,1); */
        loop_ub = Pac2Vid0All_size[1];
        I_idx_1 = 0;
        I_idx_0 = 0;
        for (i = 0; i < loop_ub; i++) {
          if (Pac2Vid0All_data[k_ina219 + Pac2Vid0All_size[0] * i] > 0) {
            I_idx_1++;
            b_tmp_data[I_idx_0] = (signed char)i;
            I_idx_0++;
          }
        }
        for (i = 0; i < I_idx_1; i++) {
          b_VIpacId[i] = (signed char)
              Pac2Vid0All_data[k_ina219 + Pac2Vid0All_size[0] * b_tmp_data[i]];
        }
        loop_ub = I_idx_1 - 1;
        for (i = 0; i <= loop_ub; i++) {
          I_data[b_VIpacId[i] - 1] = Iacs758_data[k_ina219];
        }
      }
      loop_ub = (int)N_bat;
      for (i = 0; i < loop_ub; i++) {
        VbatMat_data[k_ina219 + (int)Nina219 * i] = V_data[i];
      }
      /*  + squeeze(Voffset(k_ina219,:,:))'; */
      for (I_idx_1 = 0; I_idx_1 < b_loop_ub_tmp; I_idx_1++) {
        I_idx_0 = k_ina219 + pIshunt_size[0] * I_idx_1;
        Ishunt_data[k_ina219 + (int)Nina219 * I_idx_1] =
            I_data[I_idx_1] * pIshunt_data[I_idx_0] +
            pIshunt_data[I_idx_0 + pIshunt_size[0] * pIshunt_size[1]];
      }
      loop_ub = (int)N_bat;
      for (i = 0; i < loop_ub; i++) {
        tV_data[k_ina219 + (int)Nina219 * i] = tvv_data[i];
      }
      /* (toc0+toc1)/2; */
      loop_ub = (int)N_bat;
      for (i = 0; i < loop_ub; i++) {
        tI_data[k_ina219 + (int)Nina219 * i] = tii_data[i];
      }
      /* tV(k_ina219,:,k_t); */
      /*  switch ProjectFlag */
      /*      case 2%ESP32 */
      /*  end */
    }
  }
  b_Vbat0_size[0] = (int)N_bat;
  b_Vbat0_size[1] = (int)Nina219;
  loop_ub = (int)Nina219;
  for (i = 0; i < loop_ub; i++) {
    I_idx_0 = (int)N_bat;
    for (I_idx_1 = 0; I_idx_1 < I_idx_0; I_idx_1++) {
      b_Vbat0_data[I_idx_1 + (int)N_bat * i] =
          VbatMat_data[i + (int)Nina219 * I_idx_1];
    }
  }
  if (*Vbat0_size - 1 >= 0) {
    memcpy(&Vbat0_data[0], &b_Vbat0_data[0],
           (unsigned int)*Vbat0_size * sizeof(double));
  }
  loop_ub = (int)Nina219;
  for (i = 0; i < loop_ub; i++) {
    I_idx_0 = (int)N_bat;
    for (I_idx_1 = 0; I_idx_1 < I_idx_0; I_idx_1++) {
      tV1mat_data[I_idx_1 + (int)N_bat * i] =
          tV_data[i + (int)Nina219 * I_idx_1];
    }
  }
  *tV1_size = *Vbat0_size;
  if (*Vbat0_size - 1 >= 0) {
    memcpy(&tV1_data[0], &tV1mat_data[0],
           (unsigned int)*Vbat0_size * sizeof(double));
  }
  loop_ub = (int)Nina219;
  for (i = 0; i < loop_ub; i++) {
    I_idx_0 = (int)N_bat;
    for (I_idx_1 = 0; I_idx_1 < I_idx_0; I_idx_1++) {
      tV1mat_data[I_idx_1 + (int)N_bat * i] =
          tI_data[i + (int)Nina219 * I_idx_1];
    }
  }
  *tI1_size = *Vbat0_size;
  if (*Vbat0_size - 1 >= 0) {
    memcpy(&tI1_data[0], &tV1mat_data[0],
           (unsigned int)*Vbat0_size * sizeof(double));
  }
  tV1mat_size[0] = (int)N_bat;
  tV1mat_size[1] = (int)Nina219;
  loop_ub = (int)Nina219;
  for (i = 0; i < loop_ub; i++) {
    I_idx_0 = (int)N_bat;
    for (I_idx_1 = 0; I_idx_1 < I_idx_0; I_idx_1++) {
      tV1mat_data[I_idx_1 + (int)N_bat * i] =
          Ishunt_data[i + (int)Nina219 * I_idx_1];
    }
  }
  *Ibat_size = *Vbat0_size;
  if (*Vbat0_size - 1 >= 0) {
    memcpy(&Ibat_data[0], &tV1mat_data[0],
           (unsigned int)*Vbat0_size * sizeof(double));
  }
  Rwire0_size[0] = Rwire_size[1];
  Rwire0_size[1] = Rwire_size[0];
  loop_ub = Rwire_size[0];
  for (i = 0; i < loop_ub; i++) {
    I_idx_0 = Rwire_size[1];
    for (I_idx_1 = 0; I_idx_1 < I_idx_0; I_idx_1++) {
      Rwire0_data[I_idx_1 + Rwire0_size[0] * i] =
          Rwire_data[i + Rwire_size[0] * I_idx_1];
    }
  }
  I_idx_0 = Rwire_size[0] * Rwire_size[1];
  if (I_idx_0 == 1) {
    b_loop_ub_tmp = *Vbat0_size;
  } else {
    b_loop_ub_tmp = I_idx_0;
  }
  if ((I_idx_0 == *Vbat0_size) && (*Vbat0_size == b_loop_ub_tmp)) {
    Vbat_size = *Vbat0_size;
    for (i = 0; i < *Vbat0_size; i++) {
      Vbat_data[i] =
          (float)b_Vbat0_data[i] + Rwire0_data[i] * (float)tV1mat_data[i];
    }
  } else {
    Vbat_size =
        binary_expand_op_8(Vbat_data, b_Vbat0_data, b_Vbat0_size, Rwire0_data,
                           Rwire0_size, tV1mat_data, tV1mat_size);
  }
  VbatMat0_size[0] = (int)Nina219;
  VbatMat0_size[1] = (int)N_bat;
  if (*Vbat0_size - 1 >= 0) {
    memcpy(&VbatMat0_data[0], &VbatMat_data[0],
           (unsigned int)*Vbat0_size * sizeof(double));
  }
  if (Rwire_size[0] == 1) {
    b_loop_ub_tmp = (int)Nina219;
  } else {
    b_loop_ub_tmp = Rwire_size[0];
  }
  if (Rwire_size[1] == 1) {
    loop_ub_tmp = (int)N_bat;
  } else {
    loop_ub_tmp = Rwire_size[1];
  }
  if ((Rwire_size[0] == (int)Nina219) && (Rwire_size[1] == (int)N_bat) &&
      ((int)Nina219 == b_loop_ub_tmp) && ((int)N_bat == loop_ub_tmp)) {
    for (i = 0; i < *Vbat0_size; i++) {
      VbatMat_data[i] += Rwire_data[i] * Ishunt_data[i];
    }
  } else {
    binary_expand_op_7(VbatMat_data, VbatMat_size, Rwire_data, Rwire_size,
                       Ishunt_data, Ishunt_size);
  }
  return Vbat_size;
}

/*
 * File trailer for ReadVI.c
 *
 * [EOF]
 */
