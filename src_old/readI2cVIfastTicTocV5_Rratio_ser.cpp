/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: readI2cVIfastTicTocV5_Rratio_ser.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 17-Mar-2024 11:09:30
 */

/* Include Files */
#include "readI2cVIfastTicTocV5_Rratio_ser.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_emxutil.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_rtwutil.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h"
#include "rand.h"
#include "rt_nonfinite.h"
#include "toc.h"

/* Type Definitions */
#ifndef typedef_struct_T
#define typedef_struct_T
typedef struct {
  unsigned short data[4];
} struct_T;
#endif /* typedef_struct_T */

/* Function Definitions */
/*
 * UNTITLED3 Summary of this function goes here
 *    Detailed explanation goes here
 *
 * Arguments    : const signed char VIpacId[16]
 *                short N_bat1
 *                emxArray_real_T *V
 *                emxArray_real_T *b_I
 *                double tV[16]
 *                double tI[16]
 *                double err_data[]
 *                int err_size[2]
 *                emxArray_real_T *Vdebug
 * Return Type  : void
 */
void c_readI2cVIfastTicTocV5_Rratio_(const signed char VIpacId[16],
                                     short N_bat1, emxArray_real_T *V,
                                     emxArray_real_T *b_I, double tV[16],
                                     double tI[16], double err_data[],
                                     int err_size[2], emxArray_real_T *Vdebug)
{
  emxArray_real_T *b_tI;
  emxArray_real_T *b_tV;
  emxArray_uint16_T *outV2;
  struct_T I2Cmsg0_payload[8];
  struct_T I2Cmsg1_payload[8];
  double VcolB[16];
  double a[2];
  double toc0;
  double toc1;
  double toc2;
  double toc3;
  double *tI_data;
  double *tV_data;
  int b_i;
  int i;
  unsigned short u;
  short y;
  unsigned short *outV2_data;
  signed char i1;
  err_size[0] = 0;
  err_size[1] = 0;
  /*  PAC_vbus1   = 0x07; */
  /*  PAC_vbus2   = 0x08; */
  /*  PAC_vsense1 = 0x0B; */
  /*  PAC_vsense2 = 0x0C; */
  /*  PAC_vbus1_avg   = 0x0F; */
  /*  PAC_vbus2_avg   = 0x10; */
  /*  PAC_vsense1_avg = 0x13; */
  /*  PAC_vsense2_avg = 0x14; */
  /*  Rshunt = Esp32_v1.Rshunt; */
  /*  if ~AvgFlag */
  /*      PAC_vbus1_0   = PAC_vbus1; */
  /*      PAC_vbus2_0   = PAC_vbus2; */
  /*      PAC_vsense1_0 = PAC_vsense1; */
  /*      PAC_vsense2_0 = PAC_vsense2; */
  /*  else %avrg */
  /*      PAC_vbus1_0   = PAC_vbus1_avg; */
  /*      PAC_vbus2_0   = PAC_vbus2_avg; */
  /*      PAC_vsense1_0 = PAC_vsense1_avg; */
  /*      PAC_vsense2_0 = PAC_vsense2_avg; */
  /*  end */
  /*  */
  /*  N_i2c = N_bat1*N_bat2;%length(Esp32_v1.i2c); */
  /* *N_bat2; */
  if (N_bat1 > 16383) {
    y = MAX_int16_T;
  } else if (N_bat1 <= -16384) {
    y = MIN_int16_T;
  } else {
    y = (short)(N_bat1 << 1);
  }
  emxInit_uint16_T(&outV2, 2);
  i = outV2->size[0] * outV2->size[1];
  outV2->size[0] = y;
  outV2->size[1] = 2;
  emxEnsureCapacity_uint16_T(outV2, i);
  outV2_data = outV2->data;
  b_i = y << 1;
  for (i = 0; i < b_i; i++) {
    outV2_data[i] = 0U;
  }
  /* uint16(zeros(N_i2c*2,2)); */
  /*  writeI2C(dev,0x1F,1);%refresh (without avg) writeI2C(dev,0x00,1);%refresh
   * (with avg) */
  if (N_bat1 > 16383) {
    y = MAX_int16_T;
  } else if (N_bat1 <= -16384) {
    y = MIN_int16_T;
  } else {
    y = (short)(N_bat1 << 1);
  }
  emxInit_real_T(&b_tV, 1);
  i = b_tV->size[0];
  b_tV->size[0] = y;
  emxEnsureCapacity_real_T(b_tV, i);
  tV_data = b_tV->data;
  b_i = y;
  for (i = 0; i < b_i; i++) {
    tV_data[i] = 0.0;
  }
  if (N_bat1 > 16383) {
    y = MAX_int16_T;
  } else if (N_bat1 <= -16384) {
    y = MIN_int16_T;
  } else {
    y = (short)(N_bat1 << 1);
  }
  emxInit_real_T(&b_tI, 1);
  i = b_tI->size[0];
  b_tI->size[0] = y;
  emxEnsureCapacity_real_T(b_tI, i);
  tI_data = b_tI->data;
  b_i = y;
  for (i = 0; i < b_i; i++) {
    tI_data[i] = 0.0;
  }
  toc0 = toc();
  for (b_i = 0; b_i < 8; b_i++) {
    b_rand(a);
    toc3 = rt_roundd_snf(a[0] * 10.0);
    if (toc3 < 128.0) {
      if (toc3 >= -128.0) {
        i1 = (signed char)toc3;
      } else {
        i1 = MIN_int8_T;
      }
    } else if (toc3 >= 128.0) {
      i1 = MAX_int8_T;
    } else {
      i1 = 0;
    }
    if (i1 < 0) {
      i1 = 0;
    }
    I2Cmsg0_payload[b_i].data[0] = (unsigned short)i1;
    toc3 = rt_roundd_snf(a[1] * 10.0);
    if (toc3 < 128.0) {
      if (toc3 >= -128.0) {
        i1 = (signed char)toc3;
      } else {
        i1 = MIN_int8_T;
      }
    } else if (toc3 >= 128.0) {
      i1 = MAX_int8_T;
    } else {
      i1 = 0;
    }
    if (i1 < 0) {
      i1 = 0;
    }
    I2Cmsg0_payload[b_i].data[1] = (unsigned short)i1;
    b_rand(a);
    toc3 = rt_roundd_snf(a[0] * 10.0);
    if (toc3 < 128.0) {
      if (toc3 >= -128.0) {
        i1 = (signed char)toc3;
      } else {
        i1 = MIN_int8_T;
      }
    } else if (toc3 >= 128.0) {
      i1 = MAX_int8_T;
    } else {
      i1 = 0;
    }
    if (i1 < 0) {
      i1 = 0;
    }
    I2Cmsg0_payload[b_i].data[2] = (unsigned short)i1;
    toc3 = rt_roundd_snf(a[1] * 10.0);
    if (toc3 < 128.0) {
      if (toc3 >= -128.0) {
        i1 = (signed char)toc3;
      } else {
        i1 = MIN_int8_T;
      }
    } else if (toc3 >= 128.0) {
      i1 = MAX_int8_T;
    } else {
      i1 = 0;
    }
    if (i1 < 0) {
      i1 = 0;
    }
    I2Cmsg0_payload[b_i].data[3] = (unsigned short)i1;
  }
  toc1 = toc();
  toc2 = toc();
  for (b_i = 0; b_i < 8; b_i++) {
    b_rand(a);
    toc3 = rt_roundd_snf(a[0] * 10.0);
    if (toc3 < 65536.0) {
      if (toc3 >= 0.0) {
        u = (unsigned short)toc3;
      } else {
        u = 0U;
      }
    } else if (toc3 >= 65536.0) {
      u = MAX_uint16_T;
    } else {
      u = 0U;
    }
    I2Cmsg1_payload[b_i].data[0] = u;
    toc3 = rt_roundd_snf(a[1] * 10.0);
    if (toc3 < 65536.0) {
      if (toc3 >= 0.0) {
        u = (unsigned short)toc3;
      } else {
        u = 0U;
      }
    } else if (toc3 >= 65536.0) {
      u = MAX_uint16_T;
    } else {
      u = 0U;
    }
    I2Cmsg1_payload[b_i].data[1] = u;
    b_rand(a);
    toc3 = rt_roundd_snf(a[0] * 10.0);
    if (toc3 < 65536.0) {
      if (toc3 >= 0.0) {
        u = (unsigned short)toc3;
      } else {
        u = 0U;
      }
    } else if (toc3 >= 65536.0) {
      u = MAX_uint16_T;
    } else {
      u = 0U;
    }
    I2Cmsg1_payload[b_i].data[2] = u;
    toc3 = rt_roundd_snf(a[1] * 10.0);
    if (toc3 < 65536.0) {
      if (toc3 >= 0.0) {
        u = (unsigned short)toc3;
      } else {
        u = 0U;
      }
    } else if (toc3 >= 65536.0) {
      u = MAX_uint16_T;
    } else {
      u = 0U;
    }
    I2Cmsg1_payload[b_i].data[3] = u;
  }
  toc3 = toc();
  toc0 = (toc0 + toc1) / 2.0;
  toc3 = (toc2 + toc3) / 2.0;
  for (b_i = 0; b_i < 8; b_i++) {
    outV2_data[b_i] = I2Cmsg0_payload[b_i].data[0];
    outV2_data[b_i + outV2->size[0]] = I2Cmsg0_payload[b_i].data[1];
    outV2_data[b_i + 8] = I2Cmsg1_payload[b_i].data[0];
    outV2_data[(b_i + outV2->size[0]) + 8] = I2Cmsg1_payload[b_i].data[1];
    tV_data[b_i] = toc0;
    tV_data[b_i + 8] = toc3;
    tI_data[b_i] = toc0;
    tI_data[b_i + 8] = toc3;
  }
  /*  for k_i2c = 1:N_i2c */
  /*      dev = Esp32_v1.i2c(k_i2c); */
  /*      try */
  /*  %         %ORG */
  /*  %         writeRegister(dev,0x1D,[0x55
   * 0x55],'uint8');%writeI2C16uint(dev,0x1D,21845);%writeRegister(dev,0x1D,[0x55
   * 0x55],'uint8');% writeI2C(dev,0x1D,0x5555);%Bi polar */
  /*  %         [~]=readRegister(dev,0x1F,1); */
  /*  %         %ORG - END */
  /*           %Test 1 */
  /*  %         writeRegister(dev,0x1D,[0x55
   * 0x55],'uint8');%writeI2C16uint(dev,0x1D,21845);%writeRegister(dev,0x1D,[0x55
   * 0x55],'uint8');% writeI2C(dev,0x1D,0x5555);%Bi polar */
  /*          [~]=readRegister(dev,0x1F,1); */
  /*          %Test 1 - END */
  /*          %
   * writeI2C(dev,0x1F,0);%writeRegister(dev,0x1F,1,'uint8');%writeI2C(dev,0x1F,1);%refresh
   * (without avg) writeI2C(dev,0x00,1);%refresh (with avg) */
  /*          %     pause(0.002); */
  /*          if Vflag */
  /*              toc0 = toc; */
  /*              try */
  /*                  outV1(2*(k_i2c-1)+1,:) =
   * readI2Cfast(Esp32_v1.i2c(k_i2c),PAC_vbus1_0,2); */
  /*              catch */
  /*                  outV1(2*(k_i2c-1)+1,:) =
   * readI2Cfast(Esp32_v1.i2c(k_i2c),PAC_vbus1_0,2); */
  /*              end */
  /*              toc1 = toc; */
  /*  %             pause(1e-2); */
  /*              toc2 = toc; */
  /*              try */
  /*                  outV1(2*(k_i2c-1)+2,:) =
   * readI2Cfast(Esp32_v1.i2c(k_i2c),PAC_vbus2_0,2); */
  /*              catch */
  /*                  outV1(2*(k_i2c-1)+2,:) =
   * readI2Cfast(Esp32_v1.i2c(k_i2c),PAC_vbus2_0,2); */
  /*              end */
  /*              toc3 = toc; */
  /*  %             pause(1e-2); */
  /*              tV((k_i2c-1)*2+1) = (toc0+toc1)/2; */
  /*              tV((k_i2c-1)*2+2) = (toc2+toc3)/2; */
  /*          end */
  /*          if Iflag */
  /*              toc0 = toc; */
  /*              outI1(2*(k_i2c-1)+1,:) =
   * readI2Cfast(Esp32_v1.i2c(k_i2c),PAC_vsense1_0,2); */
  /*              toc1 = toc; */
  /*              pause(1e-2); */
  /*              toc2 = toc; */
  /*              outI1(2*(k_i2c-1)+2,:) =
   * readI2Cfast(Esp32_v1.i2c(k_i2c),PAC_vsense2_0,2); */
  /*              toc3 = toc; */
  /*              pause(1e-2); */
  /*              tI((k_i2c-1)*2+1) = (toc0+toc1)/2; */
  /*              tI((k_i2c-1)*2+2) = (toc2+toc3)/2; */
  /*          end */
  /*      catch */
  /*         disp (['I2C error ',num2str(k_i2c)] ); */
  /*         err = [err k_i2c]; */
  /*      end */
  /*  end */
  if (N_bat1 > 0) {
    for (i = 0; i < 16; i++) {
      b_i = VIpacId[i] - 1;
      tI[i] = tI_data[b_i];
      tV[i] = tV_data[b_i];
    }
    /*              V(k0not) = twos_comp(outV1(k0not,1)*256 +
     * outV1(k0not,2),Nbits)/2^15*9*(1/(8.2/(100+100+8.2))); */
  } else {
    i = outV2->size[0] * outV2->size[1];
    outV2->size[0] = 16;
    outV2->size[1] = 1;
    emxEnsureCapacity_uint16_T(outV2, i);
    outV2_data = outV2->data;
    for (b_i = 0; b_i < 16; b_i++) {
      tI[b_i] = 0.0;
      tV[b_i] = 0.0;
      outV2_data[b_i] = 0U;
    }
    err_size[0] = 1;
    err_size[1] = 1;
    err_data[0] = -1.0;
  }
  emxFree_real_T(&b_tI);
  emxFree_real_T(&b_tV);
  i = Vdebug->size[0] * Vdebug->size[1];
  Vdebug->size[0] = outV2->size[0];
  Vdebug->size[1] = outV2->size[1];
  emxEnsureCapacity_real_T(Vdebug, i);
  tV_data = Vdebug->data;
  b_i = outV2->size[0] * outV2->size[1];
  for (i = 0; i < b_i; i++) {
    tV_data[i] = outV2_data[i];
  }
  emxFree_uint16_T(&outV2);
  c_rand(VcolB);
  i = V->size[0];
  V->size[0] = 16;
  emxEnsureCapacity_real_T(V, i);
  tV_data = V->data;
  for (i = 0; i < 16; i++) {
    tV_data[i] = VcolB[i] * 2.5 + 2.0;
  }
  c_rand(VcolB);
  i = b_I->size[0];
  b_I->size[0] = 16;
  emxEnsureCapacity_real_T(b_I, i);
  tV_data = b_I->data;
  for (i = 0; i < 16; i++) {
    tV_data[i] = VcolB[i] * 5.0;
  }
}

/*
 * UNTITLED3 Summary of this function goes here
 *    Detailed explanation goes here
 *
 * Arguments    : const signed char VIpacId[16]
 *                short N_bat1
 *                emxArray_real_T *V
 *                emxArray_real_T *b_I
 *                double tV[16]
 *                double tI[16]
 *                double err_data[]
 *                int err_size[2]
 *                emxArray_real_T *Vdebug
 * Return Type  : void
 */
void d_readI2cVIfastTicTocV5_Rratio_(const signed char VIpacId[16],
                                     short N_bat1, emxArray_real_T *V,
                                     emxArray_real_T *b_I, double tV[16],
                                     double tI[16], double err_data[],
                                     int err_size[2], emxArray_real_T *Vdebug)
{
  emxArray_real_T *b_tI;
  emxArray_real_T *b_tV;
  emxArray_uint16_T *outV2;
  struct_T I2Cmsg0_payload[8];
  struct_T I2Cmsg1_payload[8];
  double VcolB[16];
  double a[2];
  double toc0;
  double toc1;
  double toc2;
  double toc3;
  double *tI_data;
  double *tV_data;
  int b_i;
  int i;
  unsigned short u;
  short y;
  unsigned short *outV2_data;
  signed char i1;
  err_size[0] = 0;
  err_size[1] = 0;
  /*  PAC_vbus1   = 0x07; */
  /*  PAC_vbus2   = 0x08; */
  /*  PAC_vsense1 = 0x0B; */
  /*  PAC_vsense2 = 0x0C; */
  /*  PAC_vbus1_avg   = 0x0F; */
  /*  PAC_vbus2_avg   = 0x10; */
  /*  PAC_vsense1_avg = 0x13; */
  /*  PAC_vsense2_avg = 0x14; */
  /*  Rshunt = Esp32_v1.Rshunt; */
  /*  if ~AvgFlag */
  /*      PAC_vbus1_0   = PAC_vbus1; */
  /*      PAC_vbus2_0   = PAC_vbus2; */
  /*      PAC_vsense1_0 = PAC_vsense1; */
  /*      PAC_vsense2_0 = PAC_vsense2; */
  /*  else %avrg */
  /*      PAC_vbus1_0   = PAC_vbus1_avg; */
  /*      PAC_vbus2_0   = PAC_vbus2_avg; */
  /*      PAC_vsense1_0 = PAC_vsense1_avg; */
  /*      PAC_vsense2_0 = PAC_vsense2_avg; */
  /*  end */
  /*  */
  /*  N_i2c = N_bat1*N_bat2;%length(Esp32_v1.i2c); */
  /* *N_bat2; */
  if (N_bat1 > 16383) {
    y = MAX_int16_T;
  } else if (N_bat1 <= -16384) {
    y = MIN_int16_T;
  } else {
    y = (short)(N_bat1 << 1);
  }
  emxInit_uint16_T(&outV2, 2);
  i = outV2->size[0] * outV2->size[1];
  outV2->size[0] = y;
  outV2->size[1] = 2;
  emxEnsureCapacity_uint16_T(outV2, i);
  outV2_data = outV2->data;
  b_i = y << 1;
  for (i = 0; i < b_i; i++) {
    outV2_data[i] = 0U;
  }
  /* uint16(zeros(N_i2c*2,2)); */
  /*  writeI2C(dev,0x1F,1);%refresh (without avg) writeI2C(dev,0x00,1);%refresh
   * (with avg) */
  if (N_bat1 > 16383) {
    y = MAX_int16_T;
  } else if (N_bat1 <= -16384) {
    y = MIN_int16_T;
  } else {
    y = (short)(N_bat1 << 1);
  }
  emxInit_real_T(&b_tV, 1);
  i = b_tV->size[0];
  b_tV->size[0] = y;
  emxEnsureCapacity_real_T(b_tV, i);
  tV_data = b_tV->data;
  b_i = y;
  for (i = 0; i < b_i; i++) {
    tV_data[i] = 0.0;
  }
  if (N_bat1 > 16383) {
    y = MAX_int16_T;
  } else if (N_bat1 <= -16384) {
    y = MIN_int16_T;
  } else {
    y = (short)(N_bat1 << 1);
  }
  emxInit_real_T(&b_tI, 1);
  i = b_tI->size[0];
  b_tI->size[0] = y;
  emxEnsureCapacity_real_T(b_tI, i);
  tI_data = b_tI->data;
  b_i = y;
  for (i = 0; i < b_i; i++) {
    tI_data[i] = 0.0;
  }
  toc0 = toc();
  for (b_i = 0; b_i < 8; b_i++) {
    b_rand(a);
    toc3 = rt_roundd_snf(a[0] * 10.0);
    if (toc3 < 128.0) {
      if (toc3 >= -128.0) {
        i1 = (signed char)toc3;
      } else {
        i1 = MIN_int8_T;
      }
    } else if (toc3 >= 128.0) {
      i1 = MAX_int8_T;
    } else {
      i1 = 0;
    }
    if (i1 < 0) {
      i1 = 0;
    }
    I2Cmsg0_payload[b_i].data[0] = (unsigned short)i1;
    toc3 = rt_roundd_snf(a[1] * 10.0);
    if (toc3 < 128.0) {
      if (toc3 >= -128.0) {
        i1 = (signed char)toc3;
      } else {
        i1 = MIN_int8_T;
      }
    } else if (toc3 >= 128.0) {
      i1 = MAX_int8_T;
    } else {
      i1 = 0;
    }
    if (i1 < 0) {
      i1 = 0;
    }
    I2Cmsg0_payload[b_i].data[1] = (unsigned short)i1;
    b_rand(a);
    toc3 = rt_roundd_snf(a[0] * 10.0);
    if (toc3 < 128.0) {
      if (toc3 >= -128.0) {
        i1 = (signed char)toc3;
      } else {
        i1 = MIN_int8_T;
      }
    } else if (toc3 >= 128.0) {
      i1 = MAX_int8_T;
    } else {
      i1 = 0;
    }
    if (i1 < 0) {
      i1 = 0;
    }
    I2Cmsg0_payload[b_i].data[2] = (unsigned short)i1;
    toc3 = rt_roundd_snf(a[1] * 10.0);
    if (toc3 < 128.0) {
      if (toc3 >= -128.0) {
        i1 = (signed char)toc3;
      } else {
        i1 = MIN_int8_T;
      }
    } else if (toc3 >= 128.0) {
      i1 = MAX_int8_T;
    } else {
      i1 = 0;
    }
    if (i1 < 0) {
      i1 = 0;
    }
    I2Cmsg0_payload[b_i].data[3] = (unsigned short)i1;
  }
  toc1 = toc();
  toc2 = toc();
  for (b_i = 0; b_i < 8; b_i++) {
    b_rand(a);
    toc3 = rt_roundd_snf(a[0] * 10.0);
    if (toc3 < 65536.0) {
      if (toc3 >= 0.0) {
        u = (unsigned short)toc3;
      } else {
        u = 0U;
      }
    } else if (toc3 >= 65536.0) {
      u = MAX_uint16_T;
    } else {
      u = 0U;
    }
    I2Cmsg1_payload[b_i].data[0] = u;
    toc3 = rt_roundd_snf(a[1] * 10.0);
    if (toc3 < 65536.0) {
      if (toc3 >= 0.0) {
        u = (unsigned short)toc3;
      } else {
        u = 0U;
      }
    } else if (toc3 >= 65536.0) {
      u = MAX_uint16_T;
    } else {
      u = 0U;
    }
    I2Cmsg1_payload[b_i].data[1] = u;
    b_rand(a);
    toc3 = rt_roundd_snf(a[0] * 10.0);
    if (toc3 < 65536.0) {
      if (toc3 >= 0.0) {
        u = (unsigned short)toc3;
      } else {
        u = 0U;
      }
    } else if (toc3 >= 65536.0) {
      u = MAX_uint16_T;
    } else {
      u = 0U;
    }
    I2Cmsg1_payload[b_i].data[2] = u;
    toc3 = rt_roundd_snf(a[1] * 10.0);
    if (toc3 < 65536.0) {
      if (toc3 >= 0.0) {
        u = (unsigned short)toc3;
      } else {
        u = 0U;
      }
    } else if (toc3 >= 65536.0) {
      u = MAX_uint16_T;
    } else {
      u = 0U;
    }
    I2Cmsg1_payload[b_i].data[3] = u;
  }
  toc3 = toc();
  toc0 = (toc0 + toc1) / 2.0;
  toc3 = (toc2 + toc3) / 2.0;
  for (b_i = 0; b_i < 8; b_i++) {
    outV2_data[b_i] = I2Cmsg0_payload[b_i].data[0];
    outV2_data[b_i + outV2->size[0]] = I2Cmsg0_payload[b_i].data[1];
    outV2_data[b_i + 8] = I2Cmsg1_payload[b_i].data[0];
    outV2_data[(b_i + outV2->size[0]) + 8] = I2Cmsg1_payload[b_i].data[1];
    tV_data[b_i] = toc0;
    tV_data[b_i + 8] = toc3;
    tI_data[b_i] = toc0;
    tI_data[b_i + 8] = toc3;
  }
  /*  for k_i2c = 1:N_i2c */
  /*      dev = Esp32_v1.i2c(k_i2c); */
  /*      try */
  /*  %         %ORG */
  /*  %         writeRegister(dev,0x1D,[0x55
   * 0x55],'uint8');%writeI2C16uint(dev,0x1D,21845);%writeRegister(dev,0x1D,[0x55
   * 0x55],'uint8');% writeI2C(dev,0x1D,0x5555);%Bi polar */
  /*  %         [~]=readRegister(dev,0x1F,1); */
  /*  %         %ORG - END */
  /*           %Test 1 */
  /*  %         writeRegister(dev,0x1D,[0x55
   * 0x55],'uint8');%writeI2C16uint(dev,0x1D,21845);%writeRegister(dev,0x1D,[0x55
   * 0x55],'uint8');% writeI2C(dev,0x1D,0x5555);%Bi polar */
  /*          [~]=readRegister(dev,0x1F,1); */
  /*          %Test 1 - END */
  /*          %
   * writeI2C(dev,0x1F,0);%writeRegister(dev,0x1F,1,'uint8');%writeI2C(dev,0x1F,1);%refresh
   * (without avg) writeI2C(dev,0x00,1);%refresh (with avg) */
  /*          %     pause(0.002); */
  /*          if Vflag */
  /*              toc0 = toc; */
  /*              try */
  /*                  outV1(2*(k_i2c-1)+1,:) =
   * readI2Cfast(Esp32_v1.i2c(k_i2c),PAC_vbus1_0,2); */
  /*              catch */
  /*                  outV1(2*(k_i2c-1)+1,:) =
   * readI2Cfast(Esp32_v1.i2c(k_i2c),PAC_vbus1_0,2); */
  /*              end */
  /*              toc1 = toc; */
  /*  %             pause(1e-2); */
  /*              toc2 = toc; */
  /*              try */
  /*                  outV1(2*(k_i2c-1)+2,:) =
   * readI2Cfast(Esp32_v1.i2c(k_i2c),PAC_vbus2_0,2); */
  /*              catch */
  /*                  outV1(2*(k_i2c-1)+2,:) =
   * readI2Cfast(Esp32_v1.i2c(k_i2c),PAC_vbus2_0,2); */
  /*              end */
  /*              toc3 = toc; */
  /*  %             pause(1e-2); */
  /*              tV((k_i2c-1)*2+1) = (toc0+toc1)/2; */
  /*              tV((k_i2c-1)*2+2) = (toc2+toc3)/2; */
  /*          end */
  /*          if Iflag */
  /*              toc0 = toc; */
  /*              outI1(2*(k_i2c-1)+1,:) =
   * readI2Cfast(Esp32_v1.i2c(k_i2c),PAC_vsense1_0,2); */
  /*              toc1 = toc; */
  /*              pause(1e-2); */
  /*              toc2 = toc; */
  /*              outI1(2*(k_i2c-1)+2,:) =
   * readI2Cfast(Esp32_v1.i2c(k_i2c),PAC_vsense2_0,2); */
  /*              toc3 = toc; */
  /*              pause(1e-2); */
  /*              tI((k_i2c-1)*2+1) = (toc0+toc1)/2; */
  /*              tI((k_i2c-1)*2+2) = (toc2+toc3)/2; */
  /*          end */
  /*      catch */
  /*         disp (['I2C error ',num2str(k_i2c)] ); */
  /*         err = [err k_i2c]; */
  /*      end */
  /*  end */
  if (N_bat1 > 0) {
    for (i = 0; i < 16; i++) {
      b_i = VIpacId[i] - 1;
      tI[i] = tI_data[b_i];
      tV[i] = tV_data[b_i];
    }
  } else {
    i = outV2->size[0] * outV2->size[1];
    outV2->size[0] = 16;
    outV2->size[1] = 1;
    emxEnsureCapacity_uint16_T(outV2, i);
    outV2_data = outV2->data;
    for (b_i = 0; b_i < 16; b_i++) {
      tI[b_i] = 0.0;
      tV[b_i] = 0.0;
      outV2_data[b_i] = 0U;
    }
    err_size[0] = 1;
    err_size[1] = 1;
    err_data[0] = -1.0;
  }
  emxFree_real_T(&b_tI);
  emxFree_real_T(&b_tV);
  i = Vdebug->size[0] * Vdebug->size[1];
  Vdebug->size[0] = outV2->size[0];
  Vdebug->size[1] = outV2->size[1];
  emxEnsureCapacity_real_T(Vdebug, i);
  tV_data = Vdebug->data;
  b_i = outV2->size[0] * outV2->size[1];
  for (i = 0; i < b_i; i++) {
    tV_data[i] = outV2_data[i];
  }
  emxFree_uint16_T(&outV2);
  c_rand(VcolB);
  i = V->size[0];
  V->size[0] = 16;
  emxEnsureCapacity_real_T(V, i);
  tV_data = V->data;
  for (i = 0; i < 16; i++) {
    tV_data[i] = VcolB[i] * 2.5 + 2.0;
  }
  c_rand(VcolB);
  i = b_I->size[0];
  b_I->size[0] = 16;
  emxEnsureCapacity_real_T(b_I, i);
  tV_data = b_I->data;
  for (i = 0; i < 16; i++) {
    tV_data[i] = VcolB[i] * 5.0;
  }
}

/*
 * File trailer for readI2cVIfastTicTocV5_Rratio_ser.c
 *
 * [EOF]
 */
