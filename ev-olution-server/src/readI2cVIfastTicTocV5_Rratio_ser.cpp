/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: readI2cVIfastTicTocV5_Rratio_ser.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 12-Apr-2024 13:06:24
 */

/* Include Files */
#include "readI2cVIfastTicTocV5_Rratio_ser.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_rtwutil.h"
#include "rand.h"
#include "rt_nonfinite.h"
#include "toc.h"
#include <string.h>

/* Type Definitions */
#ifndef typedef_struct_T
#define typedef_struct_T
typedef struct {
  unsigned short data[4];
} struct_T;
#endif /* typedef_struct_T */

/* Function Definitions */
/*
 * Arguments    : const signed char VIpacId[16]
 *                short N_bat1
 *                double V_data[]
 *                int V_size[2]
 *                double I_data[]
 *                int I_size[2]
 *                double tV_data[]
 *                int tV_size[2]
 *                double tI_data[]
 *                int tI_size[2]
 *                double err_data[]
 *                int err_size[2]
 *                double Vdebug_data[]
 *                int Vdebug_size[2]
 * Return Type  : void
 */
void c_readI2cVIfastTicTocV5_Rratio_(
    const signed char VIpacId[16], short N_bat1, double V_data[], int V_size[2],
    double I_data[], int I_size[2], double tV_data[], int tV_size[2],
    double tI_data[], int tI_size[2], double err_data[], int err_size[2],
    double Vdebug_data[], int Vdebug_size[2])
{
  struct_T I2Cmsg0_payload[8];
  struct_T I2Cmsg1_payload[8];
  double b_tI_data[16];
  double a[2];
  double d;
  double toc0;
  double toc1;
  double toc2;
  double toc3;
  int loop_ub;
  int outV1_data_tmp;
  int outV1_size_idx_0;
  int outV1_size_idx_1;
  unsigned short outV1_data[32];
  short b_y;
  unsigned short u;
  short y;
  signed char i;
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
  /*  coder.varsize('k0B',[Nmax.NbatMax,1],[1,1]); */
  /* UNTITLED3 Summary of this function goes here */
  /*    Detailed explanation goes here */
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
  outV1_size_idx_0 = y;
  outV1_size_idx_1 = 2;
  loop_ub = y << 1;
  if (loop_ub - 1 >= 0) {
    memset(&outV1_data[0], 0, (unsigned int)loop_ub * sizeof(unsigned short));
  }
  /* uint16(zeros(N_i2c*2,2)); */
  /*  writeI2C(dev,0x1F,1);%refresh (without avg) writeI2C(dev,0x00,1);%refresh
   * (with avg) */
  if (N_bat1 > 16383) {
    b_y = MAX_int16_T;
  } else if (N_bat1 <= -16384) {
    b_y = MIN_int16_T;
  } else {
    b_y = (short)(N_bat1 << 1);
  }
  loop_ub = b_y;
  if (loop_ub - 1 >= 0) {
    memset(&tV_data[0], 0, (unsigned int)loop_ub * sizeof(double));
  }
  if (N_bat1 > 16383) {
    b_y = MAX_int16_T;
  } else if (N_bat1 <= -16384) {
    b_y = MIN_int16_T;
  } else {
    b_y = (short)(N_bat1 << 1);
  }
  loop_ub = b_y;
  if (loop_ub - 1 >= 0) {
    memset(&tI_data[0], 0, (unsigned int)loop_ub * sizeof(double));
  }
  toc0 = toc();
  for (loop_ub = 0; loop_ub < 8; loop_ub++) {
    b_rand(a);
    d = rt_roundd_snf(a[0] * 10.0);
    if (d < 128.0) {
      if (d >= -128.0) {
        i = (signed char)d;
      } else {
        i = MIN_int8_T;
      }
    } else if (d >= 128.0) {
      i = MAX_int8_T;
    } else {
      i = 0;
    }
    if (i < 0) {
      i = 0;
    }
    I2Cmsg0_payload[loop_ub].data[0] = (unsigned short)i;
    d = rt_roundd_snf(a[1] * 10.0);
    if (d < 128.0) {
      if (d >= -128.0) {
        i = (signed char)d;
      } else {
        i = MIN_int8_T;
      }
    } else if (d >= 128.0) {
      i = MAX_int8_T;
    } else {
      i = 0;
    }
    if (i < 0) {
      i = 0;
    }
    I2Cmsg0_payload[loop_ub].data[1] = (unsigned short)i;
    b_rand(a);
    d = rt_roundd_snf(a[0] * 10.0);
    if (d < 128.0) {
      if (d >= -128.0) {
        i = (signed char)d;
      } else {
        i = MIN_int8_T;
      }
    } else if (d >= 128.0) {
      i = MAX_int8_T;
    } else {
      i = 0;
    }
    if (i < 0) {
      i = 0;
    }
    I2Cmsg0_payload[loop_ub].data[2] = (unsigned short)i;
    d = rt_roundd_snf(a[1] * 10.0);
    if (d < 128.0) {
      if (d >= -128.0) {
        i = (signed char)d;
      } else {
        i = MIN_int8_T;
      }
    } else if (d >= 128.0) {
      i = MAX_int8_T;
    } else {
      i = 0;
    }
    if (i < 0) {
      i = 0;
    }
    I2Cmsg0_payload[loop_ub].data[3] = (unsigned short)i;
  }
  toc1 = toc();
  toc2 = toc();
  for (loop_ub = 0; loop_ub < 8; loop_ub++) {
    b_rand(a);
    d = rt_roundd_snf(a[0] * 10.0);
    if (d < 65536.0) {
      if (d >= 0.0) {
        u = (unsigned short)d;
      } else {
        u = 0U;
      }
    } else if (d >= 65536.0) {
      u = MAX_uint16_T;
    } else {
      u = 0U;
    }
    I2Cmsg1_payload[loop_ub].data[0] = u;
    d = rt_roundd_snf(a[1] * 10.0);
    if (d < 65536.0) {
      if (d >= 0.0) {
        u = (unsigned short)d;
      } else {
        u = 0U;
      }
    } else if (d >= 65536.0) {
      u = MAX_uint16_T;
    } else {
      u = 0U;
    }
    I2Cmsg1_payload[loop_ub].data[1] = u;
    b_rand(a);
    d = rt_roundd_snf(a[0] * 10.0);
    if (d < 65536.0) {
      if (d >= 0.0) {
        u = (unsigned short)d;
      } else {
        u = 0U;
      }
    } else if (d >= 65536.0) {
      u = MAX_uint16_T;
    } else {
      u = 0U;
    }
    I2Cmsg1_payload[loop_ub].data[2] = u;
    d = rt_roundd_snf(a[1] * 10.0);
    if (d < 65536.0) {
      if (d >= 0.0) {
        u = (unsigned short)d;
      } else {
        u = 0U;
      }
    } else if (d >= 65536.0) {
      u = MAX_uint16_T;
    } else {
      u = 0U;
    }
    I2Cmsg1_payload[loop_ub].data[3] = u;
  }
  toc3 = toc();
  d = (toc0 + toc1) / 2.0;
  toc0 = (toc2 + toc3) / 2.0;
  for (loop_ub = 0; loop_ub < 8; loop_ub++) {
    outV1_data[loop_ub] = I2Cmsg0_payload[loop_ub].data[0];
    outV1_data[loop_ub + 8] = I2Cmsg1_payload[loop_ub].data[0];
    outV1_data_tmp = loop_ub + y;
    outV1_data[outV1_data_tmp] = I2Cmsg0_payload[loop_ub].data[1];
    outV1_data[outV1_data_tmp + 8] = I2Cmsg1_payload[loop_ub].data[1];
    tV_data[loop_ub] = d;
    tV_data[loop_ub + 8] = toc0;
    tI_data[loop_ub] = d;
    tI_data[loop_ub + 8] = toc0;
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
    for (outV1_data_tmp = 0; outV1_data_tmp < 16; outV1_data_tmp++) {
      b_tI_data[outV1_data_tmp] = tI_data[VIpacId[outV1_data_tmp] - 1];
    }
    tI_size[0] = 16;
    tI_size[1] = 1;
    for (outV1_data_tmp = 0; outV1_data_tmp < 16; outV1_data_tmp++) {
      tI_data[outV1_data_tmp] = b_tI_data[outV1_data_tmp];
      b_tI_data[outV1_data_tmp] = tV_data[VIpacId[outV1_data_tmp] - 1];
    }
    tV_size[0] = 16;
    tV_size[1] = 1;
    memcpy(&tV_data[0], &b_tI_data[0], 16U * sizeof(double));
    /*              V(k0not) = twos_comp(outV1(k0not,1)*256 +
     * outV1(k0not,2),Nbits)/2^15*9*(1/(8.2/(100+100+8.2))); */
  } else {
    tI_size[0] = 16;
    tI_size[1] = 1;
    tV_size[0] = 16;
    tV_size[1] = 1;
    outV1_size_idx_0 = 16;
    outV1_size_idx_1 = 1;
    memset(&tI_data[0], 0, 16U * sizeof(double));
    memset(&tV_data[0], 0, 16U * sizeof(double));
    for (outV1_data_tmp = 0; outV1_data_tmp < 16; outV1_data_tmp++) {
      outV1_data[outV1_data_tmp] = 0U;
    }
    err_size[0] = 1;
    err_size[1] = 1;
    err_data[0] = -1.0;
  }
  Vdebug_size[0] = outV1_size_idx_0;
  Vdebug_size[1] = outV1_size_idx_1;
  loop_ub = outV1_size_idx_0 * outV1_size_idx_1;
  for (outV1_data_tmp = 0; outV1_data_tmp < loop_ub; outV1_data_tmp++) {
    Vdebug_data[outV1_data_tmp] = outV1_data[outV1_data_tmp];
  }
  c_rand(b_tI_data);
  V_size[0] = 16;
  V_size[1] = 1;
  for (outV1_data_tmp = 0; outV1_data_tmp < 16; outV1_data_tmp++) {
    V_data[outV1_data_tmp] = b_tI_data[outV1_data_tmp] * 2.5 + 2.0;
  }
  c_rand(b_tI_data);
  I_size[0] = 16;
  I_size[1] = 1;
  for (outV1_data_tmp = 0; outV1_data_tmp < 16; outV1_data_tmp++) {
    I_data[outV1_data_tmp] = b_tI_data[outV1_data_tmp] * 5.0;
  }
}

/*
 * Arguments    : const signed char VIpacId[16]
 *                short N_bat1
 *                double V_data[]
 *                int V_size[2]
 *                double I_data[]
 *                int I_size[2]
 *                double tV_data[]
 *                int tV_size[2]
 *                double tI_data[]
 *                int tI_size[2]
 *                double err_data[]
 *                int err_size[2]
 *                double Vdebug_data[]
 *                int Vdebug_size[2]
 * Return Type  : void
 */
void d_readI2cVIfastTicTocV5_Rratio_(
    const signed char VIpacId[16], short N_bat1, double V_data[], int V_size[2],
    double I_data[], int I_size[2], double tV_data[], int tV_size[2],
    double tI_data[], int tI_size[2], double err_data[], int err_size[2],
    double Vdebug_data[], int Vdebug_size[2])
{
  struct_T I2Cmsg0_payload[8];
  struct_T I2Cmsg1_payload[8];
  double b_tI_data[16];
  double a[2];
  double d;
  double toc0;
  double toc1;
  double toc2;
  double toc3;
  int loop_ub;
  int outV1_data_tmp;
  int outV1_size_idx_0;
  int outV1_size_idx_1;
  unsigned short outV1_data[32];
  short b_y;
  unsigned short u;
  short y;
  signed char i;
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
  /*  coder.varsize('k0B',[Nmax.NbatMax,1],[1,1]); */
  /* UNTITLED3 Summary of this function goes here */
  /*    Detailed explanation goes here */
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
  outV1_size_idx_0 = y;
  outV1_size_idx_1 = 2;
  loop_ub = y << 1;
  if (loop_ub - 1 >= 0) {
    memset(&outV1_data[0], 0, (unsigned int)loop_ub * sizeof(unsigned short));
  }
  /* uint16(zeros(N_i2c*2,2)); */
  /*  writeI2C(dev,0x1F,1);%refresh (without avg) writeI2C(dev,0x00,1);%refresh
   * (with avg) */
  if (N_bat1 > 16383) {
    b_y = MAX_int16_T;
  } else if (N_bat1 <= -16384) {
    b_y = MIN_int16_T;
  } else {
    b_y = (short)(N_bat1 << 1);
  }
  loop_ub = b_y;
  if (loop_ub - 1 >= 0) {
    memset(&tV_data[0], 0, (unsigned int)loop_ub * sizeof(double));
  }
  if (N_bat1 > 16383) {
    b_y = MAX_int16_T;
  } else if (N_bat1 <= -16384) {
    b_y = MIN_int16_T;
  } else {
    b_y = (short)(N_bat1 << 1);
  }
  loop_ub = b_y;
  if (loop_ub - 1 >= 0) {
    memset(&tI_data[0], 0, (unsigned int)loop_ub * sizeof(double));
  }
  toc0 = toc();
  for (loop_ub = 0; loop_ub < 8; loop_ub++) {
    b_rand(a);
    d = rt_roundd_snf(a[0] * 10.0);
    if (d < 128.0) {
      if (d >= -128.0) {
        i = (signed char)d;
      } else {
        i = MIN_int8_T;
      }
    } else if (d >= 128.0) {
      i = MAX_int8_T;
    } else {
      i = 0;
    }
    if (i < 0) {
      i = 0;
    }
    I2Cmsg0_payload[loop_ub].data[0] = (unsigned short)i;
    d = rt_roundd_snf(a[1] * 10.0);
    if (d < 128.0) {
      if (d >= -128.0) {
        i = (signed char)d;
      } else {
        i = MIN_int8_T;
      }
    } else if (d >= 128.0) {
      i = MAX_int8_T;
    } else {
      i = 0;
    }
    if (i < 0) {
      i = 0;
    }
    I2Cmsg0_payload[loop_ub].data[1] = (unsigned short)i;
    b_rand(a);
    d = rt_roundd_snf(a[0] * 10.0);
    if (d < 128.0) {
      if (d >= -128.0) {
        i = (signed char)d;
      } else {
        i = MIN_int8_T;
      }
    } else if (d >= 128.0) {
      i = MAX_int8_T;
    } else {
      i = 0;
    }
    if (i < 0) {
      i = 0;
    }
    I2Cmsg0_payload[loop_ub].data[2] = (unsigned short)i;
    d = rt_roundd_snf(a[1] * 10.0);
    if (d < 128.0) {
      if (d >= -128.0) {
        i = (signed char)d;
      } else {
        i = MIN_int8_T;
      }
    } else if (d >= 128.0) {
      i = MAX_int8_T;
    } else {
      i = 0;
    }
    if (i < 0) {
      i = 0;
    }
    I2Cmsg0_payload[loop_ub].data[3] = (unsigned short)i;
  }
  toc1 = toc();
  toc2 = toc();
  for (loop_ub = 0; loop_ub < 8; loop_ub++) {
    b_rand(a);
    d = rt_roundd_snf(a[0] * 10.0);
    if (d < 65536.0) {
      if (d >= 0.0) {
        u = (unsigned short)d;
      } else {
        u = 0U;
      }
    } else if (d >= 65536.0) {
      u = MAX_uint16_T;
    } else {
      u = 0U;
    }
    I2Cmsg1_payload[loop_ub].data[0] = u;
    d = rt_roundd_snf(a[1] * 10.0);
    if (d < 65536.0) {
      if (d >= 0.0) {
        u = (unsigned short)d;
      } else {
        u = 0U;
      }
    } else if (d >= 65536.0) {
      u = MAX_uint16_T;
    } else {
      u = 0U;
    }
    I2Cmsg1_payload[loop_ub].data[1] = u;
    b_rand(a);
    d = rt_roundd_snf(a[0] * 10.0);
    if (d < 65536.0) {
      if (d >= 0.0) {
        u = (unsigned short)d;
      } else {
        u = 0U;
      }
    } else if (d >= 65536.0) {
      u = MAX_uint16_T;
    } else {
      u = 0U;
    }
    I2Cmsg1_payload[loop_ub].data[2] = u;
    d = rt_roundd_snf(a[1] * 10.0);
    if (d < 65536.0) {
      if (d >= 0.0) {
        u = (unsigned short)d;
      } else {
        u = 0U;
      }
    } else if (d >= 65536.0) {
      u = MAX_uint16_T;
    } else {
      u = 0U;
    }
    I2Cmsg1_payload[loop_ub].data[3] = u;
  }
  toc3 = toc();
  d = (toc0 + toc1) / 2.0;
  toc0 = (toc2 + toc3) / 2.0;
  for (loop_ub = 0; loop_ub < 8; loop_ub++) {
    outV1_data[loop_ub] = I2Cmsg0_payload[loop_ub].data[0];
    outV1_data[loop_ub + 8] = I2Cmsg1_payload[loop_ub].data[0];
    outV1_data_tmp = loop_ub + y;
    outV1_data[outV1_data_tmp] = I2Cmsg0_payload[loop_ub].data[1];
    outV1_data[outV1_data_tmp + 8] = I2Cmsg1_payload[loop_ub].data[1];
    tV_data[loop_ub] = d;
    tV_data[loop_ub + 8] = toc0;
    tI_data[loop_ub] = d;
    tI_data[loop_ub + 8] = toc0;
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
    for (outV1_data_tmp = 0; outV1_data_tmp < 16; outV1_data_tmp++) {
      b_tI_data[outV1_data_tmp] = tI_data[VIpacId[outV1_data_tmp] - 1];
    }
    tI_size[0] = 16;
    tI_size[1] = 1;
    for (outV1_data_tmp = 0; outV1_data_tmp < 16; outV1_data_tmp++) {
      tI_data[outV1_data_tmp] = b_tI_data[outV1_data_tmp];
      b_tI_data[outV1_data_tmp] = tV_data[VIpacId[outV1_data_tmp] - 1];
    }
    tV_size[0] = 16;
    tV_size[1] = 1;
    memcpy(&tV_data[0], &b_tI_data[0], 16U * sizeof(double));
  } else {
    tI_size[0] = 16;
    tI_size[1] = 1;
    tV_size[0] = 16;
    tV_size[1] = 1;
    outV1_size_idx_0 = 16;
    outV1_size_idx_1 = 1;
    memset(&tI_data[0], 0, 16U * sizeof(double));
    memset(&tV_data[0], 0, 16U * sizeof(double));
    for (outV1_data_tmp = 0; outV1_data_tmp < 16; outV1_data_tmp++) {
      outV1_data[outV1_data_tmp] = 0U;
    }
    err_size[0] = 1;
    err_size[1] = 1;
    err_data[0] = -1.0;
  }
  Vdebug_size[0] = outV1_size_idx_0;
  Vdebug_size[1] = outV1_size_idx_1;
  loop_ub = outV1_size_idx_0 * outV1_size_idx_1;
  for (outV1_data_tmp = 0; outV1_data_tmp < loop_ub; outV1_data_tmp++) {
    Vdebug_data[outV1_data_tmp] = outV1_data[outV1_data_tmp];
  }
  c_rand(b_tI_data);
  V_size[0] = 16;
  V_size[1] = 1;
  for (outV1_data_tmp = 0; outV1_data_tmp < 16; outV1_data_tmp++) {
    V_data[outV1_data_tmp] = b_tI_data[outV1_data_tmp] * 2.5 + 2.0;
  }
  c_rand(b_tI_data);
  I_size[0] = 16;
  I_size[1] = 1;
  for (outV1_data_tmp = 0; outV1_data_tmp < 16; outV1_data_tmp++) {
    I_data[outV1_data_tmp] = b_tI_data[outV1_data_tmp] * 5.0;
  }
}

/*
 * File trailer for readI2cVIfastTicTocV5_Rratio_ser.c
 *
 * [EOF]
 */
