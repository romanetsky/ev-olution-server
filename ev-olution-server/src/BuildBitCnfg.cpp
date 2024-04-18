/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: BuildBitCnfg.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 17-Apr-2024 18:20:59
 */

/* Include Files */
#include "BuildBitCnfg.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_emxutil.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_rtwutil.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h"
#include "rt_nonfinite.h"
#include "sort.h"
#include <stddef.h>
#include <stdio.h>
#include <string.h>

/* Function Declarations */
static short div_s16s32_floor(int numerator, int denominator);

/* Function Definitions */
/*
 * Arguments    : int numerator
 *                int denominator
 * Return Type  : short
 */
static short div_s16s32_floor(int numerator, int denominator)
{
  unsigned int absDenominator;
  unsigned int absNumerator;
  unsigned int tempAbsQuotient;
  short quotient;
  boolean_T quotientNeedsNegation;
  if (denominator == 0) {
    if (numerator >= 0) {
      quotient = MAX_int16_T;
    } else {
      quotient = MIN_int16_T;
    }
  } else {
    if (numerator < 0) {
      absNumerator = ~(unsigned int)numerator + 1U;
    } else {
      absNumerator = (unsigned int)numerator;
    }
    if (denominator < 0) {
      absDenominator = ~(unsigned int)denominator + 1U;
    } else {
      absDenominator = (unsigned int)denominator;
    }
    quotientNeedsNegation = ((numerator < 0) != (denominator < 0));
    tempAbsQuotient = absNumerator / absDenominator;
    if (quotientNeedsNegation) {
      absNumerator %= absDenominator;
      if (absNumerator > 0U) {
        tempAbsQuotient++;
      }
      quotient = (short)-(int)tempAbsQuotient;
    } else {
      quotient = (short)tempAbsQuotient;
    }
  }
  return quotient;
}

/*
 * Arguments    : short N_bat1
 *                short N_bat2
 *                double N_bat
 *                short NbatMax
 *                unsigned char bitCnfg_data[]
 *                int bitCnfg_size[3]
 *                char bitCnfgStr_data[]
 *                int bitCnfgStr_size[2]
 * Return Type  : short
 */
short BuildBitCnfg(short N_bat1, short N_bat2, double N_bat, short NbatMax,
                   unsigned char bitCnfg_data[], int bitCnfg_size[3],
                   char bitCnfgStr_data[], int bitCnfgStr_size[2])
{
  /*static*/ short b_tmp_data[32767];
  /*static*/ short tmp_data[32767];
  emxArray_char_T *b_str;
  emxArray_char_T *c_str;
  emxArray_char_T *d_str;
  emxArray_char_T *str;
  double d;
  int b_k_bat2;
  int i;
  int i1;
  int i3;
  int i4;
  int k;
  int k_cnfg;
  int nbytes;
  short iv[2];
  short N_bitCnfg;
  short i2;
  short i5;
  short k_bat1;
  short k_bat2;
  short yk;
  unsigned char u;
  char *str_data;
  i = N_bat2 * N_bat1;
  i1 = i;
  if (i > 32767) {
    i1 = 32767;
  } else if (i < -32768) {
    i1 = -32768;
  }
  d = rt_roundd_snf(N_bat + (double)i1);
  if (d < 32768.0) {
    if (d >= -32768.0) {
      i2 = (short)d;
    } else {
      i2 = MIN_int16_T;
    }
  } else if (d >= 32768.0) {
    i2 = MAX_int16_T;
  } else {
    i2 = 0;
  }
  if (i > 32767) {
    i = 32767;
  } else if (i < -32768) {
    i = -32768;
  }
  i += i2;
  if (i > 32767) {
    i = 32767;
  } else if (i < -32768) {
    i = -32768;
  }
  i += N_bat2;
  if (i > 32767) {
    i = 32767;
  } else if (i < -32768) {
    i = -32768;
  }
  i1 = i - 1;
  if (i - 1 < -32768) {
    i1 = -32768;
  }
  N_bitCnfg = (short)i1;
  bitCnfg_size[0] = (short)i1;
  bitCnfg_size[1] = NbatMax;
  bitCnfg_size[2] = NbatMax;
  nbytes = (short)i1 * NbatMax * NbatMax;
  if (nbytes - 1 >= 0) {
    memset(&bitCnfg_data[0], 0, (unsigned int)nbytes * sizeof(unsigned char));
  }
  /* {}; */
  bitCnfgStr_size[0] = (short)i1;
  bitCnfgStr_size[1] = 10;
  nbytes = (short)i1 * 10;
  for (i = 0; i < nbytes; i++) {
    bitCnfgStr_data[i] = ' ';
  }
  /* {}; */
  /* test A, D */
  i = (int)N_bat;
  emxInit_char_T(&str);
  for (k_cnfg = 0; k_cnfg < i; k_cnfg++) {
    if ((unsigned int)k_cnfg + 1U < 256U) {
      u = (unsigned char)((double)k_cnfg + 1.0);
    } else {
      u = MAX_uint8_T;
    }
    bitCnfg_data[k_cnfg] = u;
    /* bitCnfg{k_cnfg} = k_bat; */
    if ((unsigned int)k_cnfg + 1U < 2147483648U) {
      k = k_cnfg + 1;
      i3 = k;
    } else {
      k = MAX_int32_T;
      i3 = MAX_int32_T;
    }
    nbytes = (int)snprintf(NULL, 0, "A%02d or D%02d", k, i3) + 1;
    i4 = str->size[0] * str->size[1];
    str->size[0] = 1;
    str->size[1] = nbytes;
    emxEnsureCapacity_char_T(str, i4);
    str_data = str->data;
    snprintf(&str_data[0], (size_t)nbytes, "A%02d or D%02d", k, i3);
    for (i3 = 0; i3 < 10; i3++) {
      bitCnfgStr_data[k_cnfg + (short)i1 * i3] = str_data[i3];
    }
    /* ['A',sprintf('%d',int32(k_bat)),' or D',sprintf('%d',int32(k_bat))];%10
     */
  }
  emxFree_char_T(&str);
  if ((int)N_bat - 1 < 0) {
    k_cnfg = 0;
  } else {
    k_cnfg = (int)N_bat;
  }
  /* test C */
  i = N_bat2 - 1;
  if (N_bat2 - 1 < -32768) {
    i = -32768;
  }
  i2 = (short)i;
  emxInit_char_T(&b_str);
  for (k_bat2 = 0; k_bat2 <= i2; k_bat2++) {
    i = N_bat1 - 1;
    if (N_bat1 - 1 < -32768) {
      i = -32768;
    }
    i5 = (short)i;
    for (k_bat1 = 0; k_bat1 <= i5; k_bat1++) {
      if (N_bat1 == 0) {
        yk = (short)(k_bat1 + 1);
      } else {
        yk = (short)((k_bat1 -
                      (short)(div_s16s32_floor(k_bat1 + 1, N_bat1) * N_bat1)) +
                     1);
      }
      i = k_bat2 * N_bat1;
      i3 = i;
      if (i > 32767) {
        i3 = 32767;
      } else if (i < -32768) {
        i3 = -32768;
      }
      i3 += k_bat1;
      if (i3 > 32767) {
        i3 = 32767;
      }
      k = i3 + 1;
      if (i3 + 1 > 32767) {
        k = 32767;
      }
      iv[0] = (short)k;
      i3 = i;
      if (i > 32767) {
        i3 = 32767;
      } else if (i < -32768) {
        i3 = -32768;
      }
      i3 += yk;
      if (i3 > 32767) {
        i3 = 32767;
      } else if (i3 < -32768) {
        i3 = -32768;
      }
      k = i3 + 1;
      if (i3 + 1 > 32767) {
        k = 32767;
      }
      iv[1] = (short)k;
      sort(iv);
      yk = iv[0];
      if (iv[0] < 0) {
        yk = 0;
      } else if (iv[0] > 255) {
        yk = 255;
      }
      bitCnfg_data[k_cnfg] = (unsigned char)yk;
      yk = iv[1];
      if (iv[1] < 0) {
        yk = 0;
      } else if (iv[1] > 255) {
        yk = 255;
      }
      bitCnfg_data[k_cnfg + (short)i1] = (unsigned char)yk;
      if (N_bat1 == 0) {
        yk = (short)(k_bat1 + 1);
      } else {
        yk = (short)((k_bat1 -
                      (short)(div_s16s32_floor(k_bat1 + 1, N_bat1) * N_bat1)) +
                     1);
      }
      if (i > 32767) {
        i = 32767;
      } else if (i < -32768) {
        i = -32768;
      }
      i += yk;
      if (i > 32767) {
        i = 32767;
      } else if (i < -32768) {
        i = -32768;
      }
      i3 = i + 1;
      if (i + 1 > 32767) {
        i3 = 32767;
      }
      nbytes = (int)snprintf(NULL, 0, "C%02d", (int)(short)i3) + 1;
      i = b_str->size[0] * b_str->size[1];
      b_str->size[0] = 1;
      b_str->size[1] = nbytes;
      emxEnsureCapacity_char_T(b_str, i);
      str_data = b_str->data;
      snprintf(&str_data[0], (size_t)nbytes, "C%02d", (int)(short)i3);
      bitCnfgStr_data[k_cnfg] = str_data[0];
      bitCnfgStr_data[k_cnfg + (short)i1] = str_data[1];
      bitCnfgStr_data[k_cnfg + (short)i1 * 2] = str_data[2];
      /* (2+sum(double((mod(k_bat1+1,N_bat1)+k_bat2*N_bat1+1)>=10)))) =
       * ['C',sprintf('%d',int32(mod(k_bat1+1,N_bat1)+k_bat2*N_bat1+1))]; */
      k_cnfg++;
    }
  }
  emxFree_char_T(&b_str);
  /* test B */
  i = N_bat2 - 1;
  if (N_bat2 - 1 < -32768) {
    i = -32768;
  }
  i2 = (short)i;
  emxInit_char_T(&c_str);
  for (k_bat2 = 0; k_bat2 <= i2; k_bat2++) {
    i = N_bat1 - 1;
    if (N_bat1 - 1 < -32768) {
      i = -32768;
    }
    i5 = (short)i;
    for (k_bat1 = 0; k_bat1 <= i5; k_bat1++) {
      i = k_bat1 + 2;
      if (k_bat1 + 2 > 32767) {
        i = 32767;
      }
      if (N_bat1 == 0) {
        yk = (short)i;
      } else {
        yk = (short)((short)i -
                     (short)(div_s16s32_floor((short)i, N_bat1) * N_bat1));
      }
      i = k_bat2 * N_bat1;
      i3 = i;
      if (i > 32767) {
        i3 = 32767;
      } else if (i < -32768) {
        i3 = -32768;
      }
      i3 += k_bat1;
      if (i3 > 32767) {
        i3 = 32767;
      }
      k = i3 + 1;
      if (i3 + 1 > 32767) {
        k = 32767;
      }
      iv[0] = (short)k;
      i3 = i;
      if (i > 32767) {
        i3 = 32767;
      } else if (i < -32768) {
        i3 = -32768;
      }
      i3 += yk;
      if (i3 > 32767) {
        i3 = 32767;
      } else if (i3 < -32768) {
        i3 = -32768;
      }
      k = i3 + 1;
      if (i3 + 1 > 32767) {
        k = 32767;
      }
      iv[1] = (short)k;
      sort(iv);
      yk = iv[0];
      if (iv[0] < 0) {
        yk = 0;
      } else if (iv[0] > 255) {
        yk = 255;
      }
      bitCnfg_data[k_cnfg] = (unsigned char)yk;
      yk = iv[1];
      if (iv[1] < 0) {
        yk = 0;
      } else if (iv[1] > 255) {
        yk = 255;
      }
      bitCnfg_data[k_cnfg + (short)i1] = (unsigned char)yk;
      if (N_bat1 == 0) {
        yk = (short)(k_bat1 + 1);
      } else {
        yk = (short)((k_bat1 -
                      (short)(div_s16s32_floor(k_bat1 + 1, N_bat1) * N_bat1)) +
                     1);
      }
      if (i > 32767) {
        i = 32767;
      } else if (i < -32768) {
        i = -32768;
      }
      i += yk;
      if (i > 32767) {
        i = 32767;
      } else if (i < -32768) {
        i = -32768;
      }
      i3 = i + 1;
      if (i + 1 > 32767) {
        i3 = 32767;
      }
      nbytes = (int)snprintf(NULL, 0, "B%02d", (int)(short)i3) + 1;
      i = c_str->size[0] * c_str->size[1];
      c_str->size[0] = 1;
      c_str->size[1] = nbytes;
      emxEnsureCapacity_char_T(c_str, i);
      str_data = c_str->data;
      snprintf(&str_data[0], (size_t)nbytes, "B%02d", (int)(short)i3);
      bitCnfgStr_data[k_cnfg] = str_data[0];
      bitCnfgStr_data[k_cnfg + (short)i1] = str_data[1];
      bitCnfgStr_data[k_cnfg + (short)i1 * 2] = str_data[2];
      /* (2+sum(double((mod(k_bat1+1,N_bat1)+k_bat2*N_bat1+1)>=10)))) =
       * ['B',sprintf('%d',int32(mod(k_bat1+1,N_bat1)+k_bat2*N_bat1+1))]; */
      k_cnfg++;
    }
  }
  emxFree_char_T(&c_str);
  /* COM5 */
  i = N_bat2 - 1;
  if (N_bat2 - 1 < -32768) {
    i = -32768;
  }
  emxInit_char_T(&d_str);
  for (b_k_bat2 = 0; b_k_bat2 < i; b_k_bat2++) {
    i3 = N_bat2 - 1;
    if (N_bat2 - 1 < -32768) {
      i3 = -32768;
    }
    if ((short)i3 < 0) {
      nbytes = 0;
    } else {
      nbytes = (short)i3 + 1;
    }
    if (nbytes > 0) {
      tmp_data[0] = 0;
      yk = 0;
      for (k = 2; k <= nbytes; k++) {
        yk++;
        tmp_data[k - 1] = yk;
      }
    }
    for (i3 = 0; i3 < nbytes; i3++) {
      k = tmp_data[i3] * N_bat1;
      if (k > 32767) {
        k = 32767;
      } else if (k < -32768) {
        k = -32768;
      }
      i4 = k + 1;
      if (k + 1 > 32767) {
        i4 = 32767;
      }
      b_tmp_data[i3] = (short)i4;
    }
    b_sort(b_tmp_data, &nbytes);
    i2 = b_tmp_data[0];
    if (b_tmp_data[0] < 0) {
      i2 = 0;
    } else if (b_tmp_data[0] > 255) {
      i2 = 255;
    }
    bitCnfg_data[k_cnfg] = (unsigned char)i2;
    i2 = b_tmp_data[1];
    if (b_tmp_data[1] < 0) {
      i2 = 0;
    } else if (b_tmp_data[1] > 255) {
      i2 = 255;
    }
    bitCnfg_data[k_cnfg + (short)i1] = (unsigned char)i2;
    nbytes = (int)snprintf(NULL, 0, "COM5 %02d", b_k_bat2 + 1) + 1;
    i3 = d_str->size[0] * d_str->size[1];
    d_str->size[0] = 1;
    d_str->size[1] = nbytes;
    emxEnsureCapacity_char_T(d_str, i3);
    str_data = d_str->data;
    snprintf(&str_data[0], (size_t)nbytes, "COM5 %02d", b_k_bat2 + 1);
    for (i3 = 0; i3 < 7; i3++) {
      bitCnfgStr_data[k_cnfg + (short)i1 * i3] = str_data[i3];
    }
    /* (6+sum(double(k_bat2+1)>=10))) = ['COM5 ',sprintf('%d',int32(k_bat2+1))];
     */
    k_cnfg++;
  }
  emxFree_char_T(&d_str);
  return N_bitCnfg;
}

/*
 * File trailer for BuildBitCnfg.c
 *
 * [EOF]
 */
