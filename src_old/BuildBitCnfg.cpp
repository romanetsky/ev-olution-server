/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: BuildBitCnfg.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 17-Mar-2024 11:09:30
 */

/* Include Files */
#include "BuildBitCnfg.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_emxutil.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_rtwutil.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h"
#include "rt_nonfinite.h"
#include "sort.h"
#include <stdio.h>

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
 *                emxArray_uint8_T *bitCnfg
 *                char bitCnfgStr[2]
 * Return Type  : short
 */
short BuildBitCnfg(short N_bat1, short N_bat2, double N_bat, short NbatMax,
                   emxArray_uint8_T *bitCnfg, char bitCnfgStr[2])
{
  static short b_tmp_data[32767];
  static short tmp_data[32767];
  double d;
  int i;
  int i1;
  int i4;
  int loop_ub;
  int n;
  short iv[2];
  short i2;
  short i3;
  short k_bat1;
  short k_bat2;
  short yk;
  unsigned char *bitCnfg_data;
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
  i = bitCnfg->size[0] * bitCnfg->size[1] * bitCnfg->size[2];
  bitCnfg->size[0] = (short)i1;
  bitCnfg->size[1] = NbatMax;
  bitCnfg->size[2] = NbatMax;
  emxEnsureCapacity_uint8_T(bitCnfg, i);
  bitCnfg_data = bitCnfg->data;
  loop_ub = (short)i1 * NbatMax * NbatMax;
  for (i = 0; i < loop_ub; i++) {
    bitCnfg_data[i] = 0U;
  }
  /* {}; */
  bitCnfgStr[0] = (signed char)i1;
  bitCnfgStr[1] = '\n';
  /* {}; */
  /* test A, D */
  /* test C */
  i = N_bat2 - 1;
  if (N_bat2 - 1 < -32768) {
    i = -32768;
  }
  i2 = (short)i;
  for (k_bat2 = 0; k_bat2 <= i2; k_bat2++) {
    i = N_bat1 - 1;
    if (N_bat1 - 1 < -32768) {
      i = -32768;
    }
    i3 = (short)i;
    for (k_bat1 = 0; k_bat1 <= i3; k_bat1++) {
      if (N_bat1 == 0) {
        yk = (short)(k_bat1 + 1);
      } else {
        yk = (short)((k_bat1 -
                      (short)(div_s16s32_floor(k_bat1 + 1, N_bat1) * N_bat1)) +
                     1);
      }
      i = k_bat2 * N_bat1;
      loop_ub = i;
      if (i > 32767) {
        loop_ub = 32767;
      } else if (i < -32768) {
        loop_ub = -32768;
      }
      loop_ub += k_bat1;
      if (loop_ub > 32767) {
        loop_ub = 32767;
      }
      i4 = loop_ub + 1;
      if (loop_ub + 1 > 32767) {
        i4 = 32767;
      }
      iv[0] = (short)i4;
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
      loop_ub = i + 1;
      if (i + 1 > 32767) {
        loop_ub = 32767;
      }
      iv[1] = (short)loop_ub;
      sort(iv);
      /* A check that is always false is detected at compile-time. Eliminating
       * code that follows. */
    }
  }
  /* test B */
  i = N_bat2 - 1;
  if (N_bat2 - 1 < -32768) {
    i = -32768;
  }
  i2 = (short)i;
  for (k_bat2 = 0; k_bat2 <= i2; k_bat2++) {
    i = N_bat1 - 1;
    if (N_bat1 - 1 < -32768) {
      i = -32768;
    }
    i3 = (short)i;
    for (k_bat1 = 0; k_bat1 <= i3; k_bat1++) {
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
      loop_ub = i;
      if (i > 32767) {
        loop_ub = 32767;
      } else if (i < -32768) {
        loop_ub = -32768;
      }
      loop_ub += k_bat1;
      if (loop_ub > 32767) {
        loop_ub = 32767;
      }
      i4 = loop_ub + 1;
      if (loop_ub + 1 > 32767) {
        i4 = 32767;
      }
      iv[0] = (short)i4;
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
      loop_ub = i + 1;
      if (i + 1 > 32767) {
        loop_ub = 32767;
      }
      iv[1] = (short)loop_ub;
      sort(iv);
      /* A check that is always false is detected at compile-time. Eliminating
       * code that follows. */
    }
  }
  /* COM5 */
  i = N_bat2 - 1;
  if (N_bat2 - 1 < -32768) {
    i = -32768;
  }
  i2 = (short)i;
  for (k_bat2 = 0; k_bat2 <= i2; k_bat2++) {
    i = N_bat2 - 1;
    if (N_bat2 - 1 < -32768) {
      i = -32768;
    }
    if ((short)i < 0) {
      n = 0;
    } else {
      n = (short)i + 1;
    }
    if (n > 0) {
      tmp_data[0] = 0;
      yk = 0;
      for (loop_ub = 2; loop_ub <= n; loop_ub++) {
        yk++;
        tmp_data[loop_ub - 1] = yk;
      }
    }
    for (i = 0; i < n; i++) {
      loop_ub = tmp_data[i] * N_bat1;
      if (loop_ub > 32767) {
        loop_ub = 32767;
      } else if (loop_ub < -32768) {
        loop_ub = -32768;
      }
      i4 = loop_ub + 1;
      if (loop_ub + 1 > 32767) {
        i4 = 32767;
      }
      b_tmp_data[i] = (short)i4;
    }
    b_sort(b_tmp_data, &n);
    /* A check that is always false is detected at compile-time. Eliminating
     * code that follows. */
  }
  return (short)i1;
}

/*
 * File trailer for BuildBitCnfg.c
 *
 * [EOF]
 */
