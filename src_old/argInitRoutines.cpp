#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_emxAPI.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_terminate.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Declarations */
static void argInit_16x16_uint8_T(unsigned char result[256]);

static void argInit_16x16x8x16_uint8_T(unsigned char result[32768]);

static void argInit_16x31_real32_T(float result[496]);

static void argInit_16x38_real32_T(float result[608]);

static void argInit_1x1024_real32_T(float result[1024]);

static void argInit_1x16_boolean_T(boolean_T result[16]);

static void argInit_1x16_int8_T(signed char result[16]);

static void argInit_1x16_real32_T(float result[16]);

static void argInit_1x256_char_T(char result[256]);

static void argInit_1x2_real32_T(float result[2]);

static void argInit_1x2_real_T(double result[2]);

static void argInit_1x32_real32_T(float result[32]);

static void argInit_1x6_char_T(char result[6]);

static void argInit_257x2x4x6_uint8_T(unsigned char result[12336]);

static void argInit_257x2x8_uint8_T(unsigned char result[4112]);

static void argInit_2x16_int8_T(signed char result[32]);

static void argInit_2x16x2_real32_T(float result[64]);

static void argInit_2x1_int16_T(short result[2]);

static void argInit_2x1x2_real32_T(float result[4]);

static void argInit_2x256_char_T(char result[512]);

static void argInit_2x2_struct21_T(struct21_T result[4]);

static void argInit_2x2_struct22_T(struct22_T result[4]);

static void argInit_2x6_char_T(char result[12]);

static void argInit_32x1_int16_T(short result[32]);

static void argInit_32x256_char_T(char result[8192]);

static void argInit_6x1x2x2_uint8_T(unsigned char result[24]);

static void argInit_6x2x4x6_uint8_T(unsigned char result[288]);

static void argInit_8x16_int8_T(signed char result[128]);

static void argInit_8x16_real32_T(float result[128]);

static void argInit_8x1_int16_T(short result[8]);

static void argInit_8x2_real_T(double result[16]);

static boolean_T argInit_boolean_T(void);

static char argInit_char_T(void);

static short argInit_int16_T(void);

static int argInit_int32_T(void);

static signed char argInit_int8_T(void);

static float argInit_real32_T(void);

static double argInit_real_T(void);

static void argInit_struct0_T(struct0_T *result);

static void argInit_struct10_T(struct10_T *result);

static void argInit_struct11_T(struct11_T *result);

static struct12_T argInit_struct12_T(void);

static void argInit_struct13_T(struct13_T *result);

static struct14_T argInit_struct14_T(void);

static struct15_T argInit_struct15_T(void);

static struct16_T argInit_struct16_T(void);

static struct17_T argInit_struct17_T(void);

static struct18_T argInit_struct18_T(void);

static struct19_T argInit_struct19_T(void);

static struct1_T argInit_struct1_T(void);

static void argInit_struct20_T(struct20_T *result);

static void argInit_struct21_T(struct21_T *result);

static void argInit_struct22_T(struct22_T *result);

static void argInit_struct23_T(struct23_T *result);

static void argInit_struct24_T(struct24_T *result);

static void argInit_struct25_T(struct25_T *result);

static void argInit_struct26_T(struct26_T *result);

static void argInit_struct27_T(struct27_T *result);

static void argInit_struct28_T(struct28_T *result);

static struct29_T argInit_struct29_T(void);

static void argInit_struct2_T(struct2_T *result);

static void argInit_struct3_T(struct3_T *result);

static struct4_T argInit_struct4_T(void);

static void argInit_struct5_T(struct5_T *result);

static void argInit_struct6_T(struct6_T *result);

static void argInit_struct7_T(struct7_T *result);

static void argInit_struct8_T(struct8_T *result);

static void argInit_struct9_T(struct9_T *result);

static unsigned char argInit_uint8_T(void);

/* Function Definitions */
/*
 * Arguments    : unsigned char result[256]
 * Return Type  : void
 */
static void argInit_16x16_uint8_T(unsigned char result[256])
{
  int i;
  /* Loop over the array to initialize each element. */
  for (i = 0; i < 256; i++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[i] = argInit_uint8_T();
  }
}

/*
 * Arguments    : unsigned char result[32768]
 * Return Type  : void
 */
static void argInit_16x16x8x16_uint8_T(unsigned char result[32768])
{
  int idx0;
  int idx1;
  int idx2;
  int idx3;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 16; idx0++) {
    for (idx1 = 0; idx1 < 16; idx1++) {
      for (idx2 = 0; idx2 < 8; idx2++) {
        for (idx3 = 0; idx3 < 16; idx3++) {
          /* Set the value of the array element.
Change this value to the value that the application requires. */
          result[((idx0 + (idx1 << 4)) + (idx2 << 8)) + (idx3 << 11)] =
              argInit_uint8_T();
        }
      }
    }
  }
}

/*
 * Arguments    : float result[496]
 * Return Type  : void
 */
static void argInit_16x31_real32_T(float result[496])
{
  int i;
  /* Loop over the array to initialize each element. */
  for (i = 0; i < 496; i++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[i] = argInit_real32_T();
  }
}

/*
 * Arguments    : float result[608]
 * Return Type  : void
 */
static void argInit_16x38_real32_T(float result[608])
{
  int i;
  /* Loop over the array to initialize each element. */
  for (i = 0; i < 608; i++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[i] = argInit_real32_T();
  }
}

/*
 * Arguments    : float result[1024]
 * Return Type  : void
 */
static void argInit_1x1024_real32_T(float result[1024])
{
  int idx1;
  /* Loop over the array to initialize each element. */
  for (idx1 = 0; idx1 < 1024; idx1++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx1] = argInit_real32_T();
  }
}

/*
 * Arguments    : boolean_T result[16]
 * Return Type  : void
 */
static void argInit_1x16_boolean_T(boolean_T result[16])
{
  int idx1;
  /* Loop over the array to initialize each element. */
  for (idx1 = 0; idx1 < 16; idx1++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx1] = argInit_boolean_T();
  }
}

/*
 * Arguments    : signed char result[16]
 * Return Type  : void
 */
static void argInit_1x16_int8_T(signed char result[16])
{
  int idx1;
  /* Loop over the array to initialize each element. */
  for (idx1 = 0; idx1 < 16; idx1++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx1] = argInit_int8_T();
  }
}

/*
 * Arguments    : float result[16]
 * Return Type  : void
 */
static void argInit_1x16_real32_T(float result[16])
{
  int idx1;
  /* Loop over the array to initialize each element. */
  for (idx1 = 0; idx1 < 16; idx1++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx1] = argInit_real32_T();
  }
}

/*
 * Arguments    : char result[256]
 * Return Type  : void
 */
static void argInit_1x256_char_T(char result[256])
{
  int idx1;
  /* Loop over the array to initialize each element. */
  for (idx1 = 0; idx1 < 256; idx1++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx1] = argInit_char_T();
  }
}

/*
 * Arguments    : float result[2]
 * Return Type  : void
 */
static void argInit_1x2_real32_T(float result[2])
{
  int idx1;
  /* Loop over the array to initialize each element. */
  for (idx1 = 0; idx1 < 2; idx1++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx1] = argInit_real32_T();
  }
}

/*
 * Arguments    : double result[2]
 * Return Type  : void
 */
static void argInit_1x2_real_T(double result[2])
{
  int idx1;
  /* Loop over the array to initialize each element. */
  for (idx1 = 0; idx1 < 2; idx1++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx1] = argInit_real_T();
  }
}

/*
 * Arguments    : float result[32]
 * Return Type  : void
 */
static void argInit_1x32_real32_T(float result[32])
{
  int idx1;
  /* Loop over the array to initialize each element. */
  for (idx1 = 0; idx1 < 32; idx1++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx1] = argInit_real32_T();
  }
}

/*
 * Arguments    : char result[6]
 * Return Type  : void
 */
static void argInit_1x6_char_T(char result[6])
{
  int idx1;
  /* Loop over the array to initialize each element. */
  for (idx1 = 0; idx1 < 6; idx1++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx1] = argInit_char_T();
  }
}

/*
 * Arguments    : unsigned char result[12336]
 * Return Type  : void
 */
static void argInit_257x2x4x6_uint8_T(unsigned char result[12336])
{
  int idx0;
  int idx1;
  int idx2;
  int idx3;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 257; idx0++) {
    for (idx1 = 0; idx1 < 2; idx1++) {
      for (idx2 = 0; idx2 < 4; idx2++) {
        for (idx3 = 0; idx3 < 6; idx3++) {
          /* Set the value of the array element.
Change this value to the value that the application requires. */
          result[((idx0 + 257 * idx1) + 514 * idx2) + 2056 * idx3] =
              argInit_uint8_T();
        }
      }
    }
  }
}

/*
 * Arguments    : unsigned char result[4112]
 * Return Type  : void
 */
static void argInit_257x2x8_uint8_T(unsigned char result[4112])
{
  int idx0;
  int idx1;
  int idx2;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 257; idx0++) {
    for (idx1 = 0; idx1 < 2; idx1++) {
      for (idx2 = 0; idx2 < 8; idx2++) {
        /* Set the value of the array element.
Change this value to the value that the application requires. */
        result[(idx0 + 257 * idx1) + 514 * idx2] = argInit_uint8_T();
      }
    }
  }
}

/*
 * Arguments    : signed char result[32]
 * Return Type  : void
 */
static void argInit_2x16_int8_T(signed char result[32])
{
  int i;
  /* Loop over the array to initialize each element. */
  for (i = 0; i < 32; i++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[i] = argInit_int8_T();
  }
}

/*
 * Arguments    : float result[64]
 * Return Type  : void
 */
static void argInit_2x16x2_real32_T(float result[64])
{
  int idx0;
  int idx1;
  int idx2;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 2; idx0++) {
    for (idx1 = 0; idx1 < 16; idx1++) {
      for (idx2 = 0; idx2 < 2; idx2++) {
        /* Set the value of the array element.
Change this value to the value that the application requires. */
        result[(idx0 + (idx1 << 1)) + (idx2 << 5)] = argInit_real32_T();
      }
    }
  }
}

/*
 * Arguments    : short result[2]
 * Return Type  : void
 */
static void argInit_2x1_int16_T(short result[2])
{
  int idx0;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 2; idx0++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx0] = argInit_int16_T();
  }
}

/*
 * Arguments    : float result[4]
 * Return Type  : void
 */
static void argInit_2x1x2_real32_T(float result[4])
{
  int i;
  /* Loop over the array to initialize each element. */
  for (i = 0; i < 4; i++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[i] = argInit_real32_T();
  }
}

/*
 * Arguments    : char result[512]
 * Return Type  : void
 */
static void argInit_2x256_char_T(char result[512])
{
  int i;
  /* Loop over the array to initialize each element. */
  for (i = 0; i < 512; i++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[i] = argInit_char_T();
  }
}

/*
 * Arguments    : struct21_T result[4]
 * Return Type  : void
 */
static void argInit_2x2_struct21_T(struct21_T result[4])
{
  int i;
  /* Loop over the array to initialize each element. */
  for (i = 0; i < 4; i++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    argInit_struct21_T(&result[i]);
  }
}

/*
 * Arguments    : struct22_T result[4]
 * Return Type  : void
 */
static void argInit_2x2_struct22_T(struct22_T result[4])
{
  int i;
  /* Loop over the array to initialize each element. */
  for (i = 0; i < 4; i++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    argInit_struct22_T(&result[i]);
  }
}

/*
 * Arguments    : char result[12]
 * Return Type  : void
 */
static void argInit_2x6_char_T(char result[12])
{
  int i;
  /* Loop over the array to initialize each element. */
  for (i = 0; i < 12; i++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[i] = argInit_char_T();
  }
}

/*
 * Arguments    : short result[32]
 * Return Type  : void
 */
static void argInit_32x1_int16_T(short result[32])
{
  int idx0;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 32; idx0++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx0] = argInit_int16_T();
  }
}

/*
 * Arguments    : char result[8192]
 * Return Type  : void
 */
static void argInit_32x256_char_T(char result[8192])
{
  int i;
  /* Loop over the array to initialize each element. */
  for (i = 0; i < 8192; i++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[i] = argInit_char_T();
  }
}

/*
 * Arguments    : unsigned char result[24]
 * Return Type  : void
 */
static void argInit_6x1x2x2_uint8_T(unsigned char result[24])
{
  int idx0;
  int idx2;
  int idx3;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 6; idx0++) {
    for (idx2 = 0; idx2 < 2; idx2++) {
      for (idx3 = 0; idx3 < 2; idx3++) {
        /* Set the value of the array element.
Change this value to the value that the application requires. */
        result[(idx0 + 6 * idx2) + 12 * idx3] = argInit_uint8_T();
      }
    }
  }
}

/*
 * Arguments    : unsigned char result[288]
 * Return Type  : void
 */
static void argInit_6x2x4x6_uint8_T(unsigned char result[288])
{
  int idx0;
  int idx1;
  int idx2;
  int idx3;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 6; idx0++) {
    for (idx1 = 0; idx1 < 2; idx1++) {
      for (idx2 = 0; idx2 < 4; idx2++) {
        for (idx3 = 0; idx3 < 6; idx3++) {
          /* Set the value of the array element.
Change this value to the value that the application requires. */
          result[((idx0 + 6 * idx1) + 12 * idx2) + 48 * idx3] =
              argInit_uint8_T();
        }
      }
    }
  }
}

/*
 * Arguments    : signed char result[128]
 * Return Type  : void
 */
static void argInit_8x16_int8_T(signed char result[128])
{
  int i;
  /* Loop over the array to initialize each element. */
  for (i = 0; i < 128; i++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[i] = argInit_int8_T();
  }
}

/*
 * Arguments    : float result[128]
 * Return Type  : void
 */
static void argInit_8x16_real32_T(float result[128])
{
  int i;
  /* Loop over the array to initialize each element. */
  for (i = 0; i < 128; i++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[i] = argInit_real32_T();
  }
}

/*
 * Arguments    : short result[8]
 * Return Type  : void
 */
static void argInit_8x1_int16_T(short result[8])
{
  int idx0;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 8; idx0++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx0] = argInit_int16_T();
  }
}

/*
 * Arguments    : double result[16]
 * Return Type  : void
 */
static void argInit_8x2_real_T(double result[16])
{
  int i;
  /* Loop over the array to initialize each element. */
  for (i = 0; i < 16; i++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[i] = argInit_real_T();
  }
}

/*
 * Arguments    : void
 * Return Type  : boolean_T
 */
static boolean_T argInit_boolean_T(void)
{
  return false;
}

/*
 * Arguments    : void
 * Return Type  : char
 */
static char argInit_char_T(void)
{
  return '?';
}

/*
 * Arguments    : void
 * Return Type  : short
 */
static short argInit_int16_T(void)
{
  return 0;
}

/*
 * Arguments    : void
 * Return Type  : int
 */
static int argInit_int32_T(void)
{
  return 0;
}

/*
 * Arguments    : void
 * Return Type  : signed char
 */
static signed char argInit_int8_T(void)
{
  return 0;
}

/*
 * Arguments    : void
 * Return Type  : float
 */
static float argInit_real32_T(void)
{
  return 0.0F;
}

/*
 * Arguments    : void
 * Return Type  : double
 */
static double argInit_real_T(void)
{
  return 0.0;
}

/*
 * Arguments    : struct0_T *result
 * Return Type  : void
 */
static void argInit_struct0_T(struct0_T *result)
{
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  result->Nmax = argInit_struct1_T();
  argInit_struct2_T(&result->run);
  argInit_struct3_T(&result->seq);
  argInit_struct10_T(&result->ser);
  argInit_struct13_T(&result->str);
  result->ins = argInit_struct15_T();
  argInit_struct20_T(&result->klm);
  argInit_struct23_T(&result->bat);
  argInit_struct24_T(&result->brd);
  argInit_struct27_T(&result->cnfg);
  argInit_struct28_T(&result->files);
}

/*
 * Arguments    : struct10_T *result
 * Return Type  : void
 */
static void argInit_struct10_T(struct10_T *result)
{
  boolean_T result_tmp;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  result_tmp = argInit_boolean_T();
  result->ka6005p_Flag = result_tmp;
  result->juntek_Flag = result_tmp;
  result->swm_Flag = result_tmp;
  result->sw16to1_Flag = result_tmp;
  result->swOut_Flag = result_tmp;
  argInit_struct11_T(&result->com);
  result->wifi = argInit_struct12_T();
  result->kp184_Flag = result_tmp;
  argInit_1x2_real_T(result->Esp32_v1);
}

/*
 * Arguments    : struct11_T *result
 * Return Type  : void
 */
static void argInit_struct11_T(struct11_T *result)
{
  int i;
  short result_tmp;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  result_tmp = argInit_int16_T();
  result->Ngrp = result_tmp;
  result->Nbrd = result_tmp;
  result->NuGrp = result_tmp;
  argInit_1x6_char_T(result->COM_kp184);
  argInit_2x6_char_T(result->COM_esp32);
  result->N_COM_esp32 = result_tmp;
  argInit_2x1_int16_T(result->grp);
  argInit_8x1_int16_T(result->uGroups);
  for (i = 0; i < 6; i++) {
    result->COM_ka6005P[i] = result->COM_kp184[i];
    result->COM_juntek[i] = result->COM_kp184[i];
    result->COM_swm[i] = result->COM_kp184[i];
    result->COM_sw16to1[i] = result->COM_kp184[i];
    result->COM_swOut[i] = result->COM_kp184[i];
  }
}

/*
 * Arguments    : void
 * Return Type  : struct12_T
 */
static struct12_T argInit_struct12_T(void)
{
  struct12_T result;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  result.ip = argInit_int16_T();
  return result;
}

/*
 * Arguments    : struct13_T *result
 * Return Type  : void
 */
static void argInit_struct13_T(struct13_T *result)
{
  int i;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  argInit_32x256_char_T(result->mod);
  result->num = argInit_struct14_T();
  for (i = 0; i < 8192; i++) {
    result->seq[i] = result->mod[i];
    result->ins[i] = result->mod[i];
    result->sw[i] = result->mod[i];
    result->sw16to1[i] = result->mod[i];
  }
}

/*
 * Arguments    : void
 * Return Type  : struct14_T
 */
static struct14_T argInit_struct14_T(void)
{
  struct14_T result;
  signed char result_tmp;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  result_tmp = argInit_int8_T();
  result.seq = result_tmp;
  result.ins = result_tmp;
  result.sw = result_tmp;
  result.sw16to1 = result_tmp;
  result.mod = result_tmp;
  return result;
}

/*
 * Arguments    : void
 * Return Type  : struct15_T
 */
static struct15_T argInit_struct15_T(void)
{
  struct15_T result;
  boolean_T result_tmp;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  result_tmp = argInit_boolean_T();
  result.ka6005p = result_tmp;
  result.kp184 = result_tmp;
  result.swm = result_tmp;
  result.sw16to1 = result_tmp;
  result.swOut = result_tmp;
  result.juntek = result_tmp;
  result.prm = argInit_struct16_T();
  result.ProjectFlag = argInit_int8_T();
  return result;
}

/*
 * Arguments    : void
 * Return Type  : struct16_T
 */
static struct16_T argInit_struct16_T(void)
{
  struct16_T result;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  result.jun = argInit_struct17_T();
  result.sw16to1 = argInit_struct18_T();
  result.swOut = argInit_struct19_T();
  return result;
}

/*
 * Arguments    : void
 * Return Type  : struct17_T
 */
static struct17_T argInit_struct17_T(void)
{
  struct17_T result;
  float result_tmp;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  result_tmp = argInit_real32_T();
  result.minVjuntekInput = result_tmp;
  result.juntekEfficencyFactor = result_tmp;
  result.baudrate = result_tmp;
  result.ImaxAcDC = result_tmp;
  return result;
}

/*
 * Arguments    : void
 * Return Type  : struct18_T
 */
static struct18_T argInit_struct18_T(void)
{
  struct18_T result;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  result.Ntry = argInit_int8_T();
  return result;
}

/*
 * Arguments    : void
 * Return Type  : struct19_T
 */
static struct19_T argInit_struct19_T(void)
{
  struct19_T result;
  float result_tmp;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  result_tmp = argInit_real32_T();
  result.Vratio = result_tmp;
  result.Rshunt = result_tmp;
  result.IfromVdivR_flag = argInit_boolean_T();
  return result;
}

/*
 * Arguments    : void
 * Return Type  : struct1_T
 */
static struct1_T argInit_struct1_T(void)
{
  struct1_T result;
  short result_tmp;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  result_tmp = argInit_int16_T();
  result.NbrdMax = result_tmp;
  result.NgrpMax = result_tmp;
  result.NseqMax = result_tmp;
  result.NstateMax = result_tmp;
  result.NitstMax = result_tmp;
  result.NbatMax = result_tmp;
  result.NstrRowMax = result_tmp;
  result.NstrColMax = result_tmp;
  result.NtimeMax = result_tmp;
  result.NkalmanBatState = result_tmp;
  result.NkalmanBatParams = result_tmp;
  result.NswRepMax = result_tmp;
  return result;
}

/*
 * Arguments    : struct20_T *result
 * Return Type  : void
 */
static void argInit_struct20_T(struct20_T *result)
{
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  result->kalmanFlag = argInit_boolean_T();
  result->Ta = argInit_real32_T();
  argInit_2x2_struct21_T(result->BatParamsCell);
  argInit_2x2_struct22_T(result->b_struct);
}

/*
 * Arguments    : struct21_T *result
 * Return Type  : void
 */
static void argInit_struct21_T(struct21_T *result)
{
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  argInit_16x31_real32_T(result->BatState);
  argInit_16x38_real32_T(result->BatParams);
  memcpy(&result->BatStateOrg[0], &result->BatState[0], 496U * sizeof(float));
}

/*
 * Arguments    : struct22_T *result
 * Return Type  : void
 */
static void argInit_struct22_T(struct22_T *result)
{
  float result_tmp;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  result_tmp = argInit_real32_T();
  result->Qkalman = result_tmp;
  result->Rkalman = result_tmp;
  result->eps1 = result_tmp;
  argInit_1x16_real32_T(result->P_k_k);
  argInit_16x31_real32_T(result->BatState_k_k);
  argInit_16x38_real32_T(result->BatParams);
  result->N_bat = argInit_int16_T();
}

/*
 * Arguments    : struct23_T *result
 * Return Type  : void
 */
static void argInit_struct23_T(struct23_T *result)
{
  float result_tmp;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  result_tmp = argInit_real32_T();
  result->ImaxDis = result_tmp;
  result->Icharge = result_tmp;
  result->IchargePhase2 = result_tmp;
  result->IchargePhase3 = result_tmp;
  result->minIphase2 = result_tmp;
  result->dIphase2 = result_tmp;
  argInit_1x16_real32_T(result->CutOffDisV);
  result->Vmin = result_tmp;
  result->Vmax = result_tmp;
  result->VresetMax = result_tmp;
  argInit_1x1024_real32_T(result->t);
  result->T = result_tmp;
  result->Vd = result_tmp;
  argInit_1x32_real32_T(result->Rint);
  memcpy(&result->CutOffChrV[0], &result->CutOffDisV[0], 16U * sizeof(float));
  memcpy(&result->i_in[0], &result->t[0], 1024U * sizeof(float));
}

/*
 * Arguments    : struct24_T *result
 * Return Type  : void
 */
static void argInit_struct24_T(struct24_T *result)
{
  short result_tmp;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  result_tmp = argInit_int16_T();
  result->N_bat2 = result_tmp;
  result->N_bat = result_tmp;
  result->Nbat = result_tmp;
  result->N_bat1 = result_tmp;
  result->Nina219 = argInit_real_T();
  argInit_struct25_T(&result->spi);
  argInit_struct26_T(&result->pac);
  argInit_1x32_real32_T(result->Rwire);
  result->useRwireFlag = argInit_boolean_T();
}

/*
 * Arguments    : struct25_T *result
 * Return Type  : void
 */
static void argInit_struct25_T(struct25_T *result)
{
  int i;
  unsigned char result_tmp;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  result_tmp = argInit_uint8_T();
  result->bypass = result_tmp;
  argInit_257x2x8_uint8_T(result->SwitchMat_esp);
  argInit_6x1x2x2_uint8_T(result->SwitchMat_esp2);
  result->rst = argInit_real_T();
  result->disconnect = result_tmp;
  argInit_257x2x4x6_uint8_T(result->PortSpiRow_esp);
  argInit_6x2x4x6_uint8_T(result->PortSpiRow_esp2);
  memcpy(&result->Pac2Vid[0], &result->SwitchMat_esp[0],
         4112U * sizeof(unsigned char));
  for (i = 0; i < 24; i++) {
    result->Pac2Vid2[i] = result->SwitchMat_esp2[i];
  }
}

/*
 * Arguments    : struct26_T *result
 * Return Type  : void
 */
static void argInit_struct26_T(struct26_T *result)
{
  signed char result_tmp;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  result_tmp = argInit_int8_T();
  result->Iacs758Id = result_tmp;
  argInit_8x2_real_T(result->i2cPacAdd);
  argInit_2x16_int8_T(result->VIpacId);
  result->readIpacFlag = argInit_boolean_T();
  argInit_2x16x2_real32_T(result->Rval);
  argInit_1x2_real32_T(result->Rshunt);
  argInit_2x1x2_real32_T(result->pIacs758);
  result->Iacs758Flag = result_tmp;
}

/*
 * Arguments    : struct27_T *result
 * Return Type  : void
 */
static void argInit_struct27_T(struct27_T *result)
{
  int i;
  short result_tmp;
  unsigned char b_result_tmp;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  result_tmp = argInit_int16_T();
  result->minLenIna219 = result_tmp;
  argInit_16x16_uint8_T(result->BattConfigDis1);
  b_result_tmp = argInit_uint8_T();
  result->BattConfigBypass = b_result_tmp;
  result->ToggleFlag = argInit_boolean_T();
  result->Ttoggle = argInit_real32_T();
  result->NtoggleDrop = result_tmp;
  result->BattConfigStandby = b_result_tmp;
  for (i = 0; i < 256; i++) {
    result->BattConfigChr1[i] = result->BattConfigDis1[i];
    result->BattConfigChr2[i] = result->BattConfigDis1[i];
    result->BattConfigChr3[i] = result->BattConfigDis1[i];
  }
}

/*
 * Arguments    : struct28_T *result
 * Return Type  : void
 */
static void argInit_struct28_T(struct28_T *result)
{
  int i;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  argInit_1x256_char_T(result->saveDir);
  argInit_2x256_char_T(result->savePath_pIacs);
  result->num = argInit_struct29_T();
  for (i = 0; i < 256; i++) {
    result->sufixDir[i] = result->saveDir[i];
    result->prefixFile[i] = result->saveDir[i];
  }
  for (i = 0; i < 512; i++) {
    result->savePath_Rval[i] = result->savePath_pIacs[i];
    result->savePath_Rwire[i] = result->savePath_pIacs[i];
  }
}

/*
 * Arguments    : void
 * Return Type  : struct29_T
 */
static struct29_T argInit_struct29_T(void)
{
  struct29_T result;
  int result_tmp;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  result_tmp = argInit_int32_T();
  result.sufixDir = result_tmp;
  result.prefixFile = result_tmp;
  result.savePath_pIacs = result_tmp;
  result.savePath_Rval = result_tmp;
  result.savePath_Rwire = result_tmp;
  result.saveDir = result_tmp;
  return result;
}

/*
 * Arguments    : struct2_T *result
 * Return Type  : void
 */
static void argInit_struct2_T(struct2_T *result)
{
  float b_result_tmp;
  short c_result_tmp;
  boolean_T result_tmp;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  result_tmp = argInit_boolean_T();
  result->PlotItFlag = result_tmp;
  result->PlotTempFlag = result_tmp;
  result->PlotVFlag = result_tmp;
  result->PlotIFlag = result_tmp;
  result->PlotIacsFlag = result_tmp;
  b_result_tmp = argInit_real32_T();
  result->dt = b_result_tmp;
  c_result_tmp = argInit_int16_T();
  result->T2Show = c_result_tmp;
  result->Nt = c_result_tmp;
  result->Nt0 = c_result_tmp;
  result->testVreset = result_tmp;
  result->seq = c_result_tmp;
  argInit_32x1_int16_T(result->SwRepId);
  result->Nrep = c_result_tmp;
  result->PlotSocFlag = result_tmp;
  result->MaxTime = b_result_tmp;
}

/*
 * Arguments    : struct3_T *result
 * Return Type  : void
 */
static void argInit_struct3_T(struct3_T *result)
{
  int i;
  signed char i1;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  argInit_8x16_int8_T(result->chr);
  argInit_1x16_int8_T(result->mod);
  argInit_1x16_real32_T(result->vth);
  result->Nst = argInit_int8_T();
  argInit_1x16_boolean_T(result->BrdBeforePSflag);
  result->pwr = argInit_struct4_T();
  argInit_struct5_T(&result->tst);
  argInit_struct8_T(&result->bit);
  memcpy(&result->ins[0], &result->chr[0], 128U * sizeof(signed char));
  for (i = 0; i < 16; i++) {
    i1 = result->mod[i];
    result->swm[i] = i1;
    result->sw16to1[i] = i1;
    result->VminDisFlag[i] = result->vth[i];
  }
}

/*
 * Arguments    : void
 * Return Type  : struct4_T
 */
static struct4_T argInit_struct4_T(void)
{
  struct4_T result;
  float result_tmp;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  result_tmp = argInit_real32_T();
  result.VthChr = result_tmp;
  result.VthOvDis = result_tmp;
  result.VthOvChr = result_tmp;
  result.VthUnDis = result_tmp;
  result.VthUnChr = result_tmp;
  result.VthDis = result_tmp;
  argInit_1x16_int8_T(result.VthFlag);
  return result;
}

/*
 * Arguments    : struct5_T *result
 * Return Type  : void
 */
static void argInit_struct5_T(struct5_T *result)
{
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  argInit_struct6_T(&result->v);
  argInit_struct7_T(&result->i);
  result->savePrmFlag = argInit_boolean_T();
}

/*
 * Arguments    : struct6_T *result
 * Return Type  : void
 */
static void argInit_struct6_T(struct6_T *result)
{
  int i;
  boolean_T result_tmp;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  argInit_1x16_int8_T(result->ins);
  result_tmp = argInit_boolean_T();
  result->isPrm = result_tmp;
  result->isTest = result_tmp;
  result->Nst = argInit_int8_T();
  for (i = 0; i < 16; i++) {
    result->swm[i] = result->ins[i];
    result->sw16to1[i] = result->ins[i];
    result->grp[i] = result->ins[i];
  }
}

/*
 * Arguments    : struct7_T *result
 * Return Type  : void
 */
static void argInit_struct7_T(struct7_T *result)
{
  float result_tmp;
  int i;
  signed char b_result_tmp;
  boolean_T c_result_tmp;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  argInit_8x16_int8_T(result->BrdBeforePSflag);
  argInit_1x16_int8_T(result->swm);
  result_tmp = argInit_real32_T();
  result->maxI = result_tmp;
  b_result_tmp = argInit_int8_T();
  result->Nitst = b_result_tmp;
  result->pauseOff = result_tmp;
  c_result_tmp = argInit_boolean_T();
  result->measRintR = c_result_tmp;
  result->useRwireFlag = c_result_tmp;
  result->RintBatId = b_result_tmp;
  result->Rload = result_tmp;
  result->isTest = c_result_tmp;
  argInit_8x16_real32_T(result->Rin);
  result->NTtest = b_result_tmp;
  result->minI = result_tmp;
  argInit_1x32_real32_T(result->i_in_test);
  argInit_16x16x8x16_uint8_T(result->ItestSwitch);
  result->Nst = argInit_uint8_T();
  for (i = 0; i < 128; i++) {
    result->NegIflag[i] = result->BrdBeforePSflag[i];
    result->ins[i] = result->BrdBeforePSflag[i];
    result->meas[i] = result->BrdBeforePSflag[i];
  }
  for (i = 0; i < 16; i++) {
    result->sw16to1[i] = result->swm[i];
  }
  memcpy(&result->grp[0], &result->BrdBeforePSflag[0],
         128U * sizeof(signed char));
}

/*
 * Arguments    : struct8_T *result
 * Return Type  : void
 */
static void argInit_struct8_T(struct8_T *result)
{
  float result_tmp;
  int i;
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  argInit_8x16_int8_T(result->chr);
  argInit_1x16_int8_T(result->BrdBeforePSflag);
  result_tmp = argInit_real32_T();
  result->dVthr = result_tmp;
  result->bit_flag = argInit_boolean_T();
  argInit_8x16_real32_T(result->IdisChr);
  result->Nst = argInit_int8_T();
  argInit_struct9_T(&result->meas);
  result->dIthr = result_tmp;
  memcpy(&result->ins[0], &result->chr[0], 128U * sizeof(signed char));
  for (i = 0; i < 16; i++) {
    result->swm[i] = result->BrdBeforePSflag[i];
    result->sw16to1[i] = result->BrdBeforePSflag[i];
  }
}

/*
 * Arguments    : struct9_T *result
 * Return Type  : void
 */
static void argInit_struct9_T(struct9_T *result)
{
  /* Set the value of each structure field.
Change this value to the value that the application requires. */
  argInit_8x16_int8_T(result->V);
  argInit_8x16_real32_T(result->Vd);
  memcpy(&result->b_I[0], &result->V[0], 128U * sizeof(signed char));
}

/*
 * Arguments    : void
 * Return Type  : unsigned char
 */
static unsigned char argInit_uint8_T(void)
{
  return 0U;
}

/*
 * File trailer for main.c
 *
 * [EOF]
 */
