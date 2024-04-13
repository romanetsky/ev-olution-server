#pragma once
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
