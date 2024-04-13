/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: sprintf.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 17-Mar-2024 11:09:30
 */

/* Include Files */
#include "sprintf.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_emxutil.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h"
#include "rt_nonfinite.h"
#include <stddef.h>
#include <stdio.h>

/* Function Definitions */
/*
 * Arguments    : int varargin_1
 *                emxArray_char_T *str
 * Return Type  : void
 */
void b_sprintf(int varargin_1, emxArray_char_T *str)
{
  int i;
  int nbytes;
  char *str_data;
  nbytes = snprintf(NULL, 0, "%d", varargin_1);
  i = str->size[0] * str->size[1];
  str->size[0] = 1;
  str->size[1] = nbytes + 1;
  emxEnsureCapacity_char_T(str, i);
  str_data = str->data;
  snprintf(&str_data[0], (size_t)(nbytes + 1), "%d", varargin_1);
  i = str->size[0] * str->size[1];
  if (nbytes < 1) {
    str->size[1] = 0;
  } else {
    str->size[1] = nbytes;
  }
  emxEnsureCapacity_char_T(str, i);
}

/*
 * Arguments    : int varargin_1
 *                int varargin_2
 *                emxArray_char_T *str
 * Return Type  : void
 */
void c_sprintf(int varargin_1, int varargin_2, emxArray_char_T *str)
{
  int i;
  int nbytes;
  char *str_data;
  nbytes = snprintf(NULL, 0, "[%d,%d]", varargin_1, varargin_2);
  i = str->size[0] * str->size[1];
  str->size[0] = 1;
  str->size[1] = nbytes + 1;
  emxEnsureCapacity_char_T(str, i);
  str_data = str->data;
  snprintf(&str_data[0], (size_t)(nbytes + 1), "[%d,%d]", varargin_1,
           varargin_2);
  i = str->size[0] * str->size[1];
  if (nbytes < 1) {
    str->size[1] = 0;
  } else {
    str->size[1] = nbytes;
  }
  emxEnsureCapacity_char_T(str, i);
}

/*
 * Arguments    : double varargin_1
 *                emxArray_char_T *str
 * Return Type  : void
 */
void d_sprintf(double varargin_1, emxArray_char_T *str)
{
  int i;
  int nbytes;
  char *str_data;
  nbytes = snprintf(NULL, 0, "%f", varargin_1);
  i = str->size[0] * str->size[1];
  str->size[0] = 1;
  str->size[1] = nbytes + 1;
  emxEnsureCapacity_char_T(str, i);
  str_data = str->data;
  snprintf(&str_data[0], (size_t)(nbytes + 1), "%f", varargin_1);
  i = str->size[0] * str->size[1];
  if (nbytes < 1) {
    str->size[1] = 0;
  } else {
    str->size[1] = nbytes;
  }
  emxEnsureCapacity_char_T(str, i);
}

/*
 * Arguments    : float varargin_2
 *                emxArray_char_T *str
 * Return Type  : void
 */
void e_sprintf(float varargin_2, emxArray_char_T *str)
{
  int i;
  int nbytes;
  char *str_data;
  nbytes = snprintf(NULL, 0, "VSET%d:%f", 1, varargin_2);
  i = str->size[0] * str->size[1];
  str->size[0] = 1;
  str->size[1] = nbytes + 1;
  emxEnsureCapacity_char_T(str, i);
  str_data = str->data;
  snprintf(&str_data[0], (size_t)(nbytes + 1), "VSET%d:%f", 1, varargin_2);
  i = str->size[0] * str->size[1];
  if (nbytes < 1) {
    str->size[1] = 0;
  } else {
    str->size[1] = nbytes;
  }
  emxEnsureCapacity_char_T(str, i);
}

/*
 * Arguments    : float varargin_2
 *                emxArray_char_T *str
 * Return Type  : void
 */
void f_sprintf(float varargin_2, emxArray_char_T *str)
{
  int i;
  int nbytes;
  char *str_data;
  nbytes = snprintf(NULL, 0, "ISET%d:%f", 1, varargin_2);
  i = str->size[0] * str->size[1];
  str->size[0] = 1;
  str->size[1] = nbytes + 1;
  emxEnsureCapacity_char_T(str, i);
  str_data = str->data;
  snprintf(&str_data[0], (size_t)(nbytes + 1), "ISET%d:%f", 1, varargin_2);
  i = str->size[0] * str->size[1];
  if (nbytes < 1) {
    str->size[1] = 0;
  } else {
    str->size[1] = nbytes;
  }
  emxEnsureCapacity_char_T(str, i);
}

/*
 * Arguments    : int varargin_1
 *                emxArray_char_T *str
 * Return Type  : void
 */
void g_sprintf(int varargin_1, emxArray_char_T *str)
{
  int i;
  int nbytes;
  char *str_data;
  nbytes = snprintf(NULL, 0, "%04d", varargin_1);
  i = str->size[0] * str->size[1];
  str->size[0] = 1;
  str->size[1] = nbytes + 1;
  emxEnsureCapacity_char_T(str, i);
  str_data = str->data;
  snprintf(&str_data[0], (size_t)(nbytes + 1), "%04d", varargin_1);
  i = str->size[0] * str->size[1];
  if (nbytes < 1) {
    str->size[1] = 0;
  } else {
    str->size[1] = nbytes;
  }
  emxEnsureCapacity_char_T(str, i);
}

/*
 * Arguments    : int varargin_1
 *                int varargin_2
 *                int varargin_3
 *                emxArray_char_T *str
 * Return Type  : void
 */
void h_sprintf(int varargin_1, int varargin_2, int varargin_3,
               emxArray_char_T *str)
{
  int i;
  int nbytes;
  char *str_data;
  nbytes = snprintf(NULL, 0, "(%d,%d,%d)", varargin_1, varargin_2, varargin_3);
  i = str->size[0] * str->size[1];
  str->size[0] = 1;
  str->size[1] = nbytes + 1;
  emxEnsureCapacity_char_T(str, i);
  str_data = str->data;
  snprintf(&str_data[0], (size_t)(nbytes + 1), "(%d,%d,%d)", varargin_1,
           varargin_2, varargin_3);
  i = str->size[0] * str->size[1];
  if (nbytes < 1) {
    str->size[1] = 0;
  } else {
    str->size[1] = nbytes;
  }
  emxEnsureCapacity_char_T(str, i);
}

/*
 * Arguments    : double varargin_1
 *                emxArray_char_T *str
 * Return Type  : void
 */
void i_sprintf(double varargin_1, emxArray_char_T *str)
{
  int i;
  int nbytes;
  char *str_data;
  nbytes = snprintf(NULL, 0, "%f ", varargin_1);
  i = str->size[0] * str->size[1];
  str->size[0] = 1;
  str->size[1] = nbytes + 1;
  emxEnsureCapacity_char_T(str, i);
  str_data = str->data;
  snprintf(&str_data[0], (size_t)(nbytes + 1), "%f ", varargin_1);
  i = str->size[0] * str->size[1];
  if (nbytes < 1) {
    str->size[1] = 0;
  } else {
    str->size[1] = nbytes;
  }
  emxEnsureCapacity_char_T(str, i);
}

/*
 * Arguments    : signed char varargin_1
 *                emxArray_char_T *str
 * Return Type  : void
 */
void j_sprintf(signed char varargin_1, emxArray_char_T *str)
{
  int i;
  int nbytes;
  char *str_data;
  nbytes = snprintf(NULL, 0, "%d", varargin_1);
  i = str->size[0] * str->size[1];
  str->size[0] = 1;
  str->size[1] = nbytes + 1;
  emxEnsureCapacity_char_T(str, i);
  str_data = str->data;
  snprintf(&str_data[0], (size_t)(nbytes + 1), "%d", varargin_1);
  i = str->size[0] * str->size[1];
  if (nbytes < 1) {
    str->size[1] = 0;
  } else {
    str->size[1] = nbytes;
  }
  emxEnsureCapacity_char_T(str, i);
}

/*
 * Arguments    : float varargin_1
 *                emxArray_char_T *str
 * Return Type  : void
 */
void k_sprintf(float varargin_1, emxArray_char_T *str)
{
  int i;
  int nbytes;
  char *str_data;
  nbytes = snprintf(NULL, 0, "%f", varargin_1);
  i = str->size[0] * str->size[1];
  str->size[0] = 1;
  str->size[1] = nbytes + 1;
  emxEnsureCapacity_char_T(str, i);
  str_data = str->data;
  snprintf(&str_data[0], (size_t)(nbytes + 1), "%f", varargin_1);
  i = str->size[0] * str->size[1];
  if (nbytes < 1) {
    str->size[1] = 0;
  } else {
    str->size[1] = nbytes;
  }
  emxEnsureCapacity_char_T(str, i);
}

/*
 * Arguments    : double varargin_2
 *                emxArray_char_T *str
 * Return Type  : void
 */
void l_sprintf(double varargin_2, emxArray_char_T *str)
{
  int i;
  int nbytes;
  char *str_data;
  nbytes = snprintf(NULL, 0, "ISET%d:%f", 1, varargin_2);
  i = str->size[0] * str->size[1];
  str->size[0] = 1;
  str->size[1] = nbytes + 1;
  emxEnsureCapacity_char_T(str, i);
  str_data = str->data;
  snprintf(&str_data[0], (size_t)(nbytes + 1), "ISET%d:%f", 1, varargin_2);
  i = str->size[0] * str->size[1];
  if (nbytes < 1) {
    str->size[1] = 0;
  } else {
    str->size[1] = nbytes;
  }
  emxEnsureCapacity_char_T(str, i);
}

/*
 * Arguments    : int varargin_1
 *                int varargin_2
 *                int varargin_3
 *                int varargin_4
 *                int varargin_5
 *                int varargin_6
 *                emxArray_char_T *str
 * Return Type  : void
 */
void m_sprintf(int varargin_1, int varargin_2, int varargin_3, int varargin_4,
               int varargin_5, int varargin_6, emxArray_char_T *str)
{
  int i;
  int nbytes;
  char *str_data;
  nbytes = snprintf(NULL, 0, "%d-%d-%d %d %d %d", varargin_1, varargin_2,
                    varargin_3, varargin_4, varargin_5, varargin_6);
  i = str->size[0] * str->size[1];
  str->size[0] = 1;
  str->size[1] = nbytes + 1;
  emxEnsureCapacity_char_T(str, i);
  str_data = str->data;
  snprintf(&str_data[0], (size_t)(nbytes + 1), "%d-%d-%d %d %d %d", varargin_1,
           varargin_2, varargin_3, varargin_4, varargin_5, varargin_6);
  i = str->size[0] * str->size[1];
  if (nbytes < 1) {
    str->size[1] = 0;
  } else {
    str->size[1] = nbytes;
  }
  emxEnsureCapacity_char_T(str, i);
}

/*
 * File trailer for sprintf.c
 *
 * [EOF]
 */
