/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: controlJuntekDPH8920.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 31-Mar-2024 07:04:52
 */

/* Include Files */
#include "controlJuntekDPH8920.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_emxutil.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h"
#include "combineVectorElements.h"
#include "median.h"
#include "rand.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : double out_data[]
 *                int out_size[2]
 * Return Type  : void
 */
void b_controlJuntekDPH8920(double out_data[], int out_size[2])
{
  emxArray_real_T f_out_data;
  emxArray_real_T *r;
  double e_out_data[3];
  double d_out_data[2];
  double b_out_data;
  double c_out_data;
  double d;
  double *r1;
  int b_out_size[2];
  int c_out_size[2];
  int i;
  int loop_ub;
  /* ['VSET',num2str(ch),':',num2str(Val)];%[V] */
  d = d_rand();
  b_out_data = d * 2.5 + 2.0;
  /* ['VSET',num2str(ch),':',num2str(Val)];%[V] */
  d = d_rand();
  c_out_data = d * 2.5 + 2.0;
  if (!(fabs(b_out_data - c_out_data) < 0.3)) {
    /* ['VSET',num2str(ch),':',num2str(Val)];%[V] */
    d = d_rand();
    c_out_size[0] = 1;
    c_out_size[1] = 3;
    e_out_data[0] = b_out_data;
    e_out_data[1] = c_out_data;
    e_out_data[2] = d * 2.5 + 2.0;
    out_size[0] = 1;
    out_size[1] = 1;
    median(e_out_data, c_out_size, (double *)&b_out_data);
    out_data[0] = b_out_data;
  } else {
    b_out_size[0] = 1;
    b_out_size[1] = 2;
    d_out_data[0] = b_out_data;
    d_out_data[1] = c_out_data;
    f_out_data.data = &d_out_data[0];
    f_out_data.size = &b_out_size[0];
    f_out_data.allocatedSize = 2;
    f_out_data.numDimensions = 2;
    f_out_data.canFreeData = false;
    emxInit_real_T(&r, 2);
    combineVectorElements(&f_out_data, r);
    r1 = r->data;
    out_size[0] = 1;
    out_size[1] = r->size[1];
    loop_ub = r->size[1];
    for (i = 0; i < loop_ub; i++) {
      out_data[i] = r1[i];
    }
    emxFree_real_T(&r);
  }
  /*  str0 = []; */
  /*  for k_buf = 1:1 */
  /*      request =
   * [':',juntek_address,'r30=0,',newline];%['VSET',num2str(ch),':',num2str(Val)];%[V]
   */
  /*      fwrite(s_juntek, request); */
  /*      out0 = fgets(s_juntek); */
  /*      str0 = [str0;out0]; */
  /*  end */
  /*  str1 = (str0); */
  /*  if contains(str1,':01r30') */
  /*      i1 = strfind(str1,'='); */
  /*      i2 = strfind(str1,'.'); */
  /*      out = (str2double(char(str1(i1+1:i2-1))))/100; */
  /*      if isnan(out) */
  /*          str0 = []; */
  /*          request =
   * [':',juntek_address,'r30=0,',newline];%['VSET',num2str(ch),':',num2str(Val)];%[V]
   */
  /*          fwrite(s_juntek, request); */
  /*          out0 = fgets(s_juntek); */
  /*          str0 = [str0;out0]; */
  /*          str1 = (str0); */
  /*          if contains(str1,':01r30') */
  /*              i1 = strfind(str1,'='); */
  /*              i2 = strfind(str1,'.'); */
  /*              out = (str2double(char(str1(i1+1:i2-1))))/100; */
  /*          end */
  /*      end */
  /*  else%if strfind(str1,':01ok') */
  /*      request =
   * [':',juntek_address,'r30=0,',newline];%['VSET',num2str(ch),':',num2str(Val)];%[V]
   */
  /*      fwrite(s_juntek, request); */
  /*      out0 = fgets(s_juntek); */
  /*      str1 = [out0]; */
  /*      if contains(str1,':01r30') */
  /*          i1 = strfind(str1,'='); */
  /*          i2 = strfind(str1,'.'); */
  /*          out = (str2double(char(str1(i1+1:i2-1))))/100; */
  /*          if isnan(out) */
  /*              str0 = []; */
  /*              request =
   * [':',juntek_address,'r30=0,',newline];%['VSET',num2str(ch),':',num2str(Val)];%[V]
   */
  /*              fwrite(s_juntek, request); */
  /*              out0 = fgets(s_juntek); */
  /*              str0 = [str0;out0]; */
  /*              str1 = (str0); */
  /*              if contains(str1,':01r30') */
  /*                  i1 = strfind(str1,'='); */
  /*                  i2 = strfind(str1,'.'); */
  /*                  out = (str2double(char(str1(i1+1:i2-1))))/100; */
  /*              end */
  /*          end */
  /*      else */
  /*          out = []; */
  /*      end */
  /*      % else */
  /*      %     out = []; */
  /*  end */
  /*  if isempty(out) */
  /*      out = 0; */
  /*      disp(['Juntek Error (empty) in:',Oper]); */
  /*  end */
  /*  if isnan(out) */
  /*      out = 0; */
  /*      disp(['Juntek Error (NaN) in:',Oper]) */
  /*  end */
}

/*
 * Arguments    : double out_data[]
 *                int out_size[2]
 * Return Type  : void
 */
void controlJuntekDPH8920(double out_data[], int out_size[2])
{
  emxArray_real_T f_out_data;
  emxArray_real_T *r;
  double e_out_data[3];
  double d_out_data[2];
  double b_out_data;
  double c_out_data;
  double d;
  double *r1;
  int b_out_size[2];
  int c_out_size[2];
  int i;
  int loop_ub;
  /* ['VSET',num2str(ch),':',num2str(Val)];%[V] */
  d = d_rand();
  b_out_data = d * 5.0;
  /* ['VSET',num2str(ch),':',num2str(Val)];%[V] */
  d = d_rand();
  c_out_data = d * 5.0;
  if (!(fabs(b_out_data - c_out_data) < 0.3)) {
    /* ['VSET',num2str(ch),':',num2str(Val)];%[V] */
    d = d_rand();
    c_out_size[0] = 1;
    c_out_size[1] = 3;
    e_out_data[0] = b_out_data;
    e_out_data[1] = c_out_data;
    e_out_data[2] = d * 5.0;
    out_size[0] = 1;
    out_size[1] = 1;
    median(e_out_data, c_out_size, (double *)&b_out_data);
    out_data[0] = b_out_data;
  } else {
    b_out_size[0] = 1;
    b_out_size[1] = 2;
    d_out_data[0] = b_out_data;
    d_out_data[1] = c_out_data;
    f_out_data.data = &d_out_data[0];
    f_out_data.size = &b_out_size[0];
    f_out_data.allocatedSize = 2;
    f_out_data.numDimensions = 2;
    f_out_data.canFreeData = false;
    emxInit_real_T(&r, 2);
    combineVectorElements(&f_out_data, r);
    r1 = r->data;
    out_size[0] = 1;
    out_size[1] = r->size[1];
    loop_ub = r->size[1];
    for (i = 0; i < loop_ub; i++) {
      out_data[i] = r1[i];
    }
    emxFree_real_T(&r);
  }
  /*  str0 = []; */
  /*  for k_buf = 1:1 */
  /*      request =
   * [':',juntek_address,'r31=0,',newline];%['VSET',num2str(ch),':',num2str(Val)];%[V]
   */
  /*      fwrite(s_juntek, request); */
  /*      out0 = fgets(s_juntek); */
  /*      str0 = [str0;out0]; */
  /*  end */
  /*  str1 = (str0); */
  /*  if contains(str1,':01r31') */
  /*      i1 = strfind(str1,'='); */
  /*      i2 = strfind(str1,'.'); */
  /*      out = (str2double(char(str1(i1+1:i2-1))))/1000; */
  /*      if isnan(out) */
  /*          str0 = []; */
  /*          request =
   * [':',juntek_address,'r31=0,',newline];%['VSET',num2str(ch),':',num2str(Val)];%[V]
   */
  /*          fwrite(s_juntek, request); */
  /*          out0 = fgets(s_juntek); */
  /*          str0 = [str0;out0]; */
  /*          str1 = (str0); */
  /*          if contains(str1,':01r31') */
  /*              i1 = strfind(str1,'='); */
  /*              i2 = strfind(str1,'.'); */
  /*              out = (str2double(char(str1(i1+1:i2-1))))/1000; */
  /*          end */
  /*      end */
  /*  else%if strfind(str1,':01ok') */
  /*      request =
   * [':',juntek_address,'r31=0,',newline];%['VSET',num2str(ch),':',num2str(Val)];%[V]
   */
  /*      fwrite(s_juntek, request); */
  /*      out0 = fgets(s_juntek); */
  /*      str1 = [out0]; */
  /*      if contains(str1,':01r31') */
  /*          i1 = strfind(str1,'='); */
  /*          i2 = strfind(str1,'.'); */
  /*          out = (str2double(char(str1(i1+1:i2-1))))/1000; */
  /*          if isnan(out) */
  /*              str0 = []; */
  /*              request =
   * [':',juntek_address,'r31=0,',newline];%['VSET',num2str(ch),':',num2str(Val)];%[V]
   */
  /*              fwrite(s_juntek, request); */
  /*              out0 = fgets(s_juntek); */
  /*              str0 = [str0;out0]; */
  /*              str1 = (str0); */
  /*              if contains(str1,':01r31') */
  /*                  i1 = strfind(str1,'='); */
  /*                  i2 = strfind(str1,'.'); */
  /*                  out = (str2double(char(str1(i1+1:i2-1))))/1000; */
  /*              end */
  /*          end */
  /*      else */
  /*          out = []; */
  /*      end */
  /*      % else */
  /*      %     out = []; */
  /*  end */
  /*  if isempty(out) */
  /*      out = 0; */
  /*      disp(['Juntek Error (empty) in:',Oper]); */
  /*  end */
  /*  if isnan(out) */
  /*      out = 0; */
  /*      disp(['Juntek Error (NaN) in:',Oper]) */
  /*  end */
}

/*
 * File trailer for controlJuntekDPH8920.c
 *
 * [EOF]
 */
