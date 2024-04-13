/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_initialize.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 17-Mar-2024 11:09:30
 */

/* Include Files */
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_initialize.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_data.h"
#include "CoderTimeAPI.h"
#include "eml_rand_mt19937ar_stateful.h"
#include "rt_nonfinite.h"
#include "timeKeeper.h"

/* Function Definitions */
/*
 * Arguments    : void
 * Return Type  : void
 */
void Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_initialize(void)
{
  rt_InitInfAndNaN();
  c_CoderTimeAPI_callCoderClockGe();
  timeKeeper_init();
  c_eml_rand_mt19937ar_stateful_i();
  isInitialized_Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229 = true;
}

/*
 * File trailer for Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_initialize.c
 *
 * [EOF]
 */
