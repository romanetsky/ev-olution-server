/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: defineOutStruct.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 17-Mar-2024 11:09:30
 */

/* Include Files */
#include "defineOutStruct.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_emxutil.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : short NmaxBrd
 *                short NmaxBat
 *                emxArray_real_T *outStruct_VbusTest
 *                emxArray_real_T *outStruct_VmKp184Test
 *                boolean_T *outStruct_VresetFlag
 *                double *outStruct_Rint
 * Return Type  : boolean_T
 */
boolean_T defineOutStruct(short NmaxBrd, short NmaxBat,
                          emxArray_real_T *outStruct_VbusTest,
                          emxArray_real_T *outStruct_VmKp184Test,
                          boolean_T *outStruct_VresetFlag,
                          double *outStruct_Rint)
{
  double *outStruct_VbusTest_data;
  int i;
  int loop_ub_tmp;
  boolean_T outStruct_VpassFlag;
  i = outStruct_VbusTest->size[0] * outStruct_VbusTest->size[1];
  outStruct_VbusTest->size[0] = NmaxBrd;
  outStruct_VbusTest->size[1] = NmaxBat;
  emxEnsureCapacity_real_T(outStruct_VbusTest, i);
  outStruct_VbusTest_data = outStruct_VbusTest->data;
  loop_ub_tmp = NmaxBrd * NmaxBat;
  for (i = 0; i < loop_ub_tmp; i++) {
    outStruct_VbusTest_data[i] = 0.0;
  }
  i = outStruct_VmKp184Test->size[0] * outStruct_VmKp184Test->size[1];
  outStruct_VmKp184Test->size[0] = NmaxBrd;
  outStruct_VmKp184Test->size[1] = NmaxBat;
  emxEnsureCapacity_real_T(outStruct_VmKp184Test, i);
  outStruct_VbusTest_data = outStruct_VmKp184Test->data;
  for (i = 0; i < loop_ub_tmp; i++) {
    outStruct_VbusTest_data[i] = 0.0;
  }
  outStruct_VpassFlag = false;
  *outStruct_VresetFlag = false;
  *outStruct_Rint = 0.0;
  return outStruct_VpassFlag;
}

/*
 * File trailer for defineOutStruct.c
 *
 * [EOF]
 */
