/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: ControlKp184.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 17-Mar-2024 11:09:30
 */

#ifndef CONTROLKP184_H
#define CONTROLKP184_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void ControlKp184(double is_on_data[], int is_on_size[2],
                  double what_mode_data[], int what_mode_size[2],
                  double VmV_data[], int VmV_size[2], double ImA_data[],
                  int ImA_size[2]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for ControlKp184.h
 *
 * [EOF]
 */
