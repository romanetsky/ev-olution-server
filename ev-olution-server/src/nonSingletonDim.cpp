/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: nonSingletonDim.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 08-Apr-2024 21:02:32
 */

/* Include Files */
#include "nonSingletonDim.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : int x_size
 * Return Type  : int
 */
int nonSingletonDim(int x_size)
{
  int dim;
  dim = 2;
  if (x_size != 1) {
    dim = 1;
  }
  return dim;
}

/*
 * File trailer for nonSingletonDim.c
 *
 * [EOF]
 */
