/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: ismember.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 17-Mar-2024 11:09:30
 */

/* Include Files */
#include "ismember.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : double a
 *                const unsigned char s[256]
 * Return Type  : boolean_T
 */
boolean_T isMember(double a, const unsigned char s[256])
{
  int k;
  boolean_T exitg1;
  boolean_T tf;
  tf = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 256)) {
    if (a == s[k]) {
      tf = true;
      exitg1 = true;
    } else {
      k++;
    }
  }
  return tf;
}

/*
 * File trailer for ismember.c
 *
 * [EOF]
 */
