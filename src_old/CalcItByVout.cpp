/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: CalcItByVout.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 17-Mar-2024 11:09:30
 */

/* Include Files */
#include "CalcItByVout.h"
#include "dischargeCell_array.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const float paramsBatt[38]
 *                float stateBatt[31]
 *                float Ta
 *                double Vout
 * Return Type  : void
 */
void CalcItByVout(const float paramsBatt[38], float stateBatt[31], float Ta,
                  double Vout)
{
  float stateBattVec_n_d[31];
  float stateBattVec_n_md[31];
  float f;
  int k_iter;
  memcpy(&stateBattVec_n_d[0], &stateBatt[0], 31U * sizeof(float));
  memcpy(&stateBattVec_n_md[0], &stateBatt[0], 31U * sizeof(float));
  for (k_iter = 0; k_iter < 20; k_iter++) {
    dischargeCell_array(paramsBatt, stateBatt, Ta);
    f = stateBatt[1];
    stateBattVec_n_d[1] = f + 5.0E-10F;
    stateBattVec_n_md[1] = f - 5.0E-10F;
    dischargeCell_array(paramsBatt, stateBattVec_n_d, Ta);
    dischargeCell_array(paramsBatt, stateBattVec_n_md, Ta);
    f -= (stateBatt[17] - (float)Vout) /
         ((stateBattVec_n_d[17] - stateBattVec_n_md[17]) / 1.0E-9F);
    stateBatt[1] = f;
  }
}

/*
 * File trailer for CalcItByVout.c
 *
 * [EOF]
 */
