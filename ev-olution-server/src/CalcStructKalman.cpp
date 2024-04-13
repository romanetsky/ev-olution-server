/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: CalcStructKalman.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 08-Apr-2024 21:02:32
 */

/* Include Files */
#include "CalcStructKalman.h"
#include "CellKalman.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : struct22_T *StructKalman
 *                float Ta
 *                double t_data[]
 *                const int t_size[2]
 *                const double i_in_data[]
 *                const double v_out_data[]
 * Return Type  : void
 */
void CalcStructKalman(struct22_T *StructKalman, float Ta, double t_data[],
                      const int t_size[2], const double i_in_data[],
                      const double v_out_data[])
{
  double d;
  float b_StructKalman[38];
  float BatState_k_k0[31];
  float c_StructKalman[31];
  int i;
  int i1;
  int loop_ub;
  loop_ub = t_size[0] - 1;
  for (i = 0; i <= loop_ub; i++) {
    t_data[i] /= 3600.0;
  }
  i = StructKalman->N_bat;
  for (loop_ub = 0; loop_ub < i; loop_ub++) {
    for (i1 = 0; i1 < 38; i1++) {
      b_StructKalman[i1] = StructKalman->BatParams[loop_ub + (i1 << 4)];
    }
    for (i1 = 0; i1 < 31; i1++) {
      c_StructKalman[i1] = StructKalman->BatState_k_k[loop_ub + (i1 << 4)];
    }
    d = t_data[loop_ub];
    StructKalman->P_k_k[loop_ub] = CellKalman(
        b_StructKalman, c_StructKalman, StructKalman->P_k_k[loop_ub], Ta, d,
        (float)d - StructKalman->BatState_k_k[loop_ub + 288],
        i_in_data[loop_ub], StructKalman->Qkalman, StructKalman->Rkalman,
        v_out_data[loop_ub], StructKalman->eps1, BatState_k_k0);
    for (i1 = 0; i1 < 31; i1++) {
      StructKalman->BatState_k_k[loop_ub + (i1 << 4)] = BatState_k_k0[i1];
    }
  }
}

/*
 * Arguments    : struct22_T *StructKalman
 *                float Ta
 *                double t_data[]
 *                const int t_size[2]
 *                const double i_in_data[]
 *                const double v_out_data[]
 * Return Type  : void
 */
void b_CalcStructKalman(struct22_T *StructKalman, float Ta, double t_data[],
                        const int t_size[2], const double i_in_data[],
                        const double v_out_data[])
{
  double d;
  float b_StructKalman[38];
  float BatState_k_k0[31];
  float c_StructKalman[31];
  int i;
  int i1;
  int loop_ub;
  loop_ub = t_size[1] - 1;
  for (i = 0; i <= loop_ub; i++) {
    t_data[i] /= 3600.0;
  }
  i = StructKalman->N_bat;
  for (loop_ub = 0; loop_ub < i; loop_ub++) {
    for (i1 = 0; i1 < 38; i1++) {
      b_StructKalman[i1] = StructKalman->BatParams[loop_ub + (i1 << 4)];
    }
    for (i1 = 0; i1 < 31; i1++) {
      c_StructKalman[i1] = StructKalman->BatState_k_k[loop_ub + (i1 << 4)];
    }
    d = t_data[loop_ub];
    StructKalman->P_k_k[loop_ub] = CellKalman(
        b_StructKalman, c_StructKalman, StructKalman->P_k_k[loop_ub], Ta, d,
        (float)d - StructKalman->BatState_k_k[loop_ub + 288],
        i_in_data[loop_ub], StructKalman->Qkalman, StructKalman->Rkalman,
        v_out_data[loop_ub], StructKalman->eps1, BatState_k_k0);
    for (i1 = 0; i1 < 31; i1++) {
      StructKalman->BatState_k_k[loop_ub + (i1 << 4)] = BatState_k_k0[i1];
    }
  }
}

/*
 * File trailer for CalcStructKalman.c
 *
 * [EOF]
 */
