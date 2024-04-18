/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: Esp32StepSwitchToggleCombAll_ESP_48.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 17-Mar-2024 11:09:30
 */

#ifndef ESP32STEPSWITCHTOGGLECOMBALL_ESP_48_H
#define ESP32STEPSWITCHTOGGLECOMBALL_ESP_48_H

/* Include Files */
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
boolean_T c_Esp32StepSwitchToggleCombAll_(
    double Sel_data[], int Sel_size[2], float CutOffChrV, float CutOffDisV,
    emxArray_real_T *BattConfigPerIna, const emxArray_real_T *Vbat,
    double Nina219, short prmBrd_Nbat, unsigned char prmBrd_spi_disconnect,
    unsigned char prmBrd_spi_bypass, float prmCnfg_Ttoggle,
    short prmCnfg_NtoggleDrop, short prmCnfg_minLenIna219,
    const struct4_T prmSeq, signed char VthFlag, const float t_data[],
    const int t_size[2], double tLastToggle_data[], int tLastToggle_size[2],
    boolean_T BattConfigAct_data[], const int BattConfigAct_size[2],
    boolean_T changeConfigFlag_data[], int changeConfigFlag_size[2]);

boolean_T d_Esp32StepSwitchToggleCombAll_(
    double Sel_data[], int Sel_size[2], float CutOffChrV, float CutOffDisV,
    emxArray_real_T *BattConfigPerIna, const emxArray_real_T *Vbat,
    double Nina219, short prmBrd_Nbat, unsigned char prmBrd_spi_disconnect,
    unsigned char prmBrd_spi_bypass, float prmCnfg_Ttoggle,
    short prmCnfg_NtoggleDrop, short prmCnfg_minLenIna219,
    const struct4_T prmSeq, signed char VthFlag, double t,
    double tLastToggle_data[], int tLastToggle_size[2],
    boolean_T BattConfigAct_data[], const int BattConfigAct_size[2],
    boolean_T changeConfigFlag_data[], int changeConfigFlag_size[2]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for Esp32StepSwitchToggleCombAll_ESP_48.h
 *
 * [EOF]
 */
