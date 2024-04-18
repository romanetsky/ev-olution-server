/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: CalcPortSpiPrm.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 13-Apr-2024 21:34:04
 */

#ifndef CALCPORTSPIPRM_H
#define CALCPORTSPIPRM_H

/* Include Files */
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void CalcPortSpiPrm(unsigned char prmBrdSpi_disconnect,
                    unsigned char prmBrdSpi_bypass,
                    const unsigned char prmBrdSpi_PortSpiRow_esp[12336],
                    const unsigned char prmBrdSpi_SwitchMat_esp[4112],
                    const unsigned char prmBrdSpi_PortSpiRow_esp2[288],
                    const unsigned char prmBrdSpi_SwitchMat_esp2[24],
                    const unsigned char prmBrdSpi_Pac2Vid[4112],
                    const unsigned char prmBrdSpi_Pac2Vid2[24],
                    unsigned char SwitchIn, unsigned char SpiPortRow[24],
                    double SwitchIdOut_data[], int SwitchIdOut_size[2],
                    unsigned char Pac2Vid0[256]);

void b_CalcPortSpiPrm(unsigned char prmBrdSpi_disconnect,
                      unsigned char prmBrdSpi_bypass,
                      const unsigned char prmBrdSpi_PortSpiRow_esp[12336],
                      const unsigned char prmBrdSpi_SwitchMat_esp[4112],
                      const unsigned char prmBrdSpi_PortSpiRow_esp2[288],
                      const unsigned char prmBrdSpi_SwitchMat_esp2[24],
                      const unsigned char prmBrdSpi_Pac2Vid[4112],
                      const unsigned char prmBrdSpi_Pac2Vid2[24],
                      const emxArray_uint8_T *SwitchIn,
                      unsigned char SpiPortRow[24], double SwitchIdOut_data[],
                      int SwitchIdOut_size[2], unsigned char Pac2Vid0[256]);

void c_CalcPortSpiPrm(unsigned char prmBrdSpi_disconnect,
                      unsigned char prmBrdSpi_bypass,
                      const unsigned char prmBrdSpi_PortSpiRow_esp[12336],
                      const unsigned char prmBrdSpi_SwitchMat_esp[4112],
                      const unsigned char prmBrdSpi_PortSpiRow_esp2[288],
                      const unsigned char prmBrdSpi_SwitchMat_esp2[24],
                      const unsigned char prmBrdSpi_Pac2Vid[4112],
                      const unsigned char prmBrdSpi_Pac2Vid2[24],
                      const unsigned char SwitchIn[256],
                      unsigned char SpiPortRow[24], double SwitchIdOut_data[],
                      int SwitchIdOut_size[2], unsigned char Pac2Vid0[256]);

void d_CalcPortSpiPrm(unsigned char prmBrdSpi_disconnect,
                      unsigned char prmBrdSpi_bypass,
                      const unsigned char prmBrdSpi_PortSpiRow_esp[12336],
                      const unsigned char prmBrdSpi_SwitchMat_esp[4112],
                      const unsigned char prmBrdSpi_PortSpiRow_esp2[288],
                      const unsigned char prmBrdSpi_SwitchMat_esp2[24],
                      const unsigned char prmBrdSpi_Pac2Vid[4112],
                      const unsigned char prmBrdSpi_Pac2Vid2[24],
                      const emxArray_real_T *SwitchIn,
                      unsigned char SpiPortRow[24], double SwitchIdOut_data[],
                      int SwitchIdOut_size[2], unsigned char Pac2Vid0[256]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for CalcPortSpiPrm.h
 *
 * [EOF]
 */
