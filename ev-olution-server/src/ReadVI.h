/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: ReadVI.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 10-Apr-2024 21:46:28
 */

#ifndef READVI_H
#define READVI_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
int ReadVI(signed char ProjectFlag, const signed char VIpacId[16],
           const unsigned char Pac2Vid0All_data[],
           const int Pac2Vid0All_size[3], short N_bat1, double N_bat,
           const float pIacs758_data[], signed char Iacs758Flag,
           const double pIshunt_data[], const int pIshunt_size[3],
           const float Rwire_data[], const int Rwire_size[2], float Vbat_data[],
           double Vbat0_data[], int *Vbat0_size, double Ibat_data[],
           int *Ibat_size, double VbatMat_data[], int VbatMat_size[2],
           double VbatMat0_data[], int VbatMat0_size[2], double Ishunt_data[],
           int Ishunt_size[2], double tV_data[], int tV_size[2],
           double tV1_data[], int *tV1_size, double tI_data[], int tI_size[2],
           double tI1_data[], int *tI1_size, double Iacs758_data[],
           int Iacs758_size[2], double Vdebug0_data[], int Vdebug0_size[3],
           boolean_T *keepMeas, double errI2C_data[], int errI2C_size[2]);

int b_ReadVI(double Nina219, signed char ProjectFlag,
             const signed char VIpacId[32],
             const unsigned char Pac2Vid0All_data[],
             const int Pac2Vid0All_size[3], short N_bat1, double N_bat,
             const float pIacs758_data[], const int pIacs758_size[3],
             signed char Iacs758Flag, const double pIshunt_data[],
             const int pIshunt_size[3], const float Rwire_data[],
             const int Rwire_size[2], float Vbat_data[], double Vbat0_data[],
             int *Vbat0_size, double Ibat_data[], int *Ibat_size,
             double VbatMat_data[], int VbatMat_size[2], double VbatMat0_data[],
             int VbatMat0_size[2], double Ishunt_data[], int Ishunt_size[2],
             double tV_data[], int tV_size[2], double tV1_data[], int *tV1_size,
             double tI_data[], int tI_size[2], double tI1_data[], int *tI1_size,
             double Iacs758_data[], int Iacs758_size[2], double Vdebug0_data[],
             int Vdebug0_size[3], boolean_T *keepMeas, double errI2C_data[],
             int errI2C_size[2]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for ReadVI.h
 *
 * [EOF]
 */
