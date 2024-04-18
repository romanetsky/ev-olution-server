/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 31-Mar-2024 07:04:52
 */

#ifndef CHR_DIS_KP184_KA6005P_JUNTEK_ESP32SER_20240229_TYPES_H
#define CHR_DIS_KP184_KA6005P_JUNTEK_ESP32SER_20240229_TYPES_H

/* Include Files */
#include "rtwtypes.h"

/* Type Definitions */
#ifndef typedef_struct1_T
#define typedef_struct1_T
typedef struct {
  short NswRepMax;
  short NbrdMax;
  short NgrpMax;
  short NseqMax;
  short NstateMax;
  short NitstMax;
  short NbatMax;
  short NstrRowMax;
  short NstrColMax;
  short NtimeMax;
  short NkalmanBatState;
  short NkalmanBatParams;
} struct1_T;
#endif /* typedef_struct1_T */

#ifndef typedef_struct2_T
#define typedef_struct2_T
typedef struct {
  short SwRepId[32];
  short Nrep;
  boolean_T PlotSocFlag;
  boolean_T PlotItFlag;
  boolean_T PlotTempFlag;
  boolean_T PlotVFlag;
  boolean_T PlotIFlag;
  boolean_T PlotIacsFlag;
  float MaxTime;
  float dt;
  short T2Show;
  short Nt;
  short Nt0;
  boolean_T testVreset;
  short seq;
} struct2_T;
#endif /* typedef_struct2_T */

#ifndef typedef_struct12_T
#define typedef_struct12_T
typedef struct {
  short ip;
} struct12_T;
#endif /* typedef_struct12_T */

#ifndef typedef_struct14_T
#define typedef_struct14_T
typedef struct {
  signed char mod;
  signed char seq;
  signed char ins;
  signed char sw;
  signed char sw16to1;
} struct14_T;
#endif /* typedef_struct14_T */

#ifndef typedef_struct17_T
#define typedef_struct17_T
typedef struct {
  float ImaxAcDC;
  float minVjuntekInput;
  float juntekEfficencyFactor;
  float baudrate;
} struct17_T;
#endif /* typedef_struct17_T */

#ifndef typedef_struct18_T
#define typedef_struct18_T
typedef struct {
  signed char Ntry;
} struct18_T;
#endif /* typedef_struct18_T */

#ifndef typedef_struct19_T
#define typedef_struct19_T
typedef struct {
  float Rshunt;
  float Vratio;
  boolean_T IfromVdivR_flag;
} struct19_T;
#endif /* typedef_struct19_T */

#ifndef typedef_struct16_T
#define typedef_struct16_T
typedef struct {
  struct17_T jun;
  struct18_T sw16to1;
  struct19_T swOut;
} struct16_T;
#endif /* typedef_struct16_T */

#ifndef typedef_struct15_T
#define typedef_struct15_T
typedef struct {
  boolean_T juntek;
  boolean_T ka6005p;
  boolean_T kp184;
  boolean_T swm;
  boolean_T sw16to1;
  boolean_T swOut;
  struct16_T prm;
  signed char ProjectFlag;
} struct15_T;
#endif /* typedef_struct15_T */

#ifndef typedef_struct29_T
#define typedef_struct29_T
typedef struct {
  int saveDir;
  int sufixDir;
  int prefixFile;
  int savePath_pIacs;
  int savePath_Rval;
  int savePath_Rwire;
} struct29_T;
#endif /* typedef_struct29_T */

#ifndef typedef_struct21_T
#define typedef_struct21_T
typedef struct {
  float BatState[496];
  float BatParams[608];
  float BatStateOrg[496];
} struct21_T;
#endif /* typedef_struct21_T */

#ifndef typedef_struct22_T
#define typedef_struct22_T
typedef struct {
  float eps1;
  float Qkalman;
  float Rkalman;
  float P_k_k[16];
  float BatState_k_k[496];
  float BatParams[608];
  short N_bat;
} struct22_T;
#endif /* typedef_struct22_T */

#ifndef typedef_struct4_T
#define typedef_struct4_T
typedef struct {
  float VthDis;
  float VthChr;
  float VthOvDis;
  float VthOvChr;
  float VthUnDis;
  float VthUnChr;
  signed char VthFlag[16];
} struct4_T;
#endif /* typedef_struct4_T */

#ifndef typedef_struct6_T
#define typedef_struct6_T
typedef struct {
  boolean_T isTest;
  signed char ins[16];
  signed char swm[16];
  signed char sw16to1[16];
  signed char grp[16];
  boolean_T isPrm;
  signed char Nst;
} struct6_T;
#endif /* typedef_struct6_T */

#ifndef typedef_struct7_T
#define typedef_struct7_T
typedef struct {
  signed char BrdBeforePSflag[128];
  signed char NegIflag[128];
  boolean_T isTest;
  signed char ins[128];
  signed char meas[128];
  signed char swm[16];
  signed char sw16to1[16];
  signed char grp[128];
  float Rin[128];
  signed char NTtest;
  float minI;
  float maxI;
  float i_in_test[32];
  signed char Nitst;
  unsigned char ItestSwitch[32768];
  float pauseOff;
  boolean_T measRintR;
  boolean_T useRwireFlag;
  signed char RintBatId;
  float Rload;
  unsigned char Nst;
} struct7_T;
#endif /* typedef_struct7_T */

#ifndef typedef_struct5_T
#define typedef_struct5_T
typedef struct {
  struct6_T v;
  struct7_T i;
  boolean_T savePrmFlag;
} struct5_T;
#endif /* typedef_struct5_T */

#ifndef typedef_struct11_T
#define typedef_struct11_T
typedef struct {
  char COM_esp32[12];
  short N_COM_esp32;
  short grp[2];
  short Ngrp;
  short Nbrd;
  short uGroups[8];
  short NuGrp;
  char COM_kp184[6];
  char COM_ka6005P[6];
  char COM_juntek[6];
  char COM_swm[6];
  char COM_sw16to1[6];
  char COM_swOut[6];
} struct11_T;
#endif /* typedef_struct11_T */

#ifndef typedef_struct13_T
#define typedef_struct13_T
typedef struct {
  char mod[8192];
  char seq[8192];
  char ins[8192];
  char sw[8192];
  char sw16to1[8192];
  struct14_T num;
} struct13_T;
#endif /* typedef_struct13_T */

#ifndef typedef_struct23_T
#define typedef_struct23_T
typedef struct {
  float Vd;
  float ImaxDis;
  float Icharge;
  float IchargePhase2;
  float IchargePhase3;
  float minIphase2;
  float dIphase2;
  float CutOffDisV[16];
  float CutOffChrV[16];
  float Vmin;
  float Vmax;
  float VresetMax;
  float Rint[32];
  float t[1024];
  float i_in[1024];
  float T;
} struct23_T;
#endif /* typedef_struct23_T */

#ifndef typedef_struct25_T
#define typedef_struct25_T
typedef struct {
  double rst;
  unsigned char disconnect;
  unsigned char bypass;
  unsigned char PortSpiRow_esp[12336];
  unsigned char SwitchMat_esp[4112];
  unsigned char PortSpiRow_esp2[288];
  unsigned char SwitchMat_esp2[24];
  unsigned char Pac2Vid[4112];
  unsigned char Pac2Vid2[24];
} struct25_T;
#endif /* typedef_struct25_T */

#ifndef typedef_struct26_T
#define typedef_struct26_T
typedef struct {
  double i2cPacAdd[16];
  signed char VIpacId[32];
  boolean_T readIpacFlag;
  float Rval[64];
  float Rshunt[2];
  float pIacs758[4];
  signed char Iacs758Flag;
  signed char Iacs758Id;
} struct26_T;
#endif /* typedef_struct26_T */

#ifndef typedef_struct24_T
#define typedef_struct24_T
typedef struct {
  short N_bat1;
  short N_bat2;
  short N_bat;
  double Nina219;
  short Nbat;
  struct25_T spi;
  struct26_T pac;
  float Rwire[32];
  boolean_T useRwireFlag;
} struct24_T;
#endif /* typedef_struct24_T */

#ifndef typedef_struct27_T
#define typedef_struct27_T
typedef struct {
  boolean_T ToggleFlag;
  float Ttoggle;
  short NtoggleDrop;
  short minLenIna219;
  unsigned char BattConfigDis1[256];
  unsigned char BattConfigChr1[256];
  unsigned char BattConfigChr2[256];
  unsigned char BattConfigChr3[256];
  unsigned char BattConfigStandby;
  unsigned char BattConfigBypass;
} struct27_T;
#endif /* typedef_struct27_T */

#ifndef typedef_struct28_T
#define typedef_struct28_T
typedef struct {
  char saveDir[256];
  char sufixDir[256];
  char prefixFile[256];
  char savePath_pIacs[512];
  char savePath_Rval[512];
  char savePath_Rwire[512];
  struct29_T num;
} struct28_T;
#endif /* typedef_struct28_T */

#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T
struct emxArray_real_T {
  double *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};
#endif /* struct_emxArray_real_T */
#ifndef typedef_emxArray_real_T
#define typedef_emxArray_real_T
typedef struct emxArray_real_T emxArray_real_T;
#endif /* typedef_emxArray_real_T */

#ifndef typedef_struct30_T
#define typedef_struct30_T
typedef struct {
  boolean_T VpassFlag;
  boolean_T VresetFlag;
  double Rint;
  emxArray_real_T *VbusTest;
  emxArray_real_T *VmKp184Test;
} struct30_T;
#endif /* typedef_struct30_T */

#ifndef struct_emxArray_real32_T
#define struct_emxArray_real32_T
struct emxArray_real32_T {
  float *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};
#endif /* struct_emxArray_real32_T */
#ifndef typedef_emxArray_real32_T
#define typedef_emxArray_real32_T
typedef struct emxArray_real32_T emxArray_real32_T;
#endif /* typedef_emxArray_real32_T */

#ifndef struct_emxArray_boolean_T
#define struct_emxArray_boolean_T
struct emxArray_boolean_T {
  boolean_T *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};
#endif /* struct_emxArray_boolean_T */
#ifndef typedef_emxArray_boolean_T
#define typedef_emxArray_boolean_T
typedef struct emxArray_boolean_T emxArray_boolean_T;
#endif /* typedef_emxArray_boolean_T */

#ifndef struct_emxArray_uint8_T
#define struct_emxArray_uint8_T
struct emxArray_uint8_T {
  unsigned char *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};
#endif /* struct_emxArray_uint8_T */
#ifndef typedef_emxArray_uint8_T
#define typedef_emxArray_uint8_T
typedef struct emxArray_uint8_T emxArray_uint8_T;
#endif /* typedef_emxArray_uint8_T */

#ifndef struct_emxArray_int8_T
#define struct_emxArray_int8_T
struct emxArray_int8_T {
  signed char *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};
#endif /* struct_emxArray_int8_T */
#ifndef typedef_emxArray_int8_T
#define typedef_emxArray_int8_T
typedef struct emxArray_int8_T emxArray_int8_T;
#endif /* typedef_emxArray_int8_T */

#ifndef struct_emxArray_int32_T
#define struct_emxArray_int32_T
struct emxArray_int32_T {
  int *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};
#endif /* struct_emxArray_int32_T */
#ifndef typedef_emxArray_int32_T
#define typedef_emxArray_int32_T
typedef struct emxArray_int32_T emxArray_int32_T;
#endif /* typedef_emxArray_int32_T */

#ifndef struct_emxArray_uint16_T
#define struct_emxArray_uint16_T
struct emxArray_uint16_T {
  unsigned short *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};
#endif /* struct_emxArray_uint16_T */
#ifndef typedef_emxArray_uint16_T
#define typedef_emxArray_uint16_T
typedef struct emxArray_uint16_T emxArray_uint16_T;
#endif /* typedef_emxArray_uint16_T */

#ifndef struct_emxArray_char_T
#define struct_emxArray_char_T
struct emxArray_char_T {
  char *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};
#endif /* struct_emxArray_char_T */
#ifndef typedef_emxArray_char_T
#define typedef_emxArray_char_T
typedef struct emxArray_char_T emxArray_char_T;
#endif /* typedef_emxArray_char_T */

#ifndef typedef_struct10_T
#define typedef_struct10_T
typedef struct {
  struct11_T com;
  struct12_T wifi;
  boolean_T kp184_Flag;
  boolean_T ka6005p_Flag;
  boolean_T juntek_Flag;
  boolean_T swm_Flag;
  boolean_T sw16to1_Flag;
  boolean_T swOut_Flag;
  double Esp32_v1[2];
} struct10_T;
#endif /* typedef_struct10_T */

#ifndef typedef_struct9_T
#define typedef_struct9_T
typedef struct {
  signed char V[128];
  float Vd[128];
  signed char b_I[128];
} struct9_T;
#endif /* typedef_struct9_T */

#ifndef typedef_struct8_T
#define typedef_struct8_T
typedef struct {
  signed char BrdBeforePSflag[16];
  boolean_T bit_flag;
  signed char chr[128];
  float IdisChr[128];
  signed char ins[128];
  signed char Nst;
  signed char swm[16];
  signed char sw16to1[16];
  struct9_T meas;
  float dIthr;
  float dVthr;
} struct8_T;
#endif /* typedef_struct8_T */

#ifndef typedef_struct3_T
#define typedef_struct3_T
typedef struct {
  signed char mod[16];
  signed char chr[128];
  float vth[16];
  signed char ins[128];
  signed char Nst;
  signed char swm[16];
  signed char sw16to1[16];
  float VminDisFlag[16];
  boolean_T BrdBeforePSflag[16];
  struct4_T pwr;
  struct5_T tst;
  struct8_T bit;
} struct3_T;
#endif /* typedef_struct3_T */

#ifndef typedef_struct20_T
#define typedef_struct20_T
typedef struct {
  boolean_T kalmanFlag;
  float Ta;
  struct21_T BatParamsCell[4];
  struct22_T b_struct[4];
} struct20_T;
#endif /* typedef_struct20_T */

#ifndef typedef_struct0_T
#define typedef_struct0_T
typedef struct {
  struct1_T Nmax;
  struct2_T run;
  struct3_T seq;
  struct10_T ser;
  struct13_T str;
  struct15_T ins;
  struct20_T klm;
  struct23_T bat;
  struct24_T brd;
  struct27_T cnfg;
  struct28_T files;
} struct0_T;
#endif /* typedef_struct0_T */

#endif
/*
 * File trailer for Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h
 *
 * [EOF]
 */
