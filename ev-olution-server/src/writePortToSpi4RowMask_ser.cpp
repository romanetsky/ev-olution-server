/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: writePortToSpi4RowMask_ser.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 17-Apr-2024 18:20:59
 */

/* Include Files */
#include "writePortToSpi4RowMask_ser.h"
#include "rt_nonfinite.h"
#include "coder_posix_time.h"
#include <stdio.h>
#include "spi_lib.h"

extern SpiLib spi_lib;

void sort(unsigned char x[4], int idx[4])
{
  int b_i1;
  int i;
  int i1;
  int i2;
  int i3;
  int i4;
  signed char idx4[4];
  unsigned char x4[4];
  signed char perm_idx_0;
  signed char perm_idx_1;
  signed char perm_idx_2;
  signed char perm_idx_3;
  idx4[0] = 1;
  idx4[1] = 2;
  idx4[2] = 3;
  idx4[3] = 4;
  x4[0] = x[0];
  x4[1] = x[1];
  x4[2] = x[2];
  x4[3] = x[3];
  if (x[0] <= x[1]) {
    i1 = 1;
    i2 = 2;
  } else {
    i1 = 2;
    i2 = 1;
  }
  if (x[2] <= x[3]) {
    i3 = 3;
    i4 = 4;
  } else {
    i3 = 4;
    i4 = 3;
  }
  i = x4[i3 - 1];
  b_i1 = x4[i1 - 1];
  if (b_i1 <= i) {
    b_i1 = x4[i2 - 1];
    if (b_i1 <= i) {
      perm_idx_0 = (signed char)i1;
      perm_idx_1 = (signed char)i2;
      perm_idx_2 = (signed char)i3;
      perm_idx_3 = (signed char)i4;
    } else if (b_i1 <= x4[i4 - 1]) {
      perm_idx_0 = (signed char)i1;
      perm_idx_1 = (signed char)i3;
      perm_idx_2 = (signed char)i2;
      perm_idx_3 = (signed char)i4;
    } else {
      perm_idx_0 = (signed char)i1;
      perm_idx_1 = (signed char)i3;
      perm_idx_2 = (signed char)i4;
      perm_idx_3 = (signed char)i2;
    }
  } else {
    i = x4[i4 - 1];
    if (b_i1 <= i) {
      if (x4[i2 - 1] <= i) {
        perm_idx_0 = (signed char)i3;
        perm_idx_1 = (signed char)i1;
        perm_idx_2 = (signed char)i2;
        perm_idx_3 = (signed char)i4;
      } else {
        perm_idx_0 = (signed char)i3;
        perm_idx_1 = (signed char)i1;
        perm_idx_2 = (signed char)i4;
        perm_idx_3 = (signed char)i2;
      }
    } else {
      perm_idx_0 = (signed char)i3;
      perm_idx_1 = (signed char)i4;
      perm_idx_2 = (signed char)i1;
      perm_idx_3 = (signed char)i2;
    }
  }
  idx[0] = idx4[perm_idx_0 - 1];
  idx[1] = idx4[perm_idx_1 - 1];
  idx[2] = idx4[perm_idx_2 - 1];
  idx[3] = idx4[perm_idx_3 - 1];
  x[0] = x4[perm_idx_0 - 1];
  x[1] = x4[perm_idx_1 - 1];
  x[2] = x4[perm_idx_2 - 1];
  x[3] = x4[perm_idx_3 - 1];
}
/* Function Definitions */
/*
 * UNTITLED Summary of this function goes here
 *    Detailed explanation goes here
 *  if coder.target('MATLAB')
 *
 * Arguments    : const unsigned char PortSpiRow[24]
 * Return Type  : void
 */
void writePortToSpi4RowMask_ser(const unsigned char PortSpiRow[24])
{
  static int call_counter = 0;
  ++call_counter;
  static const unsigned char com_N_maskClose[24] = {
      MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 126U,
      191U,        MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
      MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
      MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
      MAX_uint8_T, 191U,        MAX_uint8_T, MAX_uint8_T};
  static const unsigned char uv[6] = {15U, 1U, 15U, 1U, 15U, 1U};
  coderTimespec b_timespec;
  int iidx[4];
  int disConAllSwitch_tmp;
  int i;
  int i1;
  int i2;
  int k_row;
  int oldPortSpiRow_tmp;
  unsigned char dataDummy[30];
  unsigned char RmsgData[24];
  unsigned char disConAllSwitch[24];
  unsigned char oldPortSpiRow[24];
  unsigned char y[4];
  for (i = 0; i < 24; i++) {
    disConAllSwitch[i] = PortSpiRow[i];
  }
  for (i = 0; i < 3; i++) {
    disConAllSwitch_tmp = ((i << 1) + 1) << 2;
    disConAllSwitch[disConAllSwitch_tmp] = 0U;
    disConAllSwitch[disConAllSwitch_tmp + 1] = 0U;
    disConAllSwitch[disConAllSwitch_tmp + 2] = 0U;
    disConAllSwitch[disConAllSwitch_tmp + 3] = 0U;
  }
  for (i = 0; i < 6; i++) {
    disConAllSwitch_tmp = i << 2;
    dataDummy[5 * i] = disConAllSwitch[disConAllSwitch_tmp];
    dataDummy[5 * i + 1] = disConAllSwitch[disConAllSwitch_tmp + 1];
    dataDummy[5 * i + 2] = disConAllSwitch[disConAllSwitch_tmp + 2];
    dataDummy[5 * i + 3] = disConAllSwitch[disConAllSwitch_tmp + 3];
    dataDummy[5 * i + 4] = uv[i];
  }
  spi_lib.read((byte*)PortSpiRow, RmsgData, 24);
  y[0] = RmsgData[0];
  y[1] = RmsgData[1];
  y[2] = RmsgData[2];
  y[3] = RmsgData[3];
  sort(y, iidx);
  i = iidx[0];
  disConAllSwitch_tmp = iidx[1];
  k_row = iidx[2];
  i1 = iidx[3];
  for (i2 = 0; i2 < 6; i2++) {
    oldPortSpiRow_tmp = i2 << 2;
    oldPortSpiRow[oldPortSpiRow_tmp] = RmsgData[(i + oldPortSpiRow_tmp) - 1];
    oldPortSpiRow[oldPortSpiRow_tmp + 1] =
        RmsgData[(disConAllSwitch_tmp + oldPortSpiRow_tmp) - 1];
    oldPortSpiRow[oldPortSpiRow_tmp + 2] =
        RmsgData[(k_row + oldPortSpiRow_tmp) - 1];
    oldPortSpiRow[oldPortSpiRow_tmp + 3] =
        RmsgData[(i1 + oldPortSpiRow_tmp) - 1];
  }
  for (i = 0; i < 24; i++) {
    RmsgData[i] = (unsigned char)(oldPortSpiRow[i] & com_N_maskClose[i]);
  }
  spi_lib.write(RmsgData, 24);

  spi_lib.write(disConAllSwitch, 24);

  for (i = 0; i < 24; i++) {
    RmsgData[i] = (unsigned char)(PortSpiRow[i] & com_N_maskClose[i]);
  }
  spi_lib.write(RmsgData, 24);

  spi_lib.write((byte*)PortSpiRow, 24);

  /*  if ~coder.target('MATLAB') */
  /*      maskFalg = true; */
  /*  end */
  /*  maskFalg = true; */
  /*  read current */
  /*   chip1     |   chip2     |   chip3 */
  /* %comN2 Port4 chip1,comN1 Port11 chip1 */
  /* %comP1 Port18 chip1 ,comP2 Port18 chip3 */
  /*  */
  /*  for k_row = 1:N_row */
  /*      out123(k_row,:) = readSPI(SPI,PortSpiRow(k_row,1),N_chip); */
  /*      oldPortSpiRow(k_row,1:2:end) = PortSpiRow(k_row,1:2:end); */
  /*      oldPortSpiRow(k_row,2:2:end) = out123(k_row,2:2:end); */
  /*  end */
  /* mask current */
  /* write current mask */
  /*  for k_row = 1:N_row */
  /*      write(SPI,oldPortSpiRowMask(k_row,:)); */
  /*  end */

  /* mask new */
  /* write mask new */
  /*  for k_row = 1:N_row */
  /*      write(SPI0,PortSpiRowMask(k_row,:)); */
  /*  end */
  /* write new */
  /*  for k_row = 1:N_row */
  /*      write(SPI0,PortSpiRow(k_row,:)); */
  /*  end */
  /*  Rmsg = SPI_writeread(SPI0, PortSpiRow); */
  /*  ReadPortSpiRow = RmsgData([[2:4] , 1] ,:); */
  /*  ReadPortSpiRow(:,1:2:end) = ReadPortSpiRow(:,1:2:end);% - 128; */
  /* mask current */
  /* %chip1 Port6 Alert1, chip1 port 7 Alert3, chip1 port 8 Alert8, chip1 port 9
   * Alert6 */
  /* %chip1 Port12 Alert7, chip1 Port13 Alert5, chip 1 port19 NC */
  /* %chip1 Port20-23 NC */
  /* chip1 Port28 Alert1, chip1 Port29 Alert2 */
  /*  if sum(sum(sum(abs(double(PortSpiRow)-double(ReadPortSpiRow)))))>0 */
  /*  end */
}

/*
 * File trailer for writePortToSpi4RowMask_ser.c
 *
 * [EOF]
 */
