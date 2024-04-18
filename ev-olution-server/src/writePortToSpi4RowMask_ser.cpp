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
  const unsigned char dummy[] = { 15, 1, 15, 1, 15, 1 };
  const unsigned char com_N_maskClose[] = {
    0xFF, 0x7E, 0xFF, 0xFF, 0xFF, 0xFF, //%comN2 Port4 chip1,comN1 Port11 chip1
    0xFF, 0xBF, 0xFF, 0xFF, 0xFF, 0xBF, //%comP1 Port18 chip1 ,comP2 Port18 chip3
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

  int k_row;
  unsigned char u;
  unsigned char oldPortSpiRow[24];
  byte out123[6];
  for (int k_row = 0; k_row < 4; k_row++)
  {
    spi_lib.read((byte*)&PortSpiRow[k_row * 6], &oldPortSpiRow[k_row * 6], 6);
  }

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
