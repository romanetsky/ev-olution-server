/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: ControlKp184.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 17-Mar-2024 11:09:30
 */

/* Include Files */
#include "ControlKp184.h"
#include "rand.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * %CONTROLKP184 Summary of this function goes here
 *  %   Detailed explanation goes here
 *  kp184Device.comPort       = 'COM4';
 *  kp184Device.baudRate      = 9600;%2400, 4800,9600, 18200, 38400, 57600 ,
 * 115200 kp184Device.deviceAddress = 1;%1-250 s =
 * serialport(kp184Device.comPort,kp184Device.baudRate);
 *  configureTerminator(s,"CR/LF")
 *  end
 *
 * Arguments    : double is_on_data[]
 *                int is_on_size[2]
 *                double what_mode_data[]
 *                int what_mode_size[2]
 *                double VmV_data[]
 *                int VmV_size[2]
 *                double ImA_data[]
 *                int ImA_size[2]
 * Return Type  : void
 */
void ControlKp184(double is_on_data[], int is_on_size[2],
                  double what_mode_data[], int what_mode_size[2],
                  double VmV_data[], int VmV_size[2], double ImA_data[],
                  int ImA_size[2])
{
  double ImA;
  double VmV;
  /* 5 Read */
  /*  0-off, 1-on */
  VmV = d_rand() * 2.5 + 2.0;
  ImA = d_rand() * 5.0;
  is_on_size[0] = 1;
  is_on_size[1] = 1;
  is_on_data[0] = 0.0;
  what_mode_size[0] = 1;
  what_mode_size[1] = 1;
  what_mode_data[0] = 0.0;
  VmV_size[0] = 1;
  VmV_size[1] = 1;
  VmV_data[0] = VmV;
  ImA_size[0] = 1;
  ImA_size[1] = 1;
  ImA_data[0] = ImA;
}

/*
 * File trailer for ControlKp184.c
 *
 * [EOF]
 */
