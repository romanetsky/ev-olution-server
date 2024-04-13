/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: pause.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 17-Mar-2024 11:09:30
 */

/* Include Files */
#include "pause.h"
#include "rt_nonfinite.h"
#include "coder_posix_time.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : float varargin_1
 * Return Type  : void
 */
void b_pause(float varargin_1)
{
  coderTimespec b_timespec;
  float delay;
  float delayInt;
  delay = varargin_1;
  if ((varargin_1 < 0.0F) || rtIsNaNF(varargin_1)) {
    delay = 0.0F;
  }
  delayInt = (float)floor(delay);
  if (delayInt > 4.294967295E+9) {
    delayInt = 4.2949673E+9F;
  }
  delay -= delayInt;
  b_timespec.tv_sec = delayInt;
  if (delay > 0.0F) {
    b_timespec.tv_nsec = (float)floor(delay * 1.0E+9F);
  } else {
    b_timespec.tv_nsec = 0.0;
  }
  coderTimeSleep(&b_timespec);
}

/*
 * Arguments    : double varargin_1
 * Return Type  : void
 */
void pause(double varargin_1)
{
  coderTimespec b_timespec;
  double delay;
  double delayInt;
  delay = varargin_1;
  if (rtIsNaN(varargin_1)) {
    delay = 0.0;
  }
  delayInt = floor(delay);
  delay -= delayInt;
  if (delay > 0.0) {
    b_timespec.tv_nsec = floor(delay * 1.0E+9);
  } else {
    b_timespec.tv_nsec = 0.0;
  }
  b_timespec.tv_sec = delayInt;
  coderTimeSleep(&b_timespec);
}

/*
 * File trailer for pause.c
 *
 * [EOF]
 */
