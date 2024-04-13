/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: datetime.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 17-Mar-2024 11:09:30
 */

/* Include Files */
#include "datetime.h"
#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_rtwutil.h"
#include "floor.h"
#include "getLocalTime.h"
#include "minus.h"
#include "plus.h"
#include "rt_nonfinite.h"
#include "split.h"
#include "times.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : void
 * Return Type  : creal_T
 */
creal_T datetime_datetime(void)
{
  creal_T dc;
  creal_T this_data;
  double c_tm_hour;
  double c_tm_mday;
  double c_tm_min;
  double c_tm_mon;
  double c_tm_year;
  double check;
  double fracSecs;
  double second;
  boolean_T expl_temp;
  check = getLocalTime(&second, &c_tm_min, &c_tm_hour, &c_tm_mday, &c_tm_mon,
                       &c_tm_year, &expl_temp);
  fracSecs = check / 1.0E+6;
  check = (((((c_tm_year + c_tm_mon) + c_tm_mday) + c_tm_hour) + c_tm_min) +
           second) +
          fracSecs;
  if ((!rtIsInf(check)) && (!rtIsNaN(check))) {
    if ((c_tm_mon < 1.0) || (c_tm_mon > 12.0)) {
      check = floor((c_tm_mon - 1.0) / 12.0);
      c_tm_year += check;
      c_tm_mon = ((c_tm_mon - 1.0) - check * 12.0) + 1.0;
    }
    if (c_tm_mon < 3.0) {
      c_tm_year--;
      c_tm_mon += 9.0;
    } else {
      c_tm_mon -= 3.0;
    }
    if ((fracSecs < 0.0) || (fracSecs >= 1000.0)) {
      check = floor(fracSecs / 1000.0);
      second += check;
      fracSecs -= check * 1000.0;
    }
    dc.re = ((((((365.0 * c_tm_year + floor(c_tm_year / 4.0)) -
                 floor(c_tm_year / 100.0)) +
                floor(c_tm_year / 400.0)) +
               floor((153.0 * c_tm_mon + 2.0) / 5.0)) +
              c_tm_mday) +
             60.0) -
            719529.0;
    dc.im = 0.0;
    this_data =
        plus(plus(plus(times(dc), (60.0 * c_tm_hour + c_tm_min) * 60000.0),
                  second * 1000.0),
             fracSecs);
  } else {
    this_data.re = check;
    this_data.im = 0.0;
  }
  return this_data;
}

/*
 * Arguments    : const creal_T this_data
 *                double yout[6]
 * Return Type  : void
 */
void datetime_datevec(const creal_T this_data, double yout[6])
{
  creal_T da;
  creal_T msOfDay;
  creal_T r;
  double ahi;
  double bb;
  double c;
  double mo;
  double secs;
  double shi;
  double slo;
  long long i1;
  int i;
  int ia_quot;
  int ic_quot;
  int isecs;
  int qY;
  r.re = this_data.re / 8.64E+7;
  da = split(r.re);
  c = r.re * 8.64E+7;
  slo = da.im * 8.64E+7 + (da.re * 8.64E+7 - c);
  if (rtIsNaN(slo)) {
    slo = 0.0;
  }
  shi = 0.0;
  ahi = 0.0;
  if (this_data.re != c) {
    shi = this_data.re - c;
    bb = shi - this_data.re;
    ahi = (this_data.re - (shi - bb)) - (c + bb);
    if (rtIsNaN(ahi)) {
      ahi = 0.0;
    }
  }
  shi = (shi + 0.0 * this_data.im) - 0.0 * slo;
  ahi = (ahi + this_data.im) - slo;
  shi = (shi + ahi) / 8.64E+7;
  bb = 0.0;
  ahi = r.re;
  if (shi != 0.0) {
    ahi = r.re + shi;
    bb = shi - (ahi - r.re);
  }
  if (rtIsNaN(bb)) {
    bb = 0.0;
  }
  r.re = ahi;
  r.im = bb;
  r = b_floor(r);
  mo = r.re + r.im;
  msOfDay = minus(this_data, times(r));
  r.re = msOfDay.re / 1000.0;
  da = split(r.re);
  c = r.re * 1000.0;
  slo = da.im * 1000.0 + (da.re * 1000.0 - c);
  if (rtIsNaN(slo)) {
    slo = 0.0;
  }
  shi = 0.0;
  ahi = 0.0;
  if (msOfDay.re != c) {
    shi = msOfDay.re - c;
    bb = shi - msOfDay.re;
    ahi = (msOfDay.re - (shi - bb)) - (c + bb);
    if (rtIsNaN(ahi)) {
      ahi = 0.0;
    }
  }
  shi = (shi + 0.0 * msOfDay.im) - 0.0 * slo;
  ahi = (ahi + msOfDay.im) - slo;
  shi = (shi + ahi) / 1000.0;
  bb = 0.0;
  ahi = r.re;
  if (shi != 0.0) {
    ahi = r.re + shi;
    bb = shi - (ahi - r.re);
  }
  if (rtIsNaN(bb)) {
    bb = 0.0;
  }
  r.re = ahi;
  r.im = bb;
  da = b_floor(r);
  r = minus(r, da);
  secs = da.re + da.im;
  if (((mo + 719529.0) - 61.0 >= 0.0) &&
      ((mo + 719529.0) - 61.0 <= 2.147483647E+9)) {
    bb = rt_roundd_snf((mo + 719529.0) - 61.0);
    if (bb < 2.147483648E+9) {
      if (bb >= -2.147483648E+9) {
        i = (int)bb;
      } else {
        i = MIN_int32_T;
      }
    } else if (bb >= 2.147483648E+9) {
      i = MAX_int32_T;
    } else {
      i = 0;
    }
    isecs = i - 146097 * (int)((unsigned int)i / 146097U);
    slo = (double)isecs / 36524.0;
    if (slo < 0.0) {
      slo = ceil(slo);
    } else {
      slo = floor(slo);
    }
    ic_quot = (int)slo;
    qY = isecs - 36524 * (isecs / 36524);
    if (slo > 3.0) {
      ic_quot = 3;
      if (isecs < -2147374076) {
        qY = MIN_int32_T;
      } else {
        qY = isecs - 109572;
      }
    }
    slo = (double)qY / 1461.0;
    if (slo < 0.0) {
      slo = ceil(slo);
    } else {
      slo = floor(slo);
    }
    isecs = qY - 1461 * (qY / 1461);
    shi = (double)isecs / 365.0;
    if (shi < 0.0) {
      shi = ceil(shi);
    } else {
      shi = floor(shi);
    }
    ia_quot = (int)shi;
    qY = isecs - 365 * (isecs / 365);
    if (shi > 3.0) {
      ia_quot = 3;
      if (isecs < -2147482553) {
        qY = MIN_int32_T;
      } else {
        qY = isecs - 1095;
      }
    }
    slo = ((floor((double)i / 146097.0) * 400.0 + (double)ic_quot * 100.0) +
           slo * 4.0) +
          (double)ia_quot;
    i1 = qY * 5LL;
    if (i1 > 2147483647LL) {
      i1 = 2147483647LL;
    } else if (i1 < -2147483648LL) {
      i1 = -2147483648LL;
    }
    if ((int)i1 > 2147483339) {
      isecs = MAX_int32_T;
    } else {
      isecs = (int)i1 + 308;
    }
    shi = (double)isecs / 153.0;
    if (shi < 0.0) {
      shi = ceil(shi);
    } else {
      shi = floor(shi);
    }
    mo = shi - 2.0;
    i1 = ((int)shi + 2) * 153LL;
    if (i1 > 2147483647LL) {
      i1 = 2147483647LL;
    } else if (i1 < -2147483648LL) {
      i1 = -2147483648LL;
    }
    shi = (double)(int)i1 / 5.0;
    if (shi < 0.0) {
      shi = ceil(shi);
    } else {
      shi = floor(shi);
    }
    i = (int)shi;
    if ((qY >= 0) && (i < qY - MAX_int32_T)) {
      qY = MAX_int32_T;
    } else if ((qY < 0) && (i > qY - MIN_int32_T)) {
      qY = MIN_int32_T;
    } else {
      qY -= i;
    }
    if (qY > 2147483525) {
      qY = MAX_int32_T;
    } else {
      qY += 122;
    }
    if (qY > 2147483646) {
      qY = MAX_int32_T;
    } else {
      qY++;
    }
    bb = qY;
  } else {
    slo = floor(((mo + 719529.0) - 61.0) / 146097.0);
    bb = ((mo + 719529.0) - 61.0) - slo * 146097.0;
    c = floor(bb / 36524.0);
    if (c > 3.0) {
      c = 3.0;
    }
    ahi = bb - c * 36524.0;
    shi = floor(ahi / 1461.0);
    ahi -= shi * 1461.0;
    bb = floor(ahi / 365.0);
    if (bb > 3.0) {
      bb = 3.0;
    }
    ahi -= bb * 365.0;
    slo = ((slo * 400.0 + c * 100.0) + shi * 4.0) + bb;
    shi = floor((ahi * 5.0 + 308.0) / 153.0);
    mo = shi - 2.0;
    bb = ((ahi - floor(((shi - 2.0) + 4.0) * 153.0 / 5.0)) + 122.0) + 1.0;
  }
  if (mo > 9.0) {
    slo++;
    mo = (mo + 2.0) - 11.0;
  } else {
    mo = (mo + 2.0) + 1.0;
  }
  if ((secs >= 0.0) && (secs <= 2.147483647E+9)) {
    isecs = (int)rt_roundd_snf(secs);
    ia_quot = isecs - 3600 * (isecs / 3600);
    shi = (double)ia_quot / 60.0;
    if (shi < 0.0) {
      shi = ceil(shi);
    } else {
      shi = floor(shi);
    }
    c = floor((double)isecs / 3600.0);
    ahi = ia_quot - 60 * (ia_quot / 60);
  } else {
    c = floor(secs / 3600.0);
    shi = floor((secs - 3600.0 * c) / 60.0);
    ahi = secs - 60.0 * shi;
  }
  ahi += r.re + r.im;
  if (ahi == 60.0) {
    ahi = 59.999999999999993;
  }
  if ((this_data.re == rtInf) && (this_data.im == 0.0)) {
    slo = rtInf;
  } else if ((this_data.re == rtMinusInf) && (this_data.im == 0.0)) {
    slo = rtMinusInf;
  }
  yout[0] = slo;
  yout[1] = mo;
  yout[2] = bb;
  yout[3] = c;
  yout[4] = shi;
  yout[5] = ahi;
}

/*
 * File trailer for datetime.c
 *
 * [EOF]
 */
