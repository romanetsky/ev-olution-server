/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: dischargeCell_array.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 17-Mar-2024 11:09:30
 */

/* Include Files */
#include "dischargeCell_array.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Definitions */
/*
 * Q = 7;
 *  E0 = 1.2816;
 *  k = 0.0014043;
 *  A = 0.11104;
 *  B = 2.3077;
 * C
 *  Rin = 2e-3;
 *
 * Arguments    : const float paramsBatt[38]
 *                float stateBatt[31]
 *                float Ta
 *                double t
 *                float dt
 *                double i_in
 * Return Type  : void
 */
void b_dischargeCell_array(const float paramsBatt[38], float stateBatt[31],
                           float Ta, double t, float dt, double i_in)
{
  float E0_T;
  float I_chr_avrg;
  float I_dis_avrg;
  float K_T;
  float N_chr;
  float N_dis;
  float Q_Ta;
  float Q_Ta_EOL;
  float Q_Ta_tmp;
  float R_T;
  float R_T_EOL;
  float SOC2dt;
  float SOCdt;
  float SabsIt;
  float T_lp_n;
  float T_n_m1;
  float Tdt;
  float i_lp_n;
  float it;
  float tcycle;
  int Sel;
  /*  stateBatt.i_lp_n; */
  /* no state */
  if (stateBatt[2] >= 0.0F) {
    /* not i_in --> i_lp_n */
    Sel = 0;
    /* discharge */
  } else {
    Sel = 1;
    /* charge */
  }
  /* state */
  /* stateBatt.T; */
  /* stateBatt.it; % it == Q empty, it == 0 full %How much charge was discharge
   * from the battery */
  /*  stateBatt.i_lp_n; */
  /*  i_lp_n_m1 = stateBatt.i_lp_n_m1; */
  /* stateBatt(4);%i_in;%stateBatt.i_n; */
  /*  i_n_m1    = stateBatt.i_n_m1; */
  /* stateBatt.DOD_n; */
  /* stateBatt.DOD_n_m1; */
  /* DOD_n_m2; */
  /* stateBatt.Sel_n_m1; */
  /* stateBatt.eps_n; */
  /* stateBatt.Q_Ta_n; */
  I_dis_avrg = stateBatt[11];
  /* stateBatt.I_dis_avrg; */
  I_chr_avrg = stateBatt[11];
  /* stateBatt.I_chr_avrg; */
  N_chr = stateBatt[12];
  /* stateBatt.N_chr; */
  N_dis = stateBatt[13];
  /* stateBatt.N_dis; */
  T_n_m1 = stateBatt[14];
  /* stateBatt.T_n; */
  /* .T_lp_n_m1; */
  /* stateBatt.T_lp_n; */
  /*  */
  /*  t         = stateBatt(19);% */
  /* : 0.0137 */
  /* : 3.3419 */
  /* : 1 */
  tcycle = stateBatt[22];
  /* stateBatt.tcycle; */
  /*  */
  SOCdt = stateBatt[24];
  /* stateBatt.SOCdt; */
  SOC2dt = stateBatt[25];
  /* stateBatt.SOC2dt; */
  /* stateBatt.L_n; */
  SabsIt = stateBatt[27];
  /* stateBatt.SabsIt; */
  Tdt = stateBatt[28];
  /* stateBatt.Tdt; */
  /*  */
  /* params */
  /* paramsBatt.Tref; */
  /* paramsBatt.R_Tref; */
  /*  */
  /* paramsBatt.R_Tref_EOL; */
  /* paramsBatt.E0_Tref; */
  /* paramsBatt.dE_dT; */
  /* paramsBatt.K_Tref; */
  /* paramsBatt.dK_di; */
  /* paramsBatt.i_K_ref; */
  /* paramsBatt.alpha_K; */
  /* paramsBatt.Q_Ta_ref; */
  /* paramsBatt.Q_Ta_ref_EOL; */
  /* paramsBatt.dQ_dT; */
  /* paramsBatt.beta_R; */
  /* dt*paramsBatt.k_lp_per; % k_lp = 0.1 */
  /* paramsBatt.A; */
  /* paramsBatt.B; */
  /* paramsBatt.C; */
  /* paramsBatt.Rth; */
  /* aging params */
  /* paramsBatt.psi    ; */
  /* paramsBatt.gamma1; */
  /* paramsBatt.gamma2; */
  /* paramsBatt.H     ; */
  /* paramsBatt.xi    */
  /*  */
  /* : 3.6600e-05 */
  /* : 0.7170 */
  /* : 0.9160 */
  /* : 0.0693 */
  /* : 3.0000e+09 */
  /* : 0 */
  /* : 2.2500 */
  /* : 0.0371 */
  /* : 0 */
  /* : 0.0050 */
  /* : 0 */
  /*  Rth      = paramsBatt.Rth; */
  /* paramsBatt(16);%paramsBatt.Rc;% k_lp = dt/RC; of first order Low Pass
   * Filter */
  /*  k_lp = dt/RC;% */
  /* paramsBatt(17);%dt/paramsBatt.tc; */
  /*  R_Tref_EOL_AM   = paramsBatt(4);%paramsBatt.R_Tref_EOL_AM; */
  /*  Q_Ta_ref_EOL_AM   = paramsBatt(13);%paramsBatt.Q_Ta_ref_EOL_AM; */
  /*  k_lp     = dt/paramsBatt(16);%paramsBatt.Rc;% k_lp = dt/RC; of first order
   * Low Pass Filter */
  /*  k_lp = dt/RC;% */
  /*  k_tc     = paramsBatt(17);%dt/paramsBatt.tc; */
  /* calc average current */
  if (Sel != 0) {
    /* charge */
    I_chr_avrg =
        ((float)fabs(stateBatt[11]) * stateBatt[12] + (float)fabs(i_in)) /
        (stateBatt[12] + 1.0F);
    N_chr = stateBatt[12] + 1.0F;
  } else {
    /* discharge */
    I_dis_avrg =
        ((float)fabs(stateBatt[11]) * stateBatt[13] + (float)fabs(i_in)) /
        (stateBatt[13] + 1.0F);
    N_dis = stateBatt[13] + 1.0F;
  }
  /*  i0 = 1.3; */
  /*  t = [0:1e-3:5.2]; */
  /* i first order low pass filter */
  /*  i_n       = i;     */
  /* i(n-1) = i(n) */
  i_lp_n = stateBatt[2] + dt / paramsBatt[15] * ((float)i_in - stateBatt[2]);
  /* i integral */
  it = stateBatt[1] + (float)i_in * dt;
  /* Temreture */
  E0_T = paramsBatt[4] + paramsBatt[5] * (stateBatt[0] - paramsBatt[0]);
  /*  K_T  = K_Tref   * exp(alpha_K*(1/T-1/Tref)); */
  T_lp_n = 1.0F / stateBatt[0] - 1.0F / paramsBatt[0];
  K_T = (paramsBatt[6] + paramsBatt[7] * ((float)i_in - paramsBatt[8])) *
        (float)exp(paramsBatt[9] * T_lp_n);
  Q_Ta_tmp = paramsBatt[13] * (Ta - paramsBatt[0]);
  Q_Ta = paramsBatt[10] + Q_Ta_tmp;
  T_lp_n = (float)exp(paramsBatt[14] * T_lp_n);
  R_T = paramsBatt[1] * T_lp_n;
  Q_Ta_EOL = paramsBatt[12] + Q_Ta_tmp;
  R_T_EOL = paramsBatt[3] * T_lp_n;
  /*   DOD calculation */
  /*  Aging */
  switch ((int)paramsBatt[26]) {
  case 1:
    /*          SOC_n = stateBatt.SOC_n; */
    SabsIt = stateBatt[27] + (float)fabs(i_in) * dt;
    tcycle = stateBatt[22] + dt;
    SOCdt = stateBatt[24] + stateBatt[21] * dt;
    SOC2dt = stateBatt[25] + stateBatt[21] * stateBatt[21] * dt;
    Tdt = stateBatt[28] + stateBatt[0] * dt;
    Q_Ta -= stateBatt[26] * (Q_Ta - (paramsBatt[35] + Q_Ta_tmp));
    R_T += stateBatt[26] * (-R_T + paramsBatt[34] * T_lp_n);
    break;
  case 0:
    Q_Ta -= stateBatt[8] * (Q_Ta - Q_Ta_EOL);
    R_T += stateBatt[8] * (-R_T + R_T_EOL);
    break;
  case 2:
    SabsIt = stateBatt[27] + (float)fabs(i_in) * dt;
    Q_Ta -= stateBatt[8] * (Q_Ta - Q_Ta_EOL);
    R_T += stateBatt[8] * (-R_T + R_T_EOL);
    break;
  }
  if (Sel != 0) {
    /*  charge */
    /*      Eb = E0_T - K_T*Q_Ta_n./(Q_Ta_n - it).*(i_lp)- K_T*Q_Ta_n./(Q_Ta_n -
     * it).*(it)+A*exp(-B*it)-C*it;%from Q_Ta_n/10 added - 10/04/2021  */
    /* Eb = E0_T - K_T*Q_Ta_n./(Q_Ta_n*0.1 + it).*(i_lp)- K_T*Q_Ta_n./(Q_Ta_n -
     * it).*(it)+A*exp(-B*it)-C*it;%from +Q_Ta_n/10 added - 15/06/2021  */
    T_lp_n = K_T * Q_Ta;
    Q_Ta_EOL = (((E0_T - T_lp_n / (-Q_Ta * 0.1F + it) * i_lp_n) -
                 T_lp_n / (Q_Ta - it) * it) +
                paramsBatt[17] * (float)exp(-paramsBatt[18] * it)) -
               paramsBatt[19] * it;
    /* from +Q_Ta_n/10 added - 31/01/2022  */
    /*     Eb = E0_T - K_T*Q_Ta_n./(-Q_Ta_n/10 + it).*(i_lp)-
     * K_T*Q_Ta_n./(Q_Ta_n - it).*(it)+A*exp(-B*it)-C*it;%from Q_Ta_n/10 added -
     * 10/04/2021  */
    /* Electro-thermal analysis of Lithium Iron Phosphate battery for electric
     * vehicles% */
    R_T_EOL = Q_Ta_EOL - (float)i_in * R_T;
  } else {
    /*  discharge    */
    Q_Ta_EOL = ((E0_T - K_T * Q_Ta / (Q_Ta - it) * (i_lp_n + it)) +
                paramsBatt[17] * (float)exp(-paramsBatt[18] * it)) -
               paramsBatt[19] * it;
    R_T_EOL = Q_Ta_EOL - (float)i_in * R_T;
  }
  /* i first order low pass filter Tempreture */
  /* Ploss      = (E0_T - Eb)*(i_in) + (Eb - Vb)*abs(i_in) +
   * dE_dT*abs(i_in)*T;%NADAV from (E0_T - Vb)*i_in + dE_dT*i_in*T */
  /* NADAV (1985 Bernardi) from (E0_T - Vb)*i_in + dE_dT*i_in*T */
  stateBatt[14] = ((K_T * Q_Ta / (-Q_Ta * 0.1F + it) * (i_lp_n * i_lp_n) +
                    R_T * (float)(i_in * i_in)) +
                   paramsBatt[5] * (float)fabs(i_in) * stateBatt[0]) *
                      paramsBatt[20] +
                  Ta;
  /* Ploss+Ta */
  stateBatt[15] = stateBatt[16];
  T_lp_n = stateBatt[16] + dt / paramsBatt[16] * (T_n_m1 - stateBatt[16]);
  /*  simple R switch */
  stateBatt[17] = R_T_EOL - (float)i_in * paramsBatt[36];
  /* Update state cell */
  /* state */
  stateBatt[0] = T_lp_n;
  stateBatt[1] = it;
  stateBatt[2] = i_lp_n;
  /*  stateBatt.i_lp_n_m1 = i_lp_n_m1; */
  stateBatt[3] = (float)i_in;
  /*  stateBatt.i_n_m1    = i_n_m1; */
  stateBatt[7] = (float)Sel;
  /*  N_n       = N_n; */
  stateBatt[9] = Q_Ta;
  stateBatt[10] = I_dis_avrg;
  stateBatt[11] = I_chr_avrg;
  stateBatt[12] = N_chr;
  stateBatt[13] = N_dis;
  stateBatt[16] = T_lp_n;
  stateBatt[18] = (float)t;
  stateBatt[19] = R_T;
  stateBatt[20] = Q_Ta_EOL;
  stateBatt[21] = 1.0F - it / Q_Ta;
  /* SOC_n */
  stateBatt[22] = tcycle;
  stateBatt[24] = SOCdt;
  stateBatt[25] = SOC2dt;
  stateBatt[27] = SabsIt;
  stateBatt[28] = Tdt;
  /*  Ploss = (E0_T - Vb)*i + dE_dT*i*T; */
  /*  T_t = (Ploss*Rth + Ta)*exp(-t/tc); */
  /*  Eb = E0 - k*Q./(Q - i0*t).*(i0+i0*t)+A*exp(-B*i0*t); */
  /*  Vb = Eb - i0*Rin; */
  /*  figure;plot(t,Vb,'.');grid; */
  /*   */
  /*  i0 = [6.5;13;32.5]; */
  /*  Eb = E0 - k*Q./(Q - i0*t).*(i0+i0*t)+A*exp(-B*i0*t); */
  /*  Vb = Eb - i0*Rin; */
  /*  figure;plot(t,Vb,'.');grid; */
}

/*
 * Q = 7;
 *  E0 = 1.2816;
 *  k = 0.0014043;
 *  A = 0.11104;
 *  B = 2.3077;
 * C
 *  Rin = 2e-3;
 *
 * Arguments    : const float paramsBatt[38]
 *                float stateBatt[31]
 *                float Ta
 * Return Type  : void
 */
void dischargeCell_array(const float paramsBatt[38], float stateBatt[31],
                         float Ta)
{
  float E0_T;
  float I_chr_avrg;
  float I_dis_avrg;
  float K_T;
  float N_chr;
  float N_dis;
  float Q_Ta;
  float Q_Ta_EOL;
  float Q_Ta_tmp;
  float R_T;
  float R_T_EOL;
  float SOC2dt;
  float SOCdt;
  float T_lp_n;
  float T_n_m1;
  float Tdt;
  float i_lp_n;
  float tcycle;
  int Sel;
  /*  stateBatt.i_lp_n; */
  /* no state */
  if (stateBatt[2] >= 0.0F) {
    /* not i_in --> i_lp_n */
    Sel = 0;
    /* discharge */
  } else {
    Sel = 1;
    /* charge */
  }
  /* state */
  /* stateBatt.T; */
  /* stateBatt.it; % it == Q empty, it == 0 full %How much charge was discharge
   * from the battery */
  /*  stateBatt.i_lp_n; */
  /*  i_lp_n_m1 = stateBatt.i_lp_n_m1; */
  /* stateBatt(4);%i_in;%stateBatt.i_n; */
  /*  i_n_m1    = stateBatt.i_n_m1; */
  /* stateBatt.DOD_n; */
  /* stateBatt.DOD_n_m1; */
  /* DOD_n_m2; */
  /* stateBatt.Sel_n_m1; */
  /* stateBatt.eps_n; */
  /* stateBatt.Q_Ta_n; */
  I_dis_avrg = stateBatt[11];
  /* stateBatt.I_dis_avrg; */
  I_chr_avrg = stateBatt[11];
  /* stateBatt.I_chr_avrg; */
  N_chr = stateBatt[12];
  /* stateBatt.N_chr; */
  N_dis = stateBatt[13];
  /* stateBatt.N_dis; */
  T_n_m1 = stateBatt[14];
  /* stateBatt.T_n; */
  /* .T_lp_n_m1; */
  /* stateBatt.T_lp_n; */
  /*  */
  /*  t         = stateBatt(19);% */
  /* : 0.0137 */
  /* : 3.3419 */
  /* : 1 */
  tcycle = stateBatt[22];
  /* stateBatt.tcycle; */
  /*  */
  SOCdt = stateBatt[24];
  /* stateBatt.SOCdt; */
  SOC2dt = stateBatt[25];
  /* stateBatt.SOC2dt; */
  /* stateBatt.L_n; */
  /* stateBatt.SabsIt; */
  Tdt = stateBatt[28];
  /* stateBatt.Tdt; */
  /*  */
  /* params */
  /* paramsBatt.Tref; */
  /* paramsBatt.R_Tref; */
  /*  */
  /* paramsBatt.R_Tref_EOL; */
  /* paramsBatt.E0_Tref; */
  /* paramsBatt.dE_dT; */
  /* paramsBatt.K_Tref; */
  /* paramsBatt.dK_di; */
  /* paramsBatt.i_K_ref; */
  /* paramsBatt.alpha_K; */
  /* paramsBatt.Q_Ta_ref; */
  /* paramsBatt.Q_Ta_ref_EOL; */
  /* paramsBatt.dQ_dT; */
  /* paramsBatt.beta_R; */
  /* dt*paramsBatt.k_lp_per; % k_lp = 0.1 */
  /* paramsBatt.A; */
  /* paramsBatt.B; */
  /* paramsBatt.C; */
  /* paramsBatt.Rth; */
  /* aging params */
  /* paramsBatt.psi    ; */
  /* paramsBatt.gamma1; */
  /* paramsBatt.gamma2; */
  /* paramsBatt.H     ; */
  /* paramsBatt.xi    */
  /*  */
  /* : 3.6600e-05 */
  /* : 0.7170 */
  /* : 0.9160 */
  /* : 0.0693 */
  /* : 3.0000e+09 */
  /* : 0 */
  /* : 2.2500 */
  /* : 0.0371 */
  /* : 0 */
  /* : 0.0050 */
  /* : 0 */
  /*  Rth      = paramsBatt.Rth; */
  /* paramsBatt(16);%paramsBatt.Rc;% k_lp = dt/RC; of first order Low Pass
   * Filter */
  /*  k_lp = dt/RC;% */
  /* paramsBatt(17);%dt/paramsBatt.tc; */
  /*  R_Tref_EOL_AM   = paramsBatt(4);%paramsBatt.R_Tref_EOL_AM; */
  /*  Q_Ta_ref_EOL_AM   = paramsBatt(13);%paramsBatt.Q_Ta_ref_EOL_AM; */
  /*  k_lp     = dt/paramsBatt(16);%paramsBatt.Rc;% k_lp = dt/RC; of first order
   * Low Pass Filter */
  /*  k_lp = dt/RC;% */
  /*  k_tc     = paramsBatt(17);%dt/paramsBatt.tc; */
  /* calc average current */
  if (Sel != 0) {
    /* charge */
    I_chr_avrg =
        (float)fabs(stateBatt[11]) * stateBatt[12] / (stateBatt[12] + 1.0F);
    N_chr = stateBatt[12] + 1.0F;
  } else {
    /* discharge */
    I_dis_avrg =
        (float)fabs(stateBatt[11]) * stateBatt[13] / (stateBatt[13] + 1.0F);
    N_dis = stateBatt[13] + 1.0F;
  }
  /*  i0 = 1.3; */
  /*  t = [0:1e-3:5.2]; */
  /* i first order low pass filter */
  /*  i_n       = i;     */
  /* i(n-1) = i(n) */
  i_lp_n =
      stateBatt[2] + 0.000277777785F / paramsBatt[15] * (0.0F - stateBatt[2]);
  /* i integral */
  /* Temreture */
  E0_T = paramsBatt[4] + paramsBatt[5] * (stateBatt[0] - paramsBatt[0]);
  /*  K_T  = K_Tref   * exp(alpha_K*(1/T-1/Tref)); */
  T_lp_n = 1.0F / stateBatt[0] - 1.0F / paramsBatt[0];
  K_T = (paramsBatt[6] + paramsBatt[7] * (0.0F - paramsBatt[8])) *
        (float)exp(paramsBatt[9] * T_lp_n);
  Q_Ta_tmp = paramsBatt[13] * (Ta - paramsBatt[0]);
  Q_Ta = paramsBatt[10] + Q_Ta_tmp;
  T_lp_n = (float)exp(paramsBatt[14] * T_lp_n);
  R_T = paramsBatt[1] * T_lp_n;
  Q_Ta_EOL = paramsBatt[12] + Q_Ta_tmp;
  R_T_EOL = paramsBatt[3] * T_lp_n;
  /*   DOD calculation */
  /*  Aging */
  switch ((int)paramsBatt[26]) {
  case 1:
    /*          SOC_n = stateBatt.SOC_n; */
    tcycle = stateBatt[22] + 0.000277777785F;
    SOCdt = stateBatt[24] + stateBatt[21] * 0.000277777785F;
    SOC2dt = stateBatt[25] + stateBatt[21] * stateBatt[21] * 0.000277777785F;
    Tdt = stateBatt[28] + stateBatt[0] * 0.000277777785F;
    Q_Ta -= stateBatt[26] * (Q_Ta - (paramsBatt[35] + Q_Ta_tmp));
    R_T += stateBatt[26] * (-R_T + paramsBatt[34] * T_lp_n);
    break;
  case 0:
    Q_Ta -= stateBatt[8] * (Q_Ta - Q_Ta_EOL);
    R_T += stateBatt[8] * (-R_T + R_T_EOL);
    break;
  case 2:
    Q_Ta -= stateBatt[8] * (Q_Ta - Q_Ta_EOL);
    R_T += stateBatt[8] * (-R_T + R_T_EOL);
    break;
  }
  if (Sel != 0) {
    /*  charge */
    /*      Eb = E0_T - K_T*Q_Ta_n./(Q_Ta_n - it).*(i_lp)- K_T*Q_Ta_n./(Q_Ta_n -
     * it).*(it)+A*exp(-B*it)-C*it;%from Q_Ta_n/10 added - 10/04/2021  */
    /* Eb = E0_T - K_T*Q_Ta_n./(Q_Ta_n*0.1 + it).*(i_lp)- K_T*Q_Ta_n./(Q_Ta_n -
     * it).*(it)+A*exp(-B*it)-C*it;%from +Q_Ta_n/10 added - 15/06/2021  */
    T_lp_n = K_T * Q_Ta;
    Q_Ta_EOL = (((E0_T - T_lp_n / (-Q_Ta * 0.1F + stateBatt[1]) * i_lp_n) -
                 T_lp_n / (Q_Ta - stateBatt[1]) * stateBatt[1]) +
                paramsBatt[17] * (float)exp(-paramsBatt[18] * stateBatt[1])) -
               stateBatt[1] * paramsBatt[19];
    /* from +Q_Ta_n/10 added - 31/01/2022  */
    /*     Eb = E0_T - K_T*Q_Ta_n./(-Q_Ta_n/10 + it).*(i_lp)-
     * K_T*Q_Ta_n./(Q_Ta_n - it).*(it)+A*exp(-B*it)-C*it;%from Q_Ta_n/10 added -
     * 10/04/2021  */
    /* Electro-thermal analysis of Lithium Iron Phosphate battery for electric
     * vehicles% */
    R_T_EOL = Q_Ta_EOL - 0.0F * R_T;
  } else {
    /*  discharge    */
    Q_Ta_EOL =
        ((E0_T - K_T * Q_Ta / (Q_Ta - stateBatt[1]) * (i_lp_n + stateBatt[1])) +
         paramsBatt[17] * (float)exp(-paramsBatt[18] * stateBatt[1])) -
        stateBatt[1] * paramsBatt[19];
    R_T_EOL = Q_Ta_EOL - 0.0F * R_T;
  }
  /* i first order low pass filter Tempreture */
  /* Ploss      = (E0_T - Eb)*(i_in) + (Eb - Vb)*abs(i_in) +
   * dE_dT*abs(i_in)*T;%NADAV from (E0_T - Vb)*i_in + dE_dT*i_in*T */
  /* NADAV (1985 Bernardi) from (E0_T - Vb)*i_in + dE_dT*i_in*T */
  stateBatt[14] =
      ((K_T * Q_Ta / (-Q_Ta * 0.1F + stateBatt[1]) * (i_lp_n * i_lp_n) +
        R_T * 0.0F) +
       paramsBatt[5] * 0.0F * stateBatt[0]) *
          paramsBatt[20] +
      Ta;
  /* Ploss+Ta */
  stateBatt[15] = stateBatt[16];
  T_lp_n = stateBatt[16] +
           0.000277777785F / paramsBatt[16] * (T_n_m1 - stateBatt[16]);
  /*  simple R switch */
  stateBatt[17] = R_T_EOL - 0.0F * paramsBatt[36];
  /* Update state cell */
  /* state */
  stateBatt[0] = T_lp_n;
  stateBatt[2] = i_lp_n;
  /*  stateBatt.i_lp_n_m1 = i_lp_n_m1; */
  stateBatt[3] = 0.0F;
  /*  stateBatt.i_n_m1    = i_n_m1; */
  stateBatt[7] = (float)Sel;
  /*  N_n       = N_n; */
  stateBatt[9] = Q_Ta;
  stateBatt[10] = I_dis_avrg;
  stateBatt[11] = I_chr_avrg;
  stateBatt[12] = N_chr;
  stateBatt[13] = N_dis;
  stateBatt[16] = T_lp_n;
  stateBatt[18] = 0.0F;
  stateBatt[19] = R_T;
  stateBatt[20] = Q_Ta_EOL;
  stateBatt[21] = 1.0F - stateBatt[1] / Q_Ta;
  /* SOC_n */
  stateBatt[22] = tcycle;
  stateBatt[24] = SOCdt;
  stateBatt[25] = SOC2dt;
  stateBatt[28] = Tdt;
  /*  Ploss = (E0_T - Vb)*i + dE_dT*i*T; */
  /*  T_t = (Ploss*Rth + Ta)*exp(-t/tc); */
  /*  Eb = E0 - k*Q./(Q - i0*t).*(i0+i0*t)+A*exp(-B*i0*t); */
  /*  Vb = Eb - i0*Rin; */
  /*  figure;plot(t,Vb,'.');grid; */
  /*   */
  /*  i0 = [6.5;13;32.5]; */
  /*  Eb = E0 - k*Q./(Q - i0*t).*(i0+i0*t)+A*exp(-B*i0*t); */
  /*  Vb = Eb - i0*Rin; */
  /*  figure;plot(t,Vb,'.');grid; */
}

/*
 * File trailer for dischargeCell_array.c
 *
 * [EOF]
 */
