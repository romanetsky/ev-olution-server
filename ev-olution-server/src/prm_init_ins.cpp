#include "Chr_Dis_kp184_ka6005P_juntek_esp32ser_20240229_types.h"

void prm_init_ins(struct0_T& r)
{
r.ins.juntek = 1;
r.ins.ka6005p = 0;
r.ins.kp184 = 1;
r.ins.swm = 0;
r.ins.sw16to1 = 1;
r.ins.swOut = 0;
r.ins.prm.jun.ImaxAcDC = 6;
r.ins.prm.jun.minVjuntekInput = 28;
r.ins.prm.jun.juntekEfficencyFactor = 0.85;
r.ins.prm.jun.baudrate = 9600;
r.ins.prm.sw16to1.Ntry = 3;
r.ins.prm.swOut.Rshunt = 0.0001;
r.ins.prm.swOut.Vratio = 11.0115;
r.ins.prm.swOut.IfromVdivR_flag = 1;
r.ins.ProjectFlag = 3;
}