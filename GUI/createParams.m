function [prm] = createParams(sequence,ser)

%% run prms
%Run parameters
prm.run.seq = sequence;
% '1 - Discharge kp184'    
% '2 - B2B -> sep -> op -> B2B -> sep -> op -> B2B -> sep'
% '3 - Charge only'
% '4 - Charge + Discharge separately'
% '5 - chr -> dis -> chr'
% '6 - Calibration V,I'
% '7 - Discharge passive load'
prm.run.PlotSocFlag  = false;
prm.run.PlotItFlag   = false;
prm.run.PlotTempFlag = false;
prm.run.PlotVFlag    = true;
prm.run.PlotIFlag    = false;
prm.run.PlotIacsFlag = true;
prm.run.MaxTime = 15e3;
prm.run.dt = 1;
prm.run.T2Show = 100;%200
prm.run.Nt     = 2e4;

%% mode
% 1 - load (dis only), 
% 2 - B2B (dis + chr), 
% 3 - Power (chr) (ka6005P or ACDC via juntek), 
% 4 - load (dis) + ACDC (chr) separately  

% 1 Dis kp184
prm.seq(1).mod = 1;% Load
prm.seq(1).chr = 0;% chr - 1 , dis - 0
prm.seq(1).vth = 1;
prm.seq(1).ins = 1;%1-kunkin (load) 2-ka600p (chr) 3-juntek+ACDC (chr) 4-juntek B2B
prm.seq(1).Nst = 1;
prm.seq(1).swm = -1;

prm.seq(1).tst.ins = 1;
prm.seq(1).tst.v = true;
prm.seq(1).tst.i = true;
prm.seq(1).tst.swm = -1;% -1 no switch, 0 - Chr ACDC B1 + Load B2, 1 - Load B1 + Chr ACDC B2, 2 - B1 (chr) from B2 (dis), 3 - B2 (chr) from B1 (dis)

% 2 B2B -> sep -> op -> B2B -> sep -> op -> B2B -> sep
prm.seq(2).mod = [2 4 2 4 2 4];
prm.seq(2).chr = [1 1 0 0 1 1
                  0 0 1 1 0 0];% chr - 1 , dis - 0
prm.seq(2).vth = [1 1 1 1 2 2];
prm.seq(2).ins = [4 1 4 1 4 1
                  0 3 0 3 0 3];%1-kunkin (load) 2-ka600p (chr) 3-juntek+ACDC (chr) 4-juntek B2B
prm.seq(2).Nst = 6;
prm.seq(2).swm = [2 0 3 1 2 0];% -1 no switch, 0 - Chr ACDC B1 + Load B2, 1 - Load B1 + Chr ACDC B2, 2 - B1 (chr) from B2 (dis), 3 - B2 (chr) from B1 (dis)

prm.seq(2).tst.ins = 1;
prm.seq(2).tst.v = true;
prm.seq(2).tst.i = true;
prm.seq(2).tst.swm = -1;% -1 no switch, 0 - Chr ACDC B1 + Load B2, 1 - Load B1 + Chr ACDC B2, 2 - B1 (chr) from B2 (dis), 3 - B2 (chr) from B1 (dis)

% 3 Chr ka6005p
prm.seq(3).mod = [3];
prm.seq(3).chr = [1];% chr - 1 , dis - 0
prm.seq(3).vth = [1];
prm.seq(3).ins = [2];%1-kunkin (load) 2-ka600p (chr) 3-juntek+ACDC (chr) 4-juntek B2B
prm.seq(3).Nst = 1;
prm.seq(4).swm = -1;

prm.seq(3).tst.ins = 0;
prm.seq(3).tst.v = false;
prm.seq(3).tst.i = false;
prm.seq(3).tst.swm = -1;% -1 no switch, 0 - Chr ACDC B1 + Load B2, 1 - Load B1 + Chr ACDC B2, 2 - B1 (chr) from B2 (dis), 3 - B2 (chr) from B1 (dis)

% 4 Chr+Dis NA?
prm.seq(4).mod = [4];
prm.seq(4).chr = [1];
prm.seq(4).vth = [1];
prm.seq(4).ins = [1
                  2];%1-kunkin (load) 2-ka600p (chr) 3-juntek+ACDC (chr) 4-juntek B2B
prm.seq(4).Nst = 1;
prm.seq(4).swm = -1;

prm.seq(4).tst.ins = 0;
prm.seq(4).tst.v = false;
prm.seq(4).tst.i = false;
prm.seq(4).tst.swm = -1;% -1 no switch, 0 - Chr ACDC B1 + Load B2, 1 - Load B1 + Chr ACDC B2, 2 - B1 (chr) from B2 (dis), 3 - B2 (chr) from B1 (dis)


% 5 Chr -> Dis -> Chr
prm.seq(5).mod = [3 1 3];
prm.seq(5).chr = [1 0 1];
prm.seq(5).vth = [1 1 2];
prm.seq(5).ins = [2 1 2];%1-kunkin (load) 2-ka600p (chr) 3-juntek
prm.seq(5).Nst = 3;
prm.seq(5).swm = -1;

prm.seq(5).tst.ins = 1;
prm.seq(5).tst.v = true;
prm.seq(5).tst.i = true;
prm.seq(5).tst.swm = -1;% -1 no switch, 0 - Chr ACDC B1 + Load B2, 1 - Load B1 + Chr ACDC B2, 2 - B1 (chr) from B2 (dis), 3 - B2 (chr) from B1 (dis)

% 6 calibration
prm.seq(6).mod = [];
prm.seq(6).chr = [];
prm.seq(6).vth = [];
prm.seq(6).ins = [];%1-kunkin (load) 2-ka600p (chr) 3-juntek
prm.seq(6).Nst = [];
prm.seq(6).swm = -1;

prm.seq(6).tst.ins = 1;
prm.seq(6).tst.v = true;
prm.seq(6).tst.i = true;
prm.seq(6).tst.swm = -1;% -1 no switch, 0 - Chr ACDC B1 + Load B2, 1 - Load B1 + Chr ACDC B2, 2 - B1 (chr) from B2 (dis), 3 - B2 (chr) from B1 (dis)

% 7 Dis passive load
prm.seq(7).mod = 1;% Load
prm.seq(7).chr = 0;
prm.seq(7).vth = 1;
prm.seq(7).ins = 0;%1-kunkin (load) 2-ka600p (chr) 3-juntek
prm.seq(7).Nst = 1;
prm.seq(7).swm = -1;

prm.seq(7).tst.ins = -1;
prm.seq(7).tst.v = false;
prm.seq(7).tst.i = false;
prm.seq(7).tst.swm = -1;% -1 no switch, 0 - Chr ACDC B1 + Load B2, 1 - Load B1 + Chr ACDC B2, 2 - B1 (chr) from B2 (dis), 3 - B2 (chr) from B1 (dis)

% 8-juntek+ACDC (chr)
prm.seq(8).mod = [3];
prm.seq(8).chr = [1];
prm.seq(8).vth = [1];
prm.seq(8).ins = [3];%1-kunkin (load) 2-ka600p (chr) 3-juntek
prm.seq(8).Nst = 1;
prm.seq(8).swm = -1;

prm.seq(8).tst.ins = -1;
prm.seq(8).tst.v = false;
prm.seq(8).tst.i = false;
prm.seq(8).tst.swm = -1;% -1 no switch, 0 - Chr ACDC B1 + Load B2, 1 - Load B1 + Chr ACDC B2, 2 - B1 (chr) from B2 (dis), 3 - B2 (chr) from B1 (dis)

%% instruments configuration
prm.ins.ProjectFlag = 3;%0 - ina219 1 - EVBOTS_v1 2 - esp32  3 - serial esp32 4 - wifi esp32

%find ins to init
uIns = unique([prm.seq(prm.run.seq).ins(:) ; prm.seq(prm.run.seq).tst.ins]);
uIns = uIns(uIns>0);
prm.ins.juntek  = false;%true;%true;%false;%true;
prm.ins.ka6005p = false;
prm.ins.kp184   = false;
prm.ins.swm     = false;
%for k_ins = 1:length(uIns)
%    switch uIns(k_ins)
%        case 1 % kp184
%                prm.ins.kp184   = true;
%        case 2 % ka6005p
%                prm.ins.ka6005p = true;
%        case 3 % juntek + ACDC
%                prm.ins.juntek  = true;
%        case 4 % juntek B2B
%                prm.ins.juntek  = true;
%    end
%end
% init mode switch
uSwm = unique([prm.seq(prm.run.seq).swm(:) ; prm.seq(prm.run.seq).tst.swm]);
uSwm = uIns(uSwm>-1);
prm.ins.swm = false;
if length(uSwm)>0
    prm.ins.swm     = true;
end


% prm.ins.juntek_Flag = false;%true;%true;%false;%true;
% prm.ins.ka6005p = true;
% prm.ins.kp184   = true;
% prm.ins.swm     = true;
% prm.ins.juntekOnlyChr_Flag = false;
% prm.ins.OnlyDis_Flag = false;
% prm.ins.CalibrateBaordFlag = false;
% prm.ins.PwSwFalg = false;

%% str
prm.str.mod = { '1 - load (dis only)'
                '2 - B2B (dis + chr)' 
                '3 - Power (chr) (ka6005P or ACDC via juntek)' 
                '4 - load (dis) + ACDC (chr) separately'                  
};
prm.str.seq = { '1 - Dis kp184'    
                '2 - B2B -> sep -> op -> B2B -> sep -> op -> B2B -> sep'
                '3 - Chr ka6005p'
                '4 - Charge + Discharge seoarately'
                '5 - chr -> dis -> chr'
                '6 - Calibration V,I'
                '7 - Dis passive load'
                '8 - juntek+ACDC (chr)'
};
prm.str.ins ={'1 - kunkin kp184 (load)' 
              '2 - ka600p (chr)' 
              '3 - juntek+ACDC (chr)'
              '4 - juntek B2B'};
prm.str.sw  = { '-1 - no switch'
                ' 0 - Chr ACDC B1 + Load B2' 
                ' 1 - Load B1 + Chr ACDC B2' 
                ' 2 - B1 (chr) from B2 (dis)' 
                ' 3 - B2 (chr) from B1 (dis)'};
%% com ip serial wifi
% prm.ser.com.COM_esp32 = {'COM6'};%{'COM6','COM9'};%{Chr,Dis}%
% prm.ser.com.COM_kp184   = 'COM14';
% prm.ser.com.COM_ka6005P = 'COM7';
% prm.ser.com.COM_juntek  = 'COM4';
% prm.ser.com.COM_PwrSw   = 'COM3';
% prm.ser.wifi.ip = 0;%TODO
prm.ser = ser;
%% board

prm.brd.N_bat1 = 8;
prm.brd.N_bat2 = 2;
%prm.brd.N_bat = prm.brd.N_bat2 * prm.brd.N_bat1;
prm.brd.Nina219 = length(prm.ser.com.COM_esp32);
%prm.brd.Nbat = prm.brd.N_bat(1);
% prm.brd.DummyInit = [92 0 92 0 92 0];%[] if na
% SPI
mapFile              = '.\esp32\PortsMapping.mat';%'C:\Projects\Matlab\Arduino\draft\221222\flowTesting\esp32\PortsMapping.mat';
SwitchPortDataFile   = '.\esp32\SwitchDataESP3matNewPac_bypass_230710.mat';%'.\SwitchDataESP3matNewPac_230313.mat';%'C:\Projects\Matlab\Arduino\draft\221222\flowTesting\SwitchDataESP3matNewPac_230313.mat';
CreatPortFilFlag = false;
addrPort = [0x44;0x4C;0x54;0x5C];
bin2port = [[4:11]+1;[12:19]+1;[20:27]+1;[zeros(1,4),[28:31]+1]];
prm.brd.spi.rst = 10000;%SPI reset in seconds after not recieving command. 0 - default 5 minutes
prm.brd.spi.disconnect = uint8(254);
prm.brd.spi.bypass     = uint8(255);
%if CreatPortFilFlag           
%            % [SwitchCell_,SwitchCell,SwitchCellApp,N_row,N_col,ABCD1,ABCD2,PortMatAll,PortSpi,...
%            %     PortSpiRow,SwitchMat,ABCDport,ABCDport2,SwitchCell0,SwitchCellApp2,...
%            %     N_cell_esp  ,N_row_esp  ,N_col_esp  ,PortSpiRow_esp  ,SwitchMat_esp,...
%            %     N_cell_esp2 ,N_row_esp2 ,N_col_esp2 ,PortSpiRow_esp2 ,SwitchMat_esp2,...
%            %     Pac2Vid,Pac2Vid2] = CalcSwitchCell_ESP32_v1_230313_Total_mat_8_2_newPac...
%            %     (prm.brd.N_bat1,prm.brd.N_bat2,mapFile,addrPort,bin2port);
%             [SwitchCell_,SwitchCell,SwitchCellApp,N_row,N_col,ABCD1,ABCD2,PortMatAll,PortSpi,...
%                PortSpiRow,SwitchMat,ABCDport,ABCDport2,SwitchCell0,SwitchCellApp2,...
%                N_cell_esp  ,N_row_esp  ,N_col_esp  ,PortSpiRow_esp  ,SwitchMat_esp,...
%                N_cell_esp2 ,N_row_esp2 ,N_col_esp2 ,PortSpiRow_esp2 ,SwitchMat_esp2,...
%                Pac2Vid,Pac2Vid2] = CalcSwitchCell_ESP32_v1_230707_Total_mat_8_2_newPac_disBypass...
%                (prm.brd.N_bat1,prm.brd.N_bat2,mapFile,addrPort,bin2port,prm.brd.spi);
%        else
%            load(SwitchPortDataFile);
%end
% SwitchCell_
% SwitchCell,
% SwitchCellApp,
% N_row,
% N_col,
% ABCD1,
% ABCD2,
% PortMatAll,
% PortSpi,...
% PortSpiRow,
% SwitchMat
% ABCDport
% ABCDport2
% SwitchCell0
% SwitchCellApp2
% N_cell_esp
% N_row_esp
% N_col_esp
prm.brd.spi.PortSpiRow_esp = PortSpiRow_esp;
prm.brd.spi.SwitchMat_esp  = SwitchMat_esp;
% N_cell_esp2
% N_row_esp2
% N_col_esp2
prm.brd.spi.PortSpiRow_esp2 = PortSpiRow_esp2;
prm.brd.spi.SwitchMat_esp2  = SwitchMat_esp2;
prm.brd.spi.Pac2Vid         = Pac2Vid;
prm.brd.spi.Pac2Vid2        = Pac2Vid2;

%PAC
prm.brd.pac.i2cPacAdd = [[1:2:16]',[17:24]'];
prm.brd.pac.VIpacId = repmat([2 1 4 3 6 5 8 7 10 9 12 11 14 13 16 15],2,1);%[Nina219 X N_bat]
prm.brd.pac.readIpacFlag = false;
%prm.brd.pac.Rval = 1/11*ones(prm.brd.Nina219,prm.brd.N_bat,2);%kEst as design
prm.brd.pac.Rshunt = [0.0001,0.0001];%NA
RvalFile      = {'.\Boards_Params\B1_COM8\Rval_B1_COM8_230503.mat';'.\Boards_Params\B2_COM13\Rval_B2_COM13_230611.mat'   };%{'.\Boards_Params\B1_COM8\Rval_B1_COM8_230503.mat';'.\Boards_Params\B2_COM13\Rval_B2_COM13_230611.mat'   };%{'.\Boards_Params\B1_COM8\Rval_B1_COM8_230503.mat' ;'.\Boards_Params\B2_COM13\Rval_B2_COM13_230502.mat' };%{'.\Boards_Params\B4_COM14\Rval_B4_COM14_230502.mat' };%{'.\Boards_Params\B1_COM8\Rval_B1_COM8_230503.mat'};%{'.\Boards_Params\B3_COM9\Rval_B3_COM9_230501.mat'};%{'.\Boards_Params\B4_COM14\Rval_B4_COM14_230502.mat' ,'.\Boards_Params\B3_COM9\Rval_B3_COM9_230501.mat'};%{'.\Boards_Params\B2_COM13\Rval_B2_COM13_230423.mat' ,'.\Boards_Params\B4_COM14\Rval_B4_COM14_240423.mat' };%
pIacs758Files = {'.\Boards_Params\B1_COM8\pIacs_B1_COM8_230503.mat';'.\Boards_Params\B2_COM13\pIacs_B2_COM13_230611.mat'};%{'.\Boards_Params\B1_COM8\pIacs_B1_COM8_230503.mat';'.\Boards_Params\B2_COM13\pIacs_B2_COM13_230502.mat'};%{'.\Boards_Params\B4_COM14\pIacs_B4_COM14_230502.mat'};%{'.\Boards_Params\B1_COM8\pIacs_B1_COM8_230503.mat'};%{'.\Boards_Params\B3_COM9\pIacs_B3_COM9_230501.mat'};%{'.\Boards_Params\B4_COM14\pIacs_B4_COM14_230502.mat','.\Boards_Params\B3_COM9\pIacs_B3_COM9_230501.mat','.\Boards_Params\B4_COM14\pIacs_B4_COM14_250423.mat'};%{'.\Boards_Params\B2_COM13\pIacs_B2_COM13_230423.mat','.\Boards_Params\B4_COM14\pIacs_B4_COM14_240423.mat'};%
pIacs758 = zeros(prm.brd.Nina219,1,2,1);
%Rval     = zeros(prm.brd.Nina219,prm.brd.N_bat,2);
%for k_ina219 = 1:prm.brd.Nina219
%    help0 = load(RvalFile{k_ina219});
%    Rval(k_ina219,:,:)  = help0.Rval;
%    help0 = load(pIacs758Files{k_ina219});
%    pIacs758(k_ina219,1,:,1) = help0.pIacs758;
%end
%prm.brd.pac.Rval = Rval;
prm.brd.pac.pIacs758 = pIacs758;
prm.brd.pac.Iacs758Flag = 2;% 0-no acs758 , 1-ESP32 A2D , 2- PAC A2D
prm.brd.pac.Iacs758Id = 8;
prm.brd.pac.readIpacFlag = false;

%% config
prm.cnfg.ToggleFlag     = true;%false;%true;
prm.cnfg.Ttoggle        = 60;
prm.cnfg.NtoggleDrop    = 2;
prm.cnfg.minLenIna219   = 1;
prm.cnfg.BattConfigDis1 = [1:prm.brd.N_bat-prm.cnfg.NtoggleDrop]';%[1:N_bat]';%[1 ; 2 ; 3 ; 4];
prm.cnfg.BattConfigChr1 = [1:prm.brd.N_bat-prm.cnfg.NtoggleDrop]';%[1:N_bat]';%[1 ; 2 ; 3 ; 4];%[1 , 2 , 3 , 4];
prm.cnfg.BattConfigChr2 = [1:prm.brd.N_bat-prm.cnfg.NtoggleDrop]';%[1:N_bat]';%[1 ; 2 ; 3 ; 4];
prm.cnfg.BattConfigChr3 = [1:prm.brd.N_bat-prm.cnfg.NtoggleDrop]';%[1:N_bat]';%[1 ; 2 ; 3 ; 4];
prm.cnfg.BattConfigStandby = [prm.brd.spi.disconnect];%TODO
prm.cnfg.BattConfigBypass  = [prm.brd.spi.bypass];%TODO


%% bat chr dis prms
prm.bat.Vd = 1;% [V] power diode voltage;
prm.bat.ImaxDis = 8;%15;%35;%maximum current from discharge bat
prm.bat.Icharge = 5;%15;%;25;%5;%[A]
prm.bat.IchargePhase2 = 2.5;%;15;%2.5;%2.5;%2.5;%[A]
% prm.bat.IchargePhase2t = prm.bat.IchargePhase2;
prm.bat.IchargePhase3 = 2;%0.5;%0.3;%0.2;%[A]
prm.bat.minIphase2 = 2;%4;%7;%0.4;%6.5*0.03;
prm.bat.dIphase2   = 0.5;%3;%0.2;
prm.bat.CutOffDisV = 2.8;%2.7;%3.0;%2.8;%[V]
prm.bat.CutOffChrV = 4.1;%3.6;%4.1;%25;%4.1;%[V]
%t = [0:50e3];
%i = 6*abs(sin(2*pi/500*t));
%if true
%    bl = fir1(34,0.02,'low',chebwin(35,30));
%    %figure;freqz(bhi,1)
%    i_in = filter(bl,1,i);%i_in_read(1:1370,2));
%else
%    i_in = i;
%end
prm.bat.t = t;
prm.bat.i_in = i_in;

%% kalman
prm.klm.kalmanFlag = true;
BatParamsFile{1} = '.\BatRec\BatParams\rec03-Apr-2023 225800.mat';%'C:\Projects\Matlab\Arduino\draft\221222\flowTesting\BatRec\BatParams\rec03-Apr-2023 225800.mat';
BatParamsFile{2} = '.\BatRec\BatParams\rec03-Apr-2023 225800.mat';%'C:\Projects\Matlab\Arduino\draft\221222\flowTesting\BatRec\BatParams\rec03-Apr-2023 225800.mat';
for k_file = 1:length(BatParamsFile)%=Nina219
   [BatParamsCell{k_file}] = load(BatParamsFile{k_file});
   BatParamsCell{k_file}.BatStateOrg = BatParamsCell{k_file}.BatState;
end
prm.klm.Ta = BatParamsCell{k_file}.BatStateOrg(1,1);%Ta TODO
prm.klm.BatParamsCell = BatParamsCell;
%% instruments params
%juntek
prm.ins.jun.ImaxAcDC = 15;
prm.ins.jun.minVjuntekInput = 12;
prm.ins.jun.juntekEfficencyFactor = 0.85;

%% files
% data save path
prm.files.saveDir    = '.\BatRec\';
prm.files.sufixDir   = '\bat1\';
prm.files.prefixFile = '\rec';
% pIacs & Rval calibration save destination
prm.files.savePath_pIacs = '.\Boards_Params\B2_COM13\pIacs_B2_COM13_230611.mat';%'.\Boards_Params\B1_COM8\pIacs_B1_COM8_230503.mat';
prm.files.savePath_Rval  = '.\Boards_Params\B2_COM13\Rval_B2_COM13_230611.mat';%'.\Boards_Params\B1_COM8\Rval_B1_COM8_230503.mat';
