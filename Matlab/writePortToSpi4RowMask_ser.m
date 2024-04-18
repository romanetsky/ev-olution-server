function [] = writePortToSpi4RowMask_ser(PortSpiRow,SPI0,maskFalg)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% if coder.target('MATLAB')
dummy = uint8([15 1 15 1 15 1]);
if nargin==2
    maskFalg = true;
end
[N_row,N2_chip] = size(PortSpiRow);
disConAllSwitch = PortSpiRow;
disConAllSwitch(:,2:2:end) = 0;
data          = disConAllSwitch;
if maskFalg
    % read current
    N_chip = N2_chip/2;     %  chip1     |   chip2     |   chip3
    com_N_maskClose = uint8([0xFF , 0x7E , 0xFF , 0xFF ,0xFF , 0xFF;...%comN2 Port4 chip1,comN1 Port11 chip1
        0xFF , 0xBF , 0xFF , 0xFF ,0xFF , 0xBF;...%comP1 Port18 chip1 ,comP2 Port18 chip3
        0xFF , 0xFF , 0xFF , 0xFF ,0xFF , 0xFF;...
        0xFF , 0xFF , 0xFF , 0xFF ,0xFF , 0xFF]);%
    oldPortSpiRow = uint8(zeros(N_row,N2_chip));
    out123 = uint8(zeros(N_row,N2_chip));
    % for k_row = 1:N_row
    %     out123(k_row,:) = readSPI(SPI,PortSpiRow(k_row,1),N_chip);
    %     oldPortSpiRow(k_row,1:2:end) = PortSpiRow(k_row,1:2:end);
    %     oldPortSpiRow(k_row,2:2:end) = out123(k_row,2:2:end);
    % end
    if coder.target('MATLAB')
        Rmsg          = SPI_read(SPI0, [data;dummy]);
        RmsgData      = reshape([Rmsg.payload(2:5).data],6,4)';
        [SortAdd ,SortAddId] = sort(RmsgData(:,1));
        oldPortSpiRow = RmsgData(SortAddId,:);%RmsgData([[2:4] , 1] ,:);
    else
        oldPortSpiRow = PortSpiRow;
    end
    %mask current
    oldPortSpiRowMask = bitand(oldPortSpiRow,com_N_maskClose);
    %write current mask
    % for k_row = 1:N_row
    %     write(SPI,oldPortSpiRowMask(k_row,:));
    % end
    if coder.target('MATLAB')
        Wmsg = SPI_write(SPI0, oldPortSpiRowMask);

        %close all switch
        % for k_row = 1:N_row
        %     write(SPI0,disConAllSwitch(k_row,:));
        % end
        SPI_write(SPI0, disConAllSwitch);
    end
    pause(1e-3);
    %mask new
    PortSpiRowMask = bitand(PortSpiRow,com_N_maskClose);
    %write mask new
    % for k_row = 1:N_row
    %     write(SPI0,PortSpiRowMask(k_row,:));
    % end
    if coder.target('MATLAB')
        SPI_write(SPI0, PortSpiRowMask);
    end
end
%write new
% for k_row = 1:N_row
%     write(SPI0,PortSpiRow(k_row,:));
% end
% Rmsg = SPI_writeread(SPI0, PortSpiRow);
if coder.target('MATLAB')
    Rmsg = SPI_write(SPI0, PortSpiRow);

    % ReadPortSpiRow = uint8(zeros(N_row,N2_chip));
    % out123 = uint8(zeros(N_row,N2_chip));
    % for k_row = 1:N_row
    %     out123(k_row,:) = readSPI(SPI0,PortSpiRow(k_row,1),N_chip);
    %     ReadPortSpiRow(k_row,1:2:end) = out123(k_row,1:2:end)-128;
    %     ReadPortSpiRow(k_row,2:2:end) = out123(k_row,2:2:end);
    % end
    Rmsg          = SPI_read(SPI0, [data;dummy]);
    RmsgData      = reshape([Rmsg.payload(2:5).data],6,4)';
    [~ ,SortAddId] = sort(RmsgData(:,1));
    ReadPortSpiRow = RmsgData(SortAddId,:);%RmsgData([[2:4] , 1] ,:)
else
    ReadPortSpiRow = PortSpiRow;
end
% ReadPortSpiRow = RmsgData([[2:4] , 1] ,:);
% ReadPortSpiRow(:,1:2:end) = ReadPortSpiRow(:,1:2:end);% - 128;
%mask current
MaskAlert = uint8([0xFF , 0xC3 , 0xFF , 0xFF ,0xFF , 0xFF;...%chip1 Port6 Alert1, chip1 port 7 Alert3, chip1 port 8 Alert8, chip1 port 9 Alert6
    0xFF , 0x7C , 0xFF , 0xFF ,0xFF , 0xFF;...%chip1 Port12 Alert7, chip1 Port13 Alert5, chip 1 port19 NC
    0xFF , 0xF0 , 0xFF , 0xFF ,0xFF , 0xFF;...%chip1 Port20-23 NC
    0xFF , 0xFC , 0xFF , 0xFF ,0xFF , 0xFF]); %chip1 Port28 Alert1, chip1 Port29 Alert2
ReadPortSpiRowMaskAlert = bitand(ReadPortSpiRow,MaskAlert);
% if sum(sum(sum(abs(double(PortSpiRow)-double(ReadPortSpiRow)))))>0
if sum(sum(sum(abs(double(PortSpiRow)-double(ReadPortSpiRowMaskAlert)))))>0
    %close all switch
    if coder.target('MATLAB')
        SPI_write(SPI0,disConAllSwitch);
    end
    disp('Error SPI write');
    disp('write:');
    if coder.target('MATLAB')
        disp(num2str(PortSpiRow));
    else
        % disp(sprintf('%03d',PortSpiRow'));
%         fprintf(' %03d %03d %03d %03d %03d %03d\n',PortSpiRow');
    end
    disp('read:');
    if coder.target('MATLAB')
        disp(num2str(ReadPortSpiRow));
    else
        %disp(sprintf('%03d',ReadPortSpiRow'));
%         fprintf(' %03d %03d %03d %03d %03d %03d\n',ReadPortSpiRow');
    end


else
    ReadPortSpiRowAlert = bitand(ReadPortSpiRow,bitcmp(MaskAlert));
    if false %sum(sum(abs(bitcmp(MaskAlert)-ReadPortSpiRowAlert)))>0
        disp('PAC Alert. diff:');
        disp(num2str(bitcmp(MaskAlert)-ReadPortSpiRowAlert));
        %         ReadPortSpiRowAlert = bitand(ReadPortSpiRow,bitcmp(MaskAlert));
    end
    %     disp(num2str(PortSpiRow));
end
% end





