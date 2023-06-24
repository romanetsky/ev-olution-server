function crc = crc16(data)

data = double(data);
crc = uint16(hex2dec('FFFF'));
for i = 1:length(data)
    crc = bitxor(crc, bitshift(data(i), 8));
    for bit = 1:8
        if bitand(crc, hex2dec('8000'))
            crc = bitxor(bitshift(crc, 1), hex2dec('1021'));
        else
            crc = bitshift(crc, 1);
        end
        crc = bitand(crc, hex2dec('FFFF'));
    end
end
crc = bitand(crc, hex2dec('FFFF'));
