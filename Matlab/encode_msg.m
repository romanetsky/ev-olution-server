function bytes_array = encode_msg(command, data)
% command: 'SPIread', 'SPIwrite', 'PACread', 'PACwrite'
% data - data (uint8 array)

[nx, ny] = size(data);
nx = uint32(nx);
ny = uint32(ny);

bytes_array = [];
prefix = uint8(hex2dec(["0xCA", "0xFE"]));
switch command
    case 'SPIread'
        opcode = uint8(hex2dec("0x6A"));
    case 'SPIwrite'
        opcode = uint8(hex2dec("0x65"));
    case 'I2Cread'
        opcode = uint8(hex2dec("0x9A"));
    case 'I2Cwrite'
        opcode = uint8(hex2dec("0x95"));
    otherwise
        disp("unknown command");
        return;
end

sufix = uint8(hex2dec(["0xBA", "0xDA"]));
data_size = 4 + length(sufix) + uint32(nx * ny);
% serial header
bytes_array = [prefix, opcode, typecast(data_size, 'uint8'), sufix];
% header crc
hdr_crc = crc16(bytes_array);
bytes_array = [bytes_array, typecast(hdr_crc, 'uint8')];
% batch header
bytes_array = [bytes_array, typecast(nx, 'uint8'), sufix];
% data
for n = 1:nx
    bytes_array = [bytes_array, sufix, typecast(ny, 'uint8'), data(n,:)];
end
