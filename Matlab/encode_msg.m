function bytes_array = encode_msg(command, data)
% supported commands:
%   'SPIread'   - read from SPI device
%   'SPIwrite'  - write to SPI device
%   'PACread'   - read from PAC device (I2C)
%   'PACwrite'  - write to PAC device (I2C)
%   'TO'        - setup timeout before restart (in seconds - 4 bytes)
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
    case 'TO'
        opcode = uint8(hex2dec("0xDA"));
    otherwise
        disp("unknown command");
        return;
end

sufix = uint8(hex2dec(["0xBA", "0xDA"]));
% batch_hdr_size + sufix + data_length
data_size = 6 + 4 + length(sufix) + uint32(nx * ny);
% serial header
bytes_array = [prefix, opcode, typecast(data_size, 'uint8'), sufix];
% header crc
hdr_crc = crc16(bytes_array);
bytes_array = [bytes_array, typecast(hdr_crc, 'uint8')];
% batch header
bytes_array = [bytes_array, typecast(max(1,nx), 'uint8'), sufix];
% data
for n = 1:max(1,nx)
    bytes_array = [bytes_array, sufix, typecast(ny, 'uint8')];
    if ~isempty(data)
        bytes_array = [bytes_array, data(n,:)];
    end
end
