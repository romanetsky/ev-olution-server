function msg = decode_msg(bytes_array)
% bytes_array - input data array (uint8) from serial port
% msg - message struct

bytes_array = uint8(bytes_array);

if length(bytes_array) < 11
    error('not enough data to decode (minimum 11 bytes required)');
end
msg.prefix = dec2hex(bytes_array(1:2)).';
msg.prefix = msg.prefix(:).';
msg.opcode = dec2hex(bytes_array(3));
msg.opcode = ['0x', dec2hex(bytes_array(3)), ' :: ', decode_opcode(msg.opcode)];
msg.data_size = typecast(bytes_array(4:7), 'uint32');
msg.sufix = dec2hex(bytes_array(8:9)).';
msg.sufix = msg.sufix(:).';
msg.crc.value = typecast(bytes_array(10:11), 'uint16');
crc = crc16(bytes_array(1:9));
if crc == msg.crc.value
    msg.crc.pass = true;
    msg.crc.desc = ['0x', dec2hex(crc), ' :: crc passed'];
else
    msg.crc.pass = false;
    msg.crc.desc = ['crc failed: expected 0x', dec2hex(crc), ', received 0x', dec2hex(msg.crc)];
    return;
end
if length(bytes_array) < 11 + msg.data_size
    return;
end
msg.batch_nelements = typecast(bytes_array(12:15), 'uint32');
msg.batch_sufix = dec2hex(bytes_array(16:17)).';
msg.batch_sufix = msg.batch_sufix(:).';
data = bytes_array(18:end);
if msg.data_size - 6 ~= length(data)
    disp(['data size expected ', num2str(msg.data_size), ' ~= recieved ', num2str(length(data))]);
    return;
else
    n = 1;
    while ~isempty(data)
        msg.payload(n).sufix = dec2hex(data(1:2)).';
        msg.payload(n).sufix = msg.payload(n).sufix(:).';
        msg.payload(n).nelements = typecast(data(3:6), 'uint32');
        msg.payload(n).data = data(6 + (1:msg.payload(n).nelements));
        data(1:(6 + msg.payload(n).nelements)) = [];
        n = n + 1;
    end
end

function opcode_str = decode_opcode(opcode)
switch opcode
    case '6A'
        opcode_str = 'SPI read';
    case '65'
        opcode_str = 'SPI write';
    case '9A'
        opcode_str = 'I2C read';
    case '95'
        opcode_str = 'I2C write';
    case 'DA'
        opcode_str = 'Timeout';
    otherwise
        opcode_str = 'unknown opcode';
        return;
end
