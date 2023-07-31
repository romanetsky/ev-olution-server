serial_port = 'COM7';
baud_rate = 115200;
time_out = 0.005;
a = serialport(serial_port, baud_rate, 'Timeout', time_out);
pause(1);
a.flush(); % clear 'input' & 'output' buffers

%% SPI write
data = [68,0,68,0,68,0;...
        72,0,72,0,72,0;...
        81,0,81,0,81,0;...
        94,0,94,0,94,0];
for k = 0:199
    data(:,[2,4,6]) = k;
    data = uint8(data);
    bytes = encode_msg('SPIwrite', data);
    tic
    msg = esp_comm(a, bytes, time_out);
    toc
    if msg.crc.pass == false
        a.delete();
        error(msg.crc.desc);
    end
    if isfield(msg, 'payload')
        disp(reshape([msg.payload.data], [], msg.batch_nelements).')
    end
    bytes = encode_msg('SPIread', data);
    tic
    msg = esp_comm(a, bytes, time_out);
    toc
    if msg.crc.pass == false
        a.delete();
        error(msg.crc.desc);
    end
    if isfield(msg, 'payload')
        disp(reshape([msg.payload.data], [], msg.batch_nelements).')
    end
end
%% SPI read
data = [68,0,68,0,68,0;...
        72,0,72,0,72,0;...
        81,0,81,0,81,0;...
        94,0,94,0,94,0];
data = uint8(data);
bytes = encode_msg('SPIread', data);

%% I2C read
data = [0,0,0,0,0]; % first byte is the address, then vector of bytes to fill in
data = uint8(data);
bytes = encode_msg('I2Cread', data);

for rep = 1:200
    for k = 0:15
        data(:,[2,4,6]) = k;
        data = uint8(data);
        bytes = encode_msg('I2Cread', data);
        tic
        msg = esp_comm(a, bytes, time_out);
        toc
        if msg.crc.pass == false
            a.delete();
            error(msg.crc.desc);
        end
        if isfield(msg, 'payload')
            disp(reshape([msg.payload.data], [], msg.batch_nelements).')
        end
    end
end

%% close connection
a.delete();

function msg = esp_comm(a, bytes, time_out)
    %% communicate with esp32
    if ~isempty(bytes)
        write(a, bytes, 'uint8');
        % wait for data
        while a.NumBytesAvailable < 11
            pause(time_out);
        end
        hdr_raw = read(a, 11, 'uint8');
        msg = decode_msg(hdr_raw);
        wait = 0;
        while a.NumBytesAvailable < msg.data_size
            pause(time_out);
            wait = wait + time_out;
            if wait > 1
                a.delete();
                error('time out occured');
            end
        end
        data_raw = read(a, a.NumBytesAvailable, 'uint8');
        msg = decode_msg([hdr_raw, data_raw]);
    end
end
