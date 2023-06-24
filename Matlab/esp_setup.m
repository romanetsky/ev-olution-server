function [a, msg] = esp_setup(serial_port, restart_timeout_sec, baud_rate, serial_time_out)

% serial_port - serial port ('COM?')
% restart_timeout_sec - lack communication timeout for self restart (default - read from esp32)
% baud_rate - baud rate (default 115200)
% serial_time_out - timeout for serial communicaion write/read (default 5 millisecs)

if nargin < 1
    error('no COM port selected...');
end
if nargin < 4
	serial_time_out = 0.005;
end
if nargin < 3
	baud_rate = 115200;
end
if nargin < 2
	restart_timeout_sec = [];
end

a = serialport(serial_port, baud_rate, 'Timeout', serial_time_out);
pause(1);

if isempty(a)
	error(['failed to connect to esp: PORT = ', serial_port, ', BR = ', num2str(baud_rate), ', time_out = ', num2str(serial_time_out)]);
end

a.flush(); % clear 'input' & 'output' buffers

% read or update the restart timeout
msg = set_restart_timeout(a, restart_timeout_sec);
