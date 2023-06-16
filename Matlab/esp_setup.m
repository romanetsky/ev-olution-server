function a = esp_setup(serial_port, baud_rate, time_out)

if nargin < 3
	time_out = 0.005;
end
if nargin < 2
	baud_rate = 115200;
end
if nargin < 1
	serial_port = 'COM7';
end

a = serialport(serial_port, baud_rate, 'Timeout', time_out);
pause(1);

if isempty(a)
	error(['failed to connect to esp: PORT = ', serial_port, ', BR = ', num2str(baud_rate), ', time_out = ', num2str(time_out)]);
end

a.flush(); % clear 'input' & 'output' buffers
