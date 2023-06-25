function [a, msg] = esp_setup(varargin)

% for wifi setup:
%   hostname/ip - hostname or ip address (default '192.168.4.1')
%   port - tcp/ip port (default is 4556)
%   restart_timeout_sec - lack of communication timeout for self restart (default - read from esp32)
%   timeout - timeout for wifi communicaion write/read (default 5 millisecs)
% for serial setup:
%   serial_port - serial port ('COM?')
%   restart_timeout_sec - lack of communication timeout for self restart (default - read from esp32)
%   baud_rate - baud rate (default 115200)
%   serial_time_out - timeout for serial communicaion write/read (default 5 millisecs)
%

% check what setup to run: serial or wifi
wifi_setup = false;
if nargin < 1
    wifi_setup = true;
end
if wifi_setup == false && strncmp("COM", varargin{1}, 3) == false
    wifi_setup = true;
end

% wifi setup
if wifi_setup == true
    if nargin < 1
        hostname = '192.168.4.1';%'my_evo_host';
    end
    if nargin < 2
        port = 4556;
    end
    if nargin < 3
	    restart_timeout_sec = [];
    end
    if nargin < 4
	    timeout = 0.005;
    end
else
% serial setup
    serial_port = varargin{1};
    if nargin < 4
	    timeout = 0.005;
    end
    if nargin < 3
	    baud_rate = 115200;
    end
    if nargin < 2
	    restart_timeout_sec = [];
    end
end
try
    % wifi setup
    if wifi_setup == true
        disp("connecting to esp32 via wifi...");
        a = tcpclient(hostname, port, "Timeout", timeout);
    else
    % serial setup
        disp("connecting to esp32 via serial...");
        a = serialport(serial_port, baud_rate, 'Timeout', timeout);
    end
catch ME
    disp(ME.message);
    a = [];
    msg = [];
    return;
end
pause(1);

if isempty(a)
	error('failed to connect to esp...');
end

a.flush(); % clear 'input' & 'output' buffers

% read or update the restart timeout
msg = set_restart_timeout(a, restart_timeout_sec);
