function msg = SPI_read(a, data)
% SPI read
%
% a - serial/wifi connection object to esp
% data format example:
%	data = [68,0,68,0,68,0;...
%   	    72,0,72,0,72,0;...
%       	81,0,81,0,81,0;...
%        	94,0,94,0,94,0];
%

data = uint8(data);
bytes = encode_msg('SPIwriteread', data);
msg = esp_transmit(a, bytes);
if msg.crc.pass == false
	error(['Header crc failed: ', msg.crc.desc]);
end
% if isfield(msg, 'payload')
% 	disp(reshape([msg.payload.data], [], msg.batch_nelements).')
% end
