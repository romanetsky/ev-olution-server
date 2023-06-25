function msg = I2C_read(a, data)
% I2C read
%
% a - serial/wifi connection object to esp
% data format example:
%	data = [0,0,0,0,0;...
%   	    1,0,0,0,0;...
%       	2,0,0,0,0];
%	first column is the channel number,
%	then vector of zero bytes to fill in

data = uint8(data);
bytes = encode_msg('I2Cread', data);
msg = esp_transmit(a, bytes);
if msg.crc.pass == false
	error(['Header crc failed: ', msg.crc.desc]);
end
% if isfield(msg, 'payload')
% 	disp(reshape([msg.payload.data], [], msg.batch_nelements).')
% end
