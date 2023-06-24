function msg = set_restart_timeout(a, timeout)
% set Timeout for restart
%
% a - serial connection object to esp
% timeout - in seconds
%

timeout = typecast(uint32(timeout), 'uint8');
bytes = encode_msg('TO', timeout);
msg = esp_transmit(a, bytes);
if msg.crc.pass == false
	error(['Header crc failed: ', msg.crc.desc]);
end
% if isfield(msg, 'payload')
% 	disp(reshape([msg.payload.data], [], msg.batch_nelements).')
% end
