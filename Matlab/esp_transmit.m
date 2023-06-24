function msg = esp_transmit(a, bytes)
% write/read to/from esp32

msg = [];
if ~isempty(bytes)
	write(a, bytes, 'uint8');
	% wait for data
	while a.NumBytesAvailable < 11
		pause(a.Timeout);
	end
	hdr_raw = read(a, 11, 'uint8');
	msg = decode_msg(hdr_raw);
	wait = 0;
	while a.NumBytesAvailable < msg.data_size
		pause(a.Timeout);
		wait = wait + a.Timeout;
		if wait > 1
			error('time out occured');
		end
	end
	data_raw = read(a, a.NumBytesAvailable, 'uint8');
	msg = decode_msg([hdr_raw, data_raw]);
end
