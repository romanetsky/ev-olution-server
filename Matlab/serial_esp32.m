% serial = arduino('COM3', 'ESP32-WROOM-DevKitV1', 'Libraries',{'I2C','SPI', 'Serial'});
% SpiChipSelectPin  = {'D5'};%SpiChipSelectPin
% SpiBitRate = 1000000;%1Mhz
% SPI = device(serial,'SPIChipSelectPin',SpiChipSelectPin,'BitRate',SpiBitRate);
%reset SPI

Commands.NO_ACTION         = 0x00; %write0x00
Commands.POWER             = 0x04;
Commands.PORT_CONFIG_4_7   = 0x09;
Commands.PORT_CONFIG_8_11  = 0x0A;
Commands.PORT_CONFIG_12_15 = 0x0B;
Commands.PORT_CONFIG_16_19 = 0x0C;
Commands.PORT_CONFIG_20_23 = 0x0D;
Commands.PORT_CONFIG_24_27 = 0x0E;
Commands.PORT_CONFIG_28_31 = 0x0F;
Commands.PORT_STATE_4_11   = 0x44;
Commands.PORT_STATE_12_19  = 0x4C;
Commands.PORT_STATE_20_27  = 0x54;
Commands.PORT_STATE_28_31  = 0x5C;

PinMode.OUT_LED  = 0b00;
PinMode.OUT_GPIO = 0b01;
PinMode.IN_GPIO  = 0b10;
PinMode.IN_GPIO_PULL_UP = 0b11;

State.OFF = uint8(0);
State.ON  = uint8(1);
helpVec2 = uint8([1 4 16 64]);

a = serialport('COM5', 115200, 'Timeout', 0.01);
b = read(a, a.NumBytesAvailable, 'uint8');

bytes_array = encode_msg(3, [Commands.PORT_CONFIG_4_7,uint8(sum([PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.IN_GPIO , PinMode.IN_GPIO ].*helpVec2)),...
            Commands.PORT_CONFIG_4_7,uint8(sum([PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO].*helpVec2)),...
            Commands.PORT_CONFIG_4_7,uint8(sum([PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO].*helpVec2))]);
write(a, bytes_array, 'uint8');
b = read(a, a.NumBytesAvailable, 'uint8');
msg = decode_msg(uint8(b));

bytes_array = encode_msg(2, [Commands.PORT_CONFIG_8_11,uint8(sum([PinMode.IN_GPIO, PinMode.IN_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO  ].*helpVec2)),...
            Commands.PORT_CONFIG_8_11,uint8(sum([PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO].*helpVec2)),...
            Commands.PORT_CONFIG_8_11,uint8(sum([PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO].*helpVec2))]);
write(serial, bytes_array, 'uint8');
b = read(serial, serial.NumBytesAvailable, 'uint8');
msg = decode_msg(uint8(b));

bytes_array = encode_msg(2, [Commands.PORT_CONFIG_8_11,uint8(sum([PinMode.IN_GPIO, PinMode.IN_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO  ].*helpVec2)),...
            Commands.PORT_CONFIG_8_11,uint8(sum([PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO].*helpVec2)),...
            Commands.PORT_CONFIG_8_11,uint8(sum([PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO].*helpVec2))]);
write(a, bytes_array, 'uint8');
b = read(a, a.NumBytesAvailable, 'uint8');
msg = decode_msg(uint8(b));

% write(SPI, [Commands.PORT_CONFIG_4_7,uint8(sum([PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.IN_GPIO , PinMode.IN_GPIO ].*helpVec2)),...
%             Commands.PORT_CONFIG_4_7,uint8(sum([PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO].*helpVec2)),...
%             Commands.PORT_CONFIG_4_7,uint8(sum([PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO].*helpVec2))]);
% 
% write(SPI, [Commands.PORT_CONFIG_8_11,uint8(sum([PinMode.IN_GPIO, PinMode.IN_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO  ].*helpVec2)),...
%             Commands.PORT_CONFIG_8_11,uint8(sum([PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO].*helpVec2)),...
%             Commands.PORT_CONFIG_8_11,uint8(sum([PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO].*helpVec2))]);

write(SPI, [Commands.PORT_CONFIG_12_15,uint8(sum([PinMode.IN_GPIO, PinMode.IN_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO  ].*helpVec2)),...
            Commands.PORT_CONFIG_12_15,uint8(sum([PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO].*helpVec2)),...
            Commands.PORT_CONFIG_12_15,uint8(sum([PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO].*helpVec2))]);

write(SPI, [Commands.PORT_CONFIG_16_19,uint8(sum([PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO].*helpVec2)),...
            Commands.PORT_CONFIG_16_19,uint8(sum([PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO].*helpVec2)),...
            Commands.PORT_CONFIG_16_19,uint8(sum([PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO].*helpVec2))]);

write(SPI, [Commands.PORT_CONFIG_20_23,uint8(sum([PinMode.IN_GPIO, PinMode.IN_GPIO, PinMode.IN_GPIO, PinMode.IN_GPIO    ].*helpVec2)),...
            Commands.PORT_CONFIG_20_23,uint8(sum([PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO].*helpVec2)),...
            Commands.PORT_CONFIG_20_23,uint8(sum([PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO].*helpVec2))]);

write(SPI, [Commands.PORT_CONFIG_24_27,uint8(sum([PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO].*helpVec2)),...
            Commands.PORT_CONFIG_24_27,uint8(sum([PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO].*helpVec2)),...
            Commands.PORT_CONFIG_24_27,uint8(sum([PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO].*helpVec2))]);

write(SPI, [Commands.PORT_CONFIG_28_31,uint8(sum([PinMode.IN_GPIO, PinMode.IN_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO  ].*helpVec2)),...
            Commands.PORT_CONFIG_28_31,uint8(sum([PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO].*helpVec2)),...
            Commands.PORT_CONFIG_28_31,uint8(sum([PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO].*helpVec2))]);

% shutdown control  normal operation
write(SPI, [Commands.POWER, State.ON,...
            Commands.POWER, State.ON,...
            Commands.POWER, State.ON]);
