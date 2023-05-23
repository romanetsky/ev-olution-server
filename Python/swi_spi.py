import time
import machine
from machine import Pin, SPI
# from mock_machine import Pin, SPI

PW_En = machine.Pin(4, machine.Pin.OUT)
PW_En.value(0) #I4, (PW_En) should be HIGH to work with Pereferials
time.sleep_ms(100)
PW_En.value(1)


spi = SPI(2, baudrate=1000000, polarity=0, phase=0, sck=Pin(18), mosi=Pin(23), miso=Pin(19))
spi.init(baudrate=1000000)  # set the baudrate
cs = Pin(5, mode=Pin.OUT, value=1)


# workaround of absent enum in micropython
def enum(**enums: int):
    return type('Enum', (), enums)


Commands = enum(
    NO_ACTION=0x00, #write0x00
    POWER=0x04,
    PORT_CONFIG_4_7=0x09,
    PORT_CONFIG_8_11=0x0A,
    PORT_CONFIG_12_15=0x0B,
    PORT_CONFIG_16_19=0x0C,
    PORT_CONFIG_20_23=0x0D,
    PORT_CONFIG_24_27=0x0E,
    PORT_CONFIG_28_31=0x0F,
    PORT_STATE_4_11=0x44,
    PORT_STATE_12_19=0x4C,
    PORT_STATE_20_27=0x54,
    PORT_STATE_28_31=0x58
)

PinMode = enum(
    OUT_LED=0b00,
    OUT_GPIO=0b01,
    IN_GPIO=0b10,
    IN_GPIO_PULL_UP=0b11,
)

State = enum(
    OFF=0,
    ON=1
)


def four_ports_configuration(pin_mode_1: PinMode, pin_mode_2: PinMode, pin_mode_3: PinMode, pin_mode_4: PinMode) -> int:
    """
    :rtype: one byte configuretion, where LSB is first pin mode and MSB is last pin mode
    """
    result = int(pin_mode_4)
    result = result << 2 | int(pin_mode_3)
    result = result << 2 | int(pin_mode_2)
    result = result << 2 | int(pin_mode_1)
    return result


def write_to_spi(spi: spi, address: Commands, data):
    buffer = bytearray(2)
    buffer[0] = address
    buffer[1] = data
    spi.write(buffer)
#-----------------------init procedure for SPI Expander-----------------------------------

def init():
    print("start init")
    # construct an SPI bus on the given pins
    # polarity is the idle state of SCK
    # phase=0 means sample on the first edge of SCK, phase=1 means the second
    
    # TODO все пины перевести в состояние logic Output (x01)
    #  (за исключением ALERT 1..8 и TestPoint 1..5, которые должны стать input (x10) )
    # исключения: p6, p7, p8, p9, p12, p13, p19, p20, p21, p22, p23, p28, p29

    cs.off()
    write_to_spi(spi, Commands.PORT_CONFIG_4_7,
                 four_ports_configuration(PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.IN_GPIO, PinMode.IN_GPIO))

    write_to_spi(spi, Commands.PORT_CONFIG_4_7,
                 four_ports_configuration(PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO))

    write_to_spi(spi, Commands.PORT_CONFIG_4_7,
                 four_ports_configuration(PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO))
    cs.on()

    cs.off()
    write_to_spi(spi, Commands.PORT_CONFIG_8_11,
                 four_ports_configuration(PinMode.IN_GPIO, PinMode.IN_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO))

    write_to_spi(spi, Commands.PORT_CONFIG_8_11,
                 four_ports_configuration(PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO))

    write_to_spi(spi, Commands.PORT_CONFIG_8_11,
                 four_ports_configuration(PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO))
    cs.on()

    cs.off()
    write_to_spi(spi, Commands.PORT_CONFIG_12_15,
                 four_ports_configuration(PinMode.IN_GPIO, PinMode.IN_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO))

    write_to_spi(spi, Commands.PORT_CONFIG_12_15,
                 four_ports_configuration(PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO))

    write_to_spi(spi, Commands.PORT_CONFIG_12_15,
                 four_ports_configuration(PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO))
    cs.on()

    cs.off()
    write_to_spi(spi, Commands.PORT_CONFIG_16_19,
                 four_ports_configuration(PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO))
    write_to_spi(spi, Commands.PORT_CONFIG_16_19,
                 four_ports_configuration(PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO))
    write_to_spi(spi, Commands.PORT_CONFIG_16_19,
                 four_ports_configuration(PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO))
    cs.on()

    cs.off()
    write_to_spi(spi, Commands.PORT_CONFIG_20_23,
                 four_ports_configuration(PinMode.IN_GPIO, PinMode.IN_GPIO, PinMode.IN_GPIO, PinMode.IN_GPIO))
    write_to_spi(spi, Commands.PORT_CONFIG_20_23,
                 four_ports_configuration(PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO))
    write_to_spi(spi, Commands.PORT_CONFIG_20_23,
                 four_ports_configuration(PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO))
    cs.on()

    cs.off()
    write_to_spi(spi, Commands.PORT_CONFIG_24_27,
                 four_ports_configuration(PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO))
    write_to_spi(spi, Commands.PORT_CONFIG_24_27,
                 four_ports_configuration(PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO))
    write_to_spi(spi, Commands.PORT_CONFIG_24_27,
                 four_ports_configuration(PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO))
    cs.on()

    cs.off()
    write_to_spi(spi, Commands.PORT_CONFIG_28_31,
                 four_ports_configuration(PinMode.IN_GPIO, PinMode.IN_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO))
    write_to_spi(spi, Commands.PORT_CONFIG_28_31,
                 four_ports_configuration(PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO))
    write_to_spi(spi, Commands.PORT_CONFIG_28_31,
                 four_ports_configuration(PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO, PinMode.OUT_GPIO))
    cs.on()

    # перевести shutdown control в normal operation
    cs.off()
    write_to_spi(spi, Commands.POWER, State.ON)
    write_to_spi(spi, Commands.POWER, State.ON)
    write_to_spi(spi, Commands.POWER, State.ON)
    cs.on()

    print("finished init")
    
#--------------------------end of SPI expander init procedure-----------------

#---------------------SPI exander ports mapping dictinary-------------------------
spi_registers = {'1A': ['P5', 3, '0x25'], '1B': ['P31', 3, '0x3F'], '1C': ['P4', 3, '0x24'], '1D': ['P16', 3, '0x30'], '2A': ['P6', 3, '0x26'], '2B': ['P30', 3, '0x3E'], '2C': ['P15', 3, '0x2F'], '2D': ['P11', 3, '0x2B'], '3A': ['P7', 3, '0x27'], '3B': ['P29', 3, '0x3D'], '3C': ['P14', 3, '0x2E'], '3D': ['P10', 3, '0x2A'], '4A': ['P27', 3, '0x3B'], '4B': ['P28', 3, '0x3C'], '4C': ['P13', 3, '0x2D'], '4D': ['P9', 3, '0x29'], '5A': ['P25', 3, '0x39'], '5B': ['P26', 3, '0x3A'], '5C': ['P12', 3, '0x2C'], '5D': ['P8', 3, '0x28'], '6A': ['P23', 3, '0x37'], '6B': ['P24', 3, '0x38'], '6C': ['P30', 1, '0x3E'], '6D': ['P5', 1, '0x25'], '7A': ['P21', 3, '0x35'], '7B': ['P22', 3, '0x36'], '7C': ['P24', 1, '0x38'], '7D': ['P25', 1, '0x39'], '8A': ['P19', 3, '0x33'], '8B': ['P20', 3, '0x34'], '8C': ['P26', 1, '0x3A'], '8D': ['P27', 1, '0x3B'], '9A': ['P24', 2, '0x38'], '9B': ['P25', 2, '0x39'], '9C': ['P15', 1, '0x2F'], '9D': ['P10', 1, '0x2A'], '10A': ['P22', 2, '0x36'], '10B': ['P23', 2, '0x37'], '10C': ['P14', 1, '0x2E'], '10D': ['P29', 2, '0x3D'], '11A': ['P20', 2, '0x34'], '11B': ['P21', 2, '0x35'], '11C': ['P7', 2, '0x27'], '11D': ['P16', 1, '0x30'], '12A': ['P18', 2, '0x32'], '12B': ['P19', 2, '0x33'], '12C': ['P17', 1, '0x31'], '12D': ['P28', 2, '0x3C'], '13A': ['P16', 2, '0x30'], '13B': ['P17', 2, '0x31'], '13C': ['P27', 2, '0x3B'], '13D': ['P26', 2, '0x3A'], '14A': ['P11', 2, '0x2B'], '14B': ['P15', 2, '0x2F'], '14C': ['P30', 2, '0x3E'], '14D': ['P6',
2, '0x26'], '15A': ['P10', 2, '0x2A'], '15B': ['P14', 2, '0x2E'], '15C': ['P5', 2, '0x25'], '15D': ['P31', 2, '0x3F'], '16A': ['P9', 2, '0x29'], '16B': ['P13', 2, '0x2D'], '16C': ['P4', 2, '0x24'], '16D': ['P8', 2, '0x28'],
'TP5': ['P19', 1, '0x33'], 'Com_P2': ['P18', 3, '0x32'], 'Com_N2': ['P4', 1, '0x24'], 'Com_P1': ['P18', 1, '0x32'], 'Com_N1': ['P11', 1, '0x2B'], 'PhaseSW_1': ['P17', 3, '0x31'], 'PhaseSW_2': ['P31', 1, '0x3F'], 'Com5': ['P12', 2, '0x2C']}
    


def set_swi(swiname: str, act: bool):
    
    debug = 0


    if swiname == 'close_all':
        if debug == 1:
            print(swiname)
        cs(0)
        write_to_spi(spi, Commands.PORT_STATE_4_11, 0b00000000)#chip1 (right)
        write_to_spi(spi, Commands.PORT_STATE_4_11, 0b00000000)
        write_to_spi(spi, Commands.PORT_STATE_4_11, 0b00000000)#chip3 (left)
        cs(1)
        time.sleep_ms(1)
        cs(0)
        write_to_spi(spi, Commands.PORT_STATE_12_19, 0b00000000)#chip1 (right)
        write_to_spi(spi, Commands.PORT_STATE_12_19, 0b00000000)
        write_to_spi(spi, Commands.PORT_STATE_12_19, 0b00000000)#chip3 (left)
        cs(1)
        time.sleep_ms(1)
        cs(0)
        write_to_spi(spi, Commands.PORT_STATE_20_27, 0b00000000)#chip1 (right)
        write_to_spi(spi, Commands.PORT_STATE_20_27, 0b00000000)
        write_to_spi(spi, Commands.PORT_STATE_20_27, 0b00000000)#chip3 (left)
        cs(1)
        time.sleep_ms(1)
        cs(0)
        write_to_spi(spi, Commands.PORT_STATE_28_31, 0b00000000)#chip1 (right)
        write_to_spi(spi, Commands.PORT_STATE_28_31, 0b00000000)
        write_to_spi(spi, Commands.PORT_STATE_28_31, 0b00000000)#chip3 (left)
        cs(1)
        return

    
    for swi,value in spi_registers.items():
        cs(0)
        if swiname == swi:
            if debug ==1:
                print (swi)
                print (value[0])
                print (value[1])
                print (value[2])
                
           
            if value[1] == 1:
                a = int(value[2], 16)
                #a=hex(a)
                write_to_spi(spi, a, act)
                if debug ==1:
                    print ('chip1', hex(a),act)
            else:
                if debug ==1:
                    print ('chip1 NO_ACTION')
                write_to_spi(spi, Commands.NO_ACTION, 0x00)
            if value[1] == 2:
                a = int(value[2], 16)
                #a=hex(a)
                write_to_spi(spi, a, act)#0b00000001
                if debug ==1:
                    print ('chip2', hex(a),act)
                
            else:
                if debug ==1:
                    print ('chip2 NO_ACTION')
                write_to_spi(spi, Commands.NO_ACTION, act)
            if value[1] == 3:
                a = int(value[2], 16)
                #a=hex(a)
                write_to_spi(spi, a, act)
                if debug ==1:
                    print ('chip3', hex(a),act)
            else:
                if debug ==1:
                    print ('chip3 NO_ACTION')
                write_to_spi(spi, Commands.NO_ACTION, 0x00)
        cs(1)






