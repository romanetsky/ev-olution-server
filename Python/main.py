
import time
import machine
from machine import Pin, SPI
import swi_spi
import sensors
import sys
import math



#import sensors
# from mock_machine import Pin, SPI

PW_En = machine.Pin(4, machine.Pin.OUT)
PW_En.value(0) #I4, (PW_En) should be HIGH to work with Pereferials
time.sleep_ms(100)
PW_En.value(1)

time.sleep_ms(100)


sensors.PAC_init()

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


def spi_exp_init():
    print("spi_exp_start init")
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

    print("spi_exp_finished init")
#----------------------------temp SPI expanders init---------------------------------    
spi_exp_init()


#swi_spi.set_swi('Com_N1', 0)

def ser16():
    
    time.sleep_ms(10)
    swi_spi.set_swi('Com_P1',0) #close siquence
    swi_spi.set_swi('Com_P2',0)
    time.sleep_ms(10)
    swi_spi.set_swi('Com5',0)
    time.sleep_ms(10)
    swi_spi.set_swi('Com_N1',0)
    swi_spi.set_swi('Com_N2',0)
    time.sleep_ms(10)
    
    
    
    time.sleep_ms(1)
    swi_spi.set_swi('Com_N1',1)
    swi_spi.set_swi('9D',1)
    swi_spi.set_swi('10C',1)
    swi_spi.set_swi('11C',1)
    swi_spi.set_swi('12C',1)
    swi_spi.set_swi('13C',1)
    swi_spi.set_swi('14C',1)
    swi_spi.set_swi('15C',1)
    swi_spi.set_swi('16C',1)
    swi_spi.set_swi('16A',1)
    
    swi_spi.set_swi('Com5',1)
    
    swi_spi.set_swi('1D',1)
    swi_spi.set_swi('2C',1)
    swi_spi.set_swi('3C',1)
    swi_spi.set_swi('4C',1)
    swi_spi.set_swi('5C',1)
    swi_spi.set_swi('6C',1)
    swi_spi.set_swi('7C',1)
    swi_spi.set_swi('8C',1)
    swi_spi.set_swi('8A',1)
    
    swi_spi.set_swi('Com_P2',1)
    
def close_all():
    time.sleep_ms(10)
    swi_spi.set_swi('Com_P1',0) #close siquence
    swi_spi.set_swi('Com_P2',0)
#     time.sleep_ms(10)
#     swi_spi.set_swi('8A',0)
#     time.sleep_ms(10)
#     swi_spi.set_swi('8C',0)
#     time.sleep_ms(10)
#     swi_spi.set_swi('7C',0)
#     time.sleep_ms(10)
#     swi_spi.set_swi('6C',0)
#     time.sleep_ms(10)
#     swi_spi.set_swi('5C',0)
#     time.sleep_ms(10)
#     swi_spi.set_swi('4C',0)
#     time.sleep_ms(10)
#     swi_spi.set_swi('3C',0)
#     time.sleep_ms(10)
#     swi_spi.set_swi('2C',0)
#     time.sleep_ms(10)
#     swi_spi.set_swi('1D',0)
    
    time.sleep_ms(10)
    
    swi_spi.set_swi('Com5',0)


    
    time.sleep_ms(10)
    swi_spi.set_swi('Com_N1',0)
    swi_spi.set_swi('Com_N2',0)
    
   
    time.sleep_ms(1)
    swi_spi.set_swi('close_all',0)
    
    

def ser1():
    time.sleep_ms(1)
    swi_spi.set_swi('Com_P1',0) #close siquence
    swi_spi.set_swi('close_all',0)
    swi_spi.set_swi('Com_N1',0)
    
def ser2():
    time.sleep_ms(1)
    swi_spi.set_swi('Com_P1',0) #close siquence
    swi_spi.set_swi('close_all',0)
    swi_spi.set_swi('Com_N1',0)
    
    swi_spi.set_swi('Com_N1',1)#open siquence
    swi_spi.set_swi('9D',1)
    swi_spi.set_swi('10C',1)
    swi_spi.set_swi('10A',1)
    swi_spi.set_swi('Com_P1',1)
    
def ser4():
    time.sleep_ms(1)
    swi_spi.set_swi('Com_P1',0) #close siquence
    swi_spi.set_swi('close_all',0)
    swi_spi.set_swi('Com_N1',0)
    
    swi_spi.set_swi('Com_N1',1)#open siquence
    swi_spi.set_swi('9D',1)
    swi_spi.set_swi('10C',1)
    swi_spi.set_swi('11C',1)
    swi_spi.set_swi('12C',1)
    swi_spi.set_swi('12A',1)
    swi_spi.set_swi('Com_P1',1)

def ser8_9():
    time.sleep_ms(1)
    close_all() #lose siquence
    
    time.sleep_ms(1)
    swi_spi.set_swi('Com_N1',1)
    swi_spi.set_swi('9D',1)
    swi_spi.set_swi('10C',1)
    swi_spi.set_swi('11C',1)
    swi_spi.set_swi('12C',1)
    swi_spi.set_swi('13C',1)
    swi_spi.set_swi('14C',1)
    swi_spi.set_swi('15C',1)
    swi_spi.set_swi('16C',1)
    swi_spi.set_swi('16A',1)
    swi_spi.set_swi('Com_P1',1)
    #--------------------------------------ser8 end-----------
def ser8_1():
    time.sleep_ms(1)
    close_all()
    
    time.sleep_ms(1)
    swi_spi.set_swi('Com_N2',1)
    swi_spi.set_swi('1D',1)
    swi_spi.set_swi('2C',1)
    swi_spi.set_swi('3C',1)
    swi_spi.set_swi('4C',1)
    swi_spi.set_swi('5C',1)
    swi_spi.set_swi('6C',1)
    swi_spi.set_swi('7C',1)
    swi_spi.set_swi('8A',1)
    swi_spi.set_swi('Com_P2',1)
    #--------------------------------------ser8_2 end-----------

def ser8_2_5():
    time.sleep_ms(1)
    close_all()
    
    time.sleep_ms(1)
    swi_spi.set_swi('Com_N2',1)
    swi_spi.set_swi('5D',1)
    swi_spi.set_swi('6C',1)
    swi_spi.set_swi('7C',1)
    swi_spi.set_swi('8C',1)
    swi_spi.set_swi('1C',1)
    swi_spi.set_swi('2C',1)
    swi_spi.set_swi('3C',1)
    swi_spi.set_swi('4C',1)
    swi_spi.set_swi('4A',1)
    swi_spi.set_swi('Com_P2',1)
    #--------------------------------------ser8_2 end-----------
    
def ser1_8ch():
    time.sleep_ms(10)
    swi_spi.set_swi('Com_P1',0) #close siquence
    swi_spi.set_swi('Com_P2',0)
    time.sleep_ms(10)
    swi_spi.set_swi('Com_N1',0)
    swi_spi.set_swi('Com_N2',0)
    time.sleep_ms(10)
    swi_spi.set_swi('close_all',0)
    time.sleep_ms(10)
    
    
    swi_spi.set_swi('Com_N2',1)#open siquence
    swi_spi.set_swi('8D',1)
    #swi_spi.set_swi('10C',1)
    swi_spi.set_swi('8A',1)
    swi_spi.set_swi('Com_P2',1)

def ser1_7ch():
    time.sleep_ms(10)
    swi_spi.set_swi('Com_P1',0) #close siquence
    swi_spi.set_swi('Com_P2',0)
    time.sleep_ms(10)
    swi_spi.set_swi('Com_N1',0)
    swi_spi.set_swi('Com_N2',0)
    time.sleep_ms(10)
    swi_spi.set_swi('close_all',0)
    time.sleep_ms(10)
    
    
    swi_spi.set_swi('Com_N2',1)#open `()
#     #ser9_16()
#     #ser16()
#     time.sleep_ms(50)
#     sensors.PAC_read(10)
#     
#     print("Icom on current:", sensors.ACS758_Iread())
#     time.sleep_ms(500)
#     close_all()
#     time.sleep_ms(3)
#     sensors.PAC_read(10)
#     print("Icom off current:", sensors.ACS758_Iread())
#     time.sleep_ms(500)
# i = 1    
# while i<17:
#     sensors.PAC_read(i)
#     i= i + 1


def selftest(ch):
    tolerance = 0.4
    expected_current = 0.9
    expected_d_current = 0.2
    expected_voltage = 3.5
    
    time.sleep_ms(1)
    swi_spi.set_swi('Com_P1',0) #close siquence
    swi_spi.set_swi('Com_P2',0)
    swi_spi.set_swi('Com_N1',0)
    swi_spi.set_swi('Com_N2',0)
    swi_spi.set_swi('close_all',0)
    
    if ch<9:
        swi_spi.set_swi('Com_N2',1)
        swi_spi.set_swi('Com_P2',1)
    else:
        swi_spi.set_swi('Com_N1',1)
        swi_spi.set_swi('Com_P1',1)
    
    lst_AD = ['A', 'D']
    for x in range(len(lst_AD)): #activating FET xAB
        #print(lst[x]) 
        swi_spi.set_swi(str(ch)+lst_AD[x], 1) #activating FET xA
        print(str(ch) + lst_AD[x], 1) #activating FET xB
        time.sleep_ms(10)
    s_read = sensors.PAC_read(ch) #monitoring voltage on cell
    print("voltage of channel", ch, "is", s_read[0],"V")
    if (abs(s_read[0]))<expected_voltage: #monitoring voltage on cell
        print("error in channel",ch, "voltage reading")
        close_all() #not closing since current should be tested
        sys.exit()
    print("Voltage in",ch, "reading is OK!")    
        
    
    
        
    lst_BC = ['B', 'C']
    for x in range(len(lst_BC)): 
        #print(lst[x]) 
        swi_spi.set_swi(str(ch)+lst_BC[x], 1)
        print(str(ch) + lst_BC[x], 1)
    time.sleep_ms(50) #check! why I need 50m delay!
    #s_read = sensors.PAC_read(ch)
    #print("voltage of channel", ch, "is", s_read[0],"V")
        
    s_read = sensors.PAC_read(10) #reding currnet of the output (currnetly on PAC of ch.10
    print("current of channel", ch, "is", s_read[1],"Amp")
    
    if not math.isclose(abs(s_read[1]), expected_current, rel_tol=tolerance):    
        print("error in channel",ch, "current reading")
        close_all()
        sys.exit()
    print("Current through",ch, "reading is OK!") 
       
    
    lst = ['A', 'B', 'C', 'D'] 
    for x in range(len(lst)): #deactivating and then activating each FET in the channel one by one and monitoring current
        swi_spi.set_swi(str(ch)+lst[x], 0)
        print(str(ch) + lst[x], 0)
        time.sleep_ms(50) #check delay, why needed 50m
        s_read = sensors.PAC_read(10) #reding currnet of the output (currnetly on PAC of ch.10
        print("current of channel", ch, "is", s_read[1],"Amp") #checking current when FET deactivated 
        if abs(s_read[1]) > expected_d_current:    
            print("error in channel",ch, "FET", str(ch) + lst[x], "current reading")
            print("FET", str(ch) + lst[x], "mostlikely constantly SHORTING! and burned")
            close_all()
            sys.exit()
            
        
        #time.sleep_ms(500)
        swi_spi.set_swi(str(ch)+lst[x], 1)
        print(str(ch) + lst[x], 1)
        time.sleep_ms(10)
        s_read = sensors.PAC_read(10) #reding currnet of the output (currnetly on PAC of ch.10
        print("current of channel", ch, "is", s_read[1],"Amp")
        if not math.isclose(abs(s_read[1]), expected_current, rel_tol=tolerance):    
            print("error in channel",ch, "FET", str(ch) + lst[x], "current reading")
            print("FET", str(ch) + lst[x], "mostlikely constantly DISCONNECTING! and burned")
            close_all()
            sys.exit()
        print(str(ch) + lst[x], "is OK!")
        
    if ch<9: #COMFETS test
        swi_spi.set_swi('Com_N2',0)
        swi_spi.set_swi('Com_P2',1)
        
        time.sleep_ms(10)
        
        s_read = sensors.PAC_read(10) #reding currnet of the output (currnetly on PAC of ch.10
        print("current of channel", ch, "is", s_read[1],"Amp")
        if abs(s_read[1]) > expected_d_current:    
            print("error in", 'Com_N2', "current reading")
            print("FET", 'Com_N2', "mostlikely constantly SHORTING! and burned")
            close_all()
            sys.exit()
        print("Com_N2 is OK!")     
        swi_spi.set_swi('Com_N2',1)
        
        swi_spi.set_swi('Com_P2',0)
        
        time.sleep_ms(10)
        
        s_read = sensors.PAC_read(10) #reding currnet of the output (currnetly on PAC of ch.10
        print("current of channel", ch, "is", s_read[1],"Amp")
        if abs(s_read[1]) > expected_d_current:        
            print("error in", 'Com_P2', "current reading")
            print("FET", 'Com_P2', "mostlikely constantly SHORTING! and burned")
            close_all()
            sys.exit()
        print("Com_P2 is OK!") 
    
        swi_spi.set_swi('Com_P2',1)
        
    else:
        swi_spi.set_swi('Com_N1',0)
        swi_spi.set_swi('Com_P1',1)
        time.sleep_ms(10)       
        s_read = sensors.PAC_read(10) #reding currnet of the output (currnetly on PAC of ch.10
        print("current of channel", ch, "is", s_read[1],"Amp")
        if abs(s_read[1]) > expected_d_current:       
            print("error in", 'Com_N1', "current reading")
            print("FET", 'Com_N1', "mostlikely constantly SHORTING! and burned")
            close_all()
            sys.exit()
        print("Com_N1 is OK!")
            
        swi_spi.set_swi('Com_N1',1)
        
        swi_spi.set_swi('Com_P1',0)
        time.sleep_ms(10)
        s_read = sensors.PAC_read(10) #reding currnet of the output (currnetly on PAC of ch.10
        print("current of channel", ch, "is", s_read[1],"Amp")
        if abs(s_read[1]) > expected_d_current:       
            print("error in", 'Com_P1', "current reading")
            print("FET", 'Com_P1', "mostlikely constantly SHORTING! and burned")
            close_all()
            sys.exit()
        print("Com_P1 is OK!")   
        swi_spi.set_swi('Com_P1',1)
    

    close_all()
    
  
        #time.sleep_ms(500)

def com5_test():
    tolerance = 0.3
    expected_current = 0.9
    expected_d_current = 0.3
    expected_voltage = 3.55
    close_all() #init state
    
    swi_spi.set_swi('Com_N1' ,1) #Shorting all required channels to connect voltage to Com5
    swi_spi.set_swi('Com_P2' ,1)
    swi_spi.set_swi('1A' ,1)
    swi_spi.set_swi('1B' ,1)
    swi_spi.set_swi('1C' ,1)
    swi_spi.set_swi('1D' ,1)
    swi_spi.set_swi('10A' ,1)
    swi_spi.set_swi('10B' ,1)
    swi_spi.set_swi('10C' ,1)
    swi_spi.set_swi('10D' ,1)
    
    swi_spi.set_swi('Com5',1) #Activating Com5
    
    time.sleep_ms(10)
    
    s_read = sensors.PAC_read(10) #reding currnet of the output (currnetly on PAC of ch.10
    print("current is", s_read[1],"Amp")
    if not math.isclose(abs(s_read[1]), expected_current, rel_tol=tolerance):    
        print("error in Com5 current reading")
        print("Com5 most likely constantly DISCONNECTING! and burned")
        close_all()
        sys.exit()
        
    
    time.sleep_ms(10)
    
    swi_spi.set_swi('Com5',0) #Deactivating Com5
    time.sleep_ms(10)
    s_read = sensors.PAC_read(10) #reding currnet of the output (currnetly on PAC of ch.10
    print("current is", s_read[1],"Amp")
    if abs(s_read[1]) > expected_d_current:
        print("error in Com5 current reading")
        print("Com5 mostlikely constantly SHORTING! and burned")
        close_all()
        sys.exit()
    print("Com5 is OK!")
    close_all()
        

        

def full_test():
    i=1
    
    while i<17:
        selftest(i)
        time.sleep_ms(10)
        close_all()
        time.sleep_ms(10)
        i=i+1
    com5_test()


def Current_test(ch):
    tolerance = 0.4
    expected_current = 0.9
    expected_d_current = 0.2
    expected_voltage = 3.5
    R_Load = 10
    
    time.sleep_ms(1)
    swi_spi.set_swi('Com_P1',0) #close siquence
    swi_spi.set_swi('Com_P2',0)
    swi_spi.set_swi('Com_N1',0)
    swi_spi.set_swi('Com_N2',0)
    swi_spi.set_swi('close_all',0)
    
    if ch<9:
        swi_spi.set_swi('Com_N2',1)
        swi_spi.set_swi('Com_P2',1)
    else:
        swi_spi.set_swi('Com_N1',1)
        swi_spi.set_swi('Com_P1',1)
        
    swi_spi.set_swi('1D',1)
    swi_spi.set_swi('2C',1)
    #swi_spi.set_swi('2D',1)
    swi_spi.set_swi('2A',1) 
    
#     lst_AD = ['A', 'D']
#     for x in range(len(lst_AD)): #activating FET xAB
#         #print(lst[x]) 
#         swi_spi.set_swi(str(ch)+lst_AD[x], 1) #activating FET xA
#         print(str(ch) + lst_AD[x], 1) #activating FET xB
    time.sleep_ms(40)
    s_read = sensors.PAC_read(ch) #monitoring voltage on cell
    print("voltage of channel", ch, "is", s_read[0],"V")
    print("Current of channel", ch, "is", s_read[1],"Amp")
    print("Calculated current is", s_read[0]/R_Load)
    print("delta current is", (abs(s_read[0])/R_Load)-s_read[1])
    s_read = sensors.PAC_read(10)
    print ("current of refference COM is", s_read[1],"Amp")
    s_read = sensors.PAC_read(1)
    print ("current of ch1 is", s_read[1],"Amp")


           
    close_all()
    
    
