#import all requiered parts
import machine
import time
import onewire
import ds18x20
import utime
import ubinascii
import os

from micropython import const
from ulora import TTN, uLoRa

#Set the pinout for the RP2040
LORA_SCK = const(6)
LORA_MOSI = const(7)
LORA_MISO = const(4)
LORA_CS = const(5)
LORA_IRQ = const(8)
LORA_RST = const(9)


#LoRa configuration
LORA_DATARATE = "SF9BW125"  # Choose from several available

#Enter the Data for the TTN access here
DEVADDR = bytearray([INSERT DATA HERE])
NWKEY = bytearray([INSERT DATA HERE])
APP = bytearray([INSERT DATA HERE])
#Configure your contry accodtingly for Europe you may use ="EU"
TTN_CONFIG = TTN(DEVADDR, NWKEY, APP, country="EU")
FPORT = 1


#Initialize the RFM95
lora = uLoRa(
    cs=LORA_CS,
    sck=LORA_SCK,
    mosi=LORA_MOSI,
    miso=LORA_MISO,
    irq=LORA_IRQ,
    rst=LORA_RST,
    ttn_config=TTN_CONFIG,
    datarate=LORA_DATARATE,
    fport=FPORT
)

#OneWire setup
dat = machine.Pin(16)

# create the onewire object
ds = ds18x20.DS18X20(onewire.OneWire(dat))

#At this point the hardware is ready to go
# We will now scan for attached 1Wire devices

# scan for devices on the bus
roms = ds.scan()
print('found devices:', roms)
while len(roms ) == 0: 
    print("Sleep for 120 seconds")
    time.sleep(120)
    roms = ds.scan()
    print('found devices:', roms)
    
try:
    f = open('counter.txt','r')
    strval = f.read()
    f.close()
except OSError:  # open failed
   # handle the file open case
    f = open('counter.txt','w')
    f.write('0')
    f.close()
    print("Write new counter.txt with value=0")
    f = open('counter.txt','r')
    strval = f.read()
    f.close()
print(strval)
lora.frame_counter= int(strval)
print("Start with Framecounter="+str(lora.frame_counter))

while True:
    ds.convert_temp()
    time.sleep_ms(750)
    temp = 0
    for rom in roms:
        temp = ds.read_temp(rom)
        print(temp, end=' ')
        print()

    data = b""
    data += "T:"
    data += str(temp)
    print("Sending packet...", lora.frame_counter, ubinascii.hexlify(data))
    lora.send_data(data, len(data), lora.frame_counter)
    print(len(data), "bytes sent!")
    lora.frame_counter += 1
    f = open('counter.txt','w')
    f.write(str(lora.frame_counter))
    f.close()
    print("Sleep for 900 seconds")
    time.sleep(900)

