# ds18b20 temp sensor (onewire protocol)
#
# https://randomnerdtutorials.com/raspberry-pi-pico--micropython/
#
# bradcar

import machine
import onewire
import ds18x20
from os import uname
from sys import implementation
import time
from time import sleep as zzz

ds_pin = machine.Pin(22)
ds_sensor = ds18x20.DS18X20(onewire.OneWire(ds_pin))

# startup code
print("Starting...")
print("====================================")
print(implementation[0], uname()[3],
      "\nrun on", uname()[4])
print("====================================")

# roms will be a list of sensors on same GPIO pin
roms = ds_sensor.scan()
print('Found DS devices: ', roms)

while True:
# Error catching code
#     try:
#         ds_sensor.convert_temp()
#     except onewire.OneWireError as error:
#         try:
#             print("DS temp onewire: " + str(error) + '\n')
#             ds_error = True
#         except OSError:
#             pass
    
    ds_sensor.convert_temp()
    # must sleep 750ms before read 1st value
    # zzz(10)   # tested to see if we can do this way ahead of time, it works
    zzz(.75)
  
    for rom in roms:
        print(rom)
        
        tempC = ds_sensor.read_temp(rom)
        tempF = tempC * (9/5) + 32
        print(f"temperature = {tempC:.2f}°C   = {tempF:.2f}°F\n")
    
    zzz(1)
