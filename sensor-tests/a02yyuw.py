# dougl code for a02yyuw waterproof UART sensor
# https://forum.makerforums.info/t/a02yyuw-waterproof-ultrasonic-sensor/87359/18

import os
import utime
import machine
# for A02YYUW ultrasonic Sensor, loops and outputs mm distance measured
# with Rx on the sensor floating, it will output data ever 300mS and that's how this is used.
# So only connect up the Green/Tx output of the sensor to GPIO-5(Aux1 Rx)
# power should be what every your device supports(3.3V or 5V). I used the Grove conn power.

#print sys info
print(os.uname())

#indicate program found a valid start of a sensor measurement
led_onboard = machine.Pin(25, machine.Pin.OUT)
led_onboard.value(0)     # onboard LED OFF

#setup uarts
#uart0 = machine.UART(0) #,9600)
#uart0.init(9600, bits=8, parity=None, stop=1) # init with given parameters
uart1 = machine.UART(1,9600)#,9600)
#uart1.init(9600, bits=8, parity=None, stop=1) # init with given parameters

#print uart info
#print(uart0) # Grove port #1 of Maker Pi  Tx=GPIO0, Rx=GPIO1
print(uart1) # Grove Port #3 of Maker Pi, Tx=GPIO4, Rx=GPIO5

data = [0,0,0,0]

goodData = False;
while True:
    #
    # When using the real sensor COMMENT OUT the following 6 lines 
    #uart0.write(b'\xFF') #sensor start flag
    #uart0.write(b'\x07') #sensor data-H
    #uart0.write(b'\xA1') #sensor data-L
    #uart0.write(b'\xA7') #sensor checkSum
    #uart0.write("hello") #bunch of other stuff to skip
    #utime.sleep(0.1)

    #NOTE: with Rx floating (internal pull up, data is sent every 300mS, if pull down every 100ms less accurate
    while uart1.any():
        dataRead=uart1.read(1)
        if dataRead == b'\xff':
            goodData = True
            data[0]=dataRead
            data[1]=uart1.read(1)
            data[2]=uart1.read(1)
            data[3]=uart1.read(1)
            led_onboard.value(1)
        else:
            led_onboard.value(0)
        
        #test 4 bytes of data read for validity
        for i in range(0,4):
            if type(data[i]) == type(None) : goodData=False; utime.sleep(0.1)
    if( goodData ) :        
        #debugging
        #for i in range(0,4):
        #    print("index=",i,"=",data[i])
        
        #validate the data
        dataSum = sum(bytes(data[0]+data[1]+data[2]))&0x00ff
        #debugging
        #print("sum = ", dataSum, "bytes =",hex(dataSum) )

        #checking if the calculated sum is the same as the last byte
        if dataSum is int.from_bytes(data[3],'big'):
            sensorReading = data[1] + data[2]
            print("sensor data=",int.from_bytes(sensorReading,'big'))
        utime.sleep(1)
        data = [0,0,0,0]
    goodData = False

print("should never get here")