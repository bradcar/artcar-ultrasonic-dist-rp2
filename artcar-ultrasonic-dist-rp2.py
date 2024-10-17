# Raspberry Pi Pico 2 project - ArtCar ultrasonic - Oct 2024
#   based on Brad's Arduino UNO and 128x64 OLED Display for rear parking sensor
#      - speed of sound with temp & humidity correction
#      - front & Rear facing changed with button
#      - in/cm F/C changed with button
#      Raspberry Pi GitHub: https://github.com/bradcar/artcar-ultrasonic-dist-rp2
#      Arduino GitHub:      https://github.com/bradcar/art-car-ultrasonic-dist
#
# project Based on GREAT work by upir!!!
#     youtube full video: https://youtu.be/gg08H-6Z1Lo
#     created by upir, 2022

from machine import Pin, Timer
import machine
import dht
import time
import utime
import math
import os
import sys
from time import sleep as zzz

#ic2
#from machine import I2C
#from ssd1306 import SSD1306_I2C
from ssd1306 import SSD1306_SPI
import framebuf

#gc.collect() before executing gc.mem_free()

# pins
# https://www.tomshardware.com/how-to/oled-display-raspberry-pi-pico
#i2c=I2C(0,sda=Pin(0), scl=Pin(1), freq=400000)
#oled = SSD1306_I2C(128, 64, i2c)

dht_pin = machine.Pin(2)
trigger = Pin(3, Pin.OUT)
echo = Pin(4, Pin.IN)
led = Pin(25, Pin.OUT)

# https://coxxect.blogspot.com/2024/10/multi-ssd1306-oled-on-raspberry-pi-pico.html
# spi pins 10,11,12, 13
# scl (SCLK) gp10 
# SDA (MOSI) gp11
# RES (RST)  gp12
# DC         gp13
# CS dummy (not connected but assigned gp8
cs  = machine.Pin(9)    #dummy (any un-used pin), no connection
res = machine.Pin(12)
dc  = machine.Pin(13)
dht_sensor = dht.DHT22(dht_pin)

oled_spi = machine.SPI(1)
print("oled_spi:", oled_spi)
oled = SSD1306_SPI(128, 64, oled_spi, dc, res, cs)

# Constants and setup
NUMBER_OF_SENSORS = 1
MIN_DIST = 2
MAX_DIST = 100
dist_step_01 = MIN_DIST + round((MAX_DIST - MIN_DIST) / 4.0 * 1.0)
dist_step_02 = MIN_DIST + round((MAX_DIST - MIN_DIST) / 4.0 * 2.0)
dist_step_03 = MIN_DIST + round((MAX_DIST - MIN_DIST) / 4.0 * 3.0)
dist_step_04 = MIN_DIST + round((MAX_DIST - MIN_DIST) / 4.0 * 4.0)

#  DISPLAY IMAGES
# image2cpp (convert png into C code): https://javl.github.io/image2cpp/
# then replace ", 0"  with "\"
# const unsigned char bitmap_artcar_image[] PROGMEM = {0xc9, 0x04, 0x59, ...
# goes to bitmap_artcar_image=bytearray(b'\xc9\x04\x59

# 'art-car-imag', 56x15px
#bitmap_artcar_image=bytearray(b'\xc9\x04\x59\x11\x0c\x08\x43\xc8\x82\x8e\x90\x93\x10\x93\xe0\x63\x00\x78\xe0\xe3\x07\xff\x9f\xff\xff\xff\xfc\xff\xc0\x00\x00\x00\x00\x00\x03\x40\x1c\xf3\xe3\x8e\x78\x02\x47\x22\x88\x84\x51\x44\xe2\x45\x22\x88\x84\x11\x44\xa2\x47\x22\xf0\x84\x11\x78\xe2\x20\x3e\x88\x84\x1f\x44\x04\x10\x22\x88\x84\x51\x44\x08\x0c\x22\x88\x83\x91\x44\x30\x03\x00\x00\x00\x00\x00\xc0\x00\xff\xff\xff\xff\xff\x00\x00\x00\x00\x00\x00\x00\x00')
bitmap_artcar_image = bytearray([
  0xc9, 0x04, 0x59, 0x11, 0x0c, 0x08, 0x43, 0xc8, 0x82, 0x8e, 0x90, 0x93, 0x10, 0x93, 0xe0, 0x63,
  0x00, 0x78, 0xe0, 0xe3, 0x07, 0xff, 0x9f, 0xff, 0xff, 0xff, 0xfc, 0xff, 0xc0, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x03, 0x40, 0x1c, 0xf3, 0xe3, 0x8e, 0x78, 0x02, 0x47, 0x22, 0x88, 0x84, 0x51, 0x44,
  0xe2, 0x45, 0x22, 0x88, 0x84, 0x11, 0x44, 0xa2, 0x47, 0x22, 0xf0, 0x84, 0x11, 0x78, 0xe2, 0x20,
  0x3e, 0x88, 0x84, 0x1f, 0x44, 0x04, 0x10, 0x22, 0x88, 0x84, 0x51, 0x44, 0x08, 0x0c, 0x22, 0x88,
  0x83, 0x91, 0x44, 0x30, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0xff, 0xff, 0xff, 0xff,
  0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
])

loopTime = time.ticks_ms()
# init speed_sound if dht22 sensor fails
speed_sound = 344.11
temp_c = 0.0
humidity = 0.0

rear_sensors = False
metric = False
error_state = False
debug = False


def blink(timer):
    """
    Callback function to toggle the LED state, creating a blinking effect.
    To enable background blinking call with:
    timecall.init(freq=2.5, mode=Timer.PERIODIC, callback=blink)
    
    Args:
        timer: The Timer object triggering the callback.        
    """
    global led
    led.toggle()
    
def calc_speed_sound(temp_c, humidity):
    global error_state, debug
    #update speed of sound using temp & humidity from dht22
    try:
        dht_sensor.measure()
        temp_c = dht_sensor.temperature()
        temp_f = (temp_c * 9.0 / 5.0) + 32.0
        humidity = dht_sensor.humidity()
        
        # Speed sound with temp correction: (20.05 * sqrt(273.16 + temp_c))
        # online temp/humid calc: http://resource.npl.co.uk/acoustics/techguides/speedair/
        # created spreadsheet of diffs between online temp/humid and temp formula
        # did a 2d linear fit to create my own correction, error is now +/-0.07%
        # valid for 0C to 30C temp & 75 to 102 kPa pressure
        speed_sound = (20.05 * math.sqrt(273.16 + temp_c)) \
                      + (0.0006545 * humidity + 0.00475) * temp_c \
                      + (0.001057 * humidity + 0.07121)

        if debug:
            #print(f"Temp °C : {temp_c:.2f}")
            print(f"Temp °F : {temp_f:.2f}")
            print(f"Humidity: {humidity:.2f}%")
            print(f"Speed Sound: {speed_sound:.1f} m/s\n")
    except Exception as e:
        print("Error reading DHT22:", str(e))
    return(speed_sound)

    
def ultrasonic_distance(speed, timeout=50000):  # timeout in microseconds
    """
    Measure the distance using an ultrasonic sensor with a timeout.
     * tried with 3.3v, but really need 5v for ultrasonic & display
     * todo determine if this should be 30000, like in Arduino code
     * todo create consistent error handling throughout code

    Args:
        speed (float): Speed of sound in m/s.
        timeout (int): Maximum microseconds to wait for the echo signal return
        
    Returns:
        float: cm distance or
               -1 if the measurement times out waiting for each to go high
               -2 if the measurement times out waiting for each to go low
    """
    trigger.low()
    utime.sleep_us(2)
    trigger.high()
    utime.sleep_us(5)
    trigger.low()

    # Wait for echo to go high, with timeout
    start = utime.ticks_us()
    while echo.value() == 0:
        if utime.ticks_diff(utime.ticks_us(), start) > timeout:
            print("error ultrasonic: too close or malfunction.")
            return 0    # return max distance

    signal_off = utime.ticks_us()

    # Wait for echo to go low, with timeout
    start = utime.ticks_us()
    while echo.value() == 1:
        if utime.ticks_diff(utime.ticks_us(), start) > timeout:
            print("error ultrasonic: no signal returned, too far.")
            return 300  # return max distance

    signal_on = utime.ticks_us()

    # Calculate cm distance
    return (signal_on - signal_off) * (speed / 20000.0)

def display_environment(temp_c, humidity, speed_sound, dist):
    global metric, error_state, debug
    
    # never display text upside down
#     oled.rotate(True)
    if not error_state:
        temp_f = (temp_c * 9.0 / 5.0) + 32.0
        oled.fill(0)
        if metric:
            oled.text(f"Temp = {temp_c:.1f}C", 0, 0)
        else:
            oled.text(f"Temp = {temp_f:.1f}F", 0, 0)
        oled.text(f"Humid= {humidity:.1f}%", 0, 12)
        if metric:
            oled.text(f"Sound={speed_sound:.1f}m/s", 0, 24)
            oled.text(f"Dist= {dist:.1f}cm", 0, 55)
        else:
            oled.text(f"Sound={speed_sound*3.28084:.0f}ft/s", 0, 24)
            oled.text(f"Dist= {dist/2.54:.0f}in", 0, 55)
        
        fb = framebuf.FrameBuffer(bitmap_artcar_image,56,15, framebuf.MONO_HLSB)
        oled.blit(fb,22,36)
        oled.show()
        return


# projects: https://www.tomshardware.com/news/raspberry-pi
# make own fonts: https://www.youtube.com/watch?v=kxlN1knBpQ0 modified short_writer.py
#   based on Peter Hinch's conversion, writer, and larger 14px to 20px fonts 
#   https://github.com/peterhinch/micropython-font-to-py
#   https://github.com/easytarget/microPyEZfonts/blob/main/examples/fonts/ezFBfont_07_font_tiny5_ascii.py

# startup code
print("Starting...")
print("====================================")
print(sys.implementation[0], os.uname()[3],
      "\nrun on", os.uname()[4])
print("====================================")
print(f"Default Speed Sound: {speed_sound:.1f} m/s\n")
# if want blinking happening in background
#timecall = Timer()
#timecall.init(freq=2.5, mode=Timer.PERIODIC, callback=blink)

# at start display artcar image at top of oled
oled.fill(0)
fb = framebuf.FrameBuffer(bitmap_artcar_image,56,15, framebuf.MONO_HLSB)
oled.blit(fb,40,8)
oled.show()
zzz(1)

# check status of DHT sensor
try:
    dht_sensor.measure()
    temp_c = dht_sensor.temperature()
    temp_f = (temp_c * 9.0 / 5.0) + 32.0
    humidity = dht_sensor.humidity()
except Exception as e:
    oled.text("Error DHT22", 0, 24)
    oled.text(str(e), 0, 36)
    oled.show()
    temp_c = temp_f = humidity = speed_sound = 0.0  # Reset to default
    error_state = True

'''
Todo; think about ultrasonic errors and fix this
try:
    ultrasonic_distance(speed_sound, timeout=30000)
except Exception as e:
    oled.text("Error ultrasonic", 0, 24)
    #oled.text(str(e), 0, 36)
    oled.show()
    error_state = True
'''

# main loop
while True:    
# error with micropython rotate call oled.rotate display will rotate but can not be unrotated
#     if rear_sensors:
#         oled.rotate(False)
#     else:
#         oled.rotate(True)     
#    oled.rotate(True)
#    oled.rotate(False)

    elapsed_time = time.ticks_diff(time.ticks_ms(), loopTime)
    #if elapsed_time > 4000:
    if True:
        #print("Elapsed time:", elapsed_time, "msec\n")
        loopTime = time.ticks_ms()
        
        # calc speed of sound based on temp & humidity
        speed_sound = calc_speed_sound(temp_c, humidity)
        
        #get distance from ultrasonic sensor
        dist = ultrasonic_distance(speed_sound, timeout=30000)  # 30ms timeout 257cm
        
        # update dispay with results
        display_environment(temp_c, humidity, speed_sound, dist)
    #
    #Every loop do this
    led.toggle()
    time.sleep_ms(500)