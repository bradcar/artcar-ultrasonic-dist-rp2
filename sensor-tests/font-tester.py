# Raspberry Pi Pico 2 project - ArtCar ultrasonic - Oct 2024
#
# Font tester based on Peter Hinch's great work
# notice need short_writer.py to make it work

# short_writer Code created by: Charlotte Swift  - THANKS!!
# https://www.youtube.com/watch?v=kxlN1knBpQ0
# modified to fix a few errors
# saved as writer_short.py in /lib on Raspberry Pi
#
# adaptation bradcar

from machine import Pin
import machine
import time
from os import uname
from sys import implementation
from time import sleep as zzz

#from writer import Writer
from time import sleep as zzz
from writer_short import Writer

# Peter Hinch fonts: https://github.com/peterhinch/micropython-font-to-py
# other fonts: https://github.com/easytarget/microPyEZfonts/tree/main/examples/fonts
import freesans14 as font0
import freesans17 as font1
import freesans20 as font2
import ez5x7 as font3
#ic2
#from machine import I2C
#from ssd1306 import SSD1306_I2C
from ssd1306 import SSD1306_SPI
import framebuf

# pins
# https://www.tomshardware.com/how-to/oled-display-raspberry-pi-pico
#i2c=I2C(0,sda=Pin(0), scl=Pin(1), freq=400000)
#oled = SSD1306_I2C(128, 64, i2c)

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

oled_spi = machine.SPI(1)
print("oled_spi:", oled_spi)
oled = SSD1306_SPI(128, 64, oled_spi, dc, res, cs)

#  DISPLAY IMAGES
# image2cpp (convert png into C code): https://javl.github.io/image2cpp/
# then replace ", 0"  with "\"
# const unsigned char bitmap_artcar_image[] PROGMEM = {0xc9, 0x04, 0x59, ...
# goes to bitmap_artcar_image=bytearray(b'\xc9\x04\x59

# 'art-car-imag', 56x15px
bitmap_artcar_image=bytearray(b'\xc9\x04\x59\x11\x0c\x08\x43\xc8\x82\x8e\x90\x93\x10\x93\xe0\x63\x00\x78\xe0\xe3\x07\xff\x9f\xff\xff\xff\xfc\xff\xc0\x00\x00\x00\x00\x00\x03\x40\x1c\xf3\xe3\x8e\x78\x02\x47\x22\x88\x84\x51\x44\xe2\x45\x22\x88\x84\x11\x44\xa2\x47\x22\xf0\x84\x11\x78\xe2\x20\x3e\x88\x84\x1f\x44\x04\x10\x22\x88\x84\x51\x44\x08\x0c\x22\x88\x83\x91\x44\x30\x03\x00\x00\x00\x00\x00\xc0\x00\xff\xff\xff\xff\xff\x00\x00\x00\x00\x00\x00\x00\x00')

# projects: https://www.tomshardware.com/news/raspberry-pi
# fonts: https://www.youtube.com/watch?v=kxlN1knBpQ0

# startup code
# startup code
print("Starting...")
print("====================================")
print(implementation[0], uname()[3],
      "\nrun on", uname()[4])
print("====================================")

# at start display artcar image at top of oled
oled.fill(0)
fb = framebuf.FrameBuffer(bitmap_artcar_image,56,15, framebuf.MONO_HLSB)
oled.blit(fb,40,8)
oled.text("1234567890", 26, 24)

font_writer = Writer(oled,font3, verbose=False)
font_writer_2 = Writer(oled,font2, verbose=False)

while True:
    for char in range (32, 126):
        font_writer.set_textpos(0,32)
        font_writer.printstring(f"{char}: {chr(char)} ")
        print(f"{char}")
        
        font_writer_2.set_textpos(0,38)
        font_writer_2.printstring(f"{char}: {chr(char)} ")
        print(f"{char}")
        
        zzz(.3)
        oled.show()

# main loop

    led.toggle()
    time.sleep_ms(500)

