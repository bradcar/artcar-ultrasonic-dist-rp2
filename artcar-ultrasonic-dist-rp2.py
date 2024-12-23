# Raspberry Pi Pico 2 project - ArtCar ultrasonic - Oct 2024
#
# Uses 3 ultrasonic sensors, uses oled SPI display to show measured
# temp & humid and uses them to calculate speeed of sound for accurate
# distance measurement.
#
# based on Brad's Arduino UNO and 128x64 OLED Display for rear&front parking sensor
#    - speed of sound with temp & humidity correction
#    - front- & Rear-facing changed with button #2
#      - on Raspberry Pi recalc offsets, and flip sensor graphics
#      - on arduino could rotate display and flip,text,some graphics(not sensor)
#    - in/cm F/C changed with button #1
#    - rear-sensors/front-sebnsors changed with button #2
#    - buttons debounced with efficient rp2 interrupts -- nice!
#    - ssd1309 SDI or I2C code (sw is ssd1306)
#
#    Raspberry Pi GitHub: https://github.com/bradcar/artcar-ultrasonic-dist-rp2
#    Arduino GitHub:      https://github.com/bradcar/art-car-ultrasonic-dist
#    # by bradcar
#
# project Based on GREAT work by upir!!!
#     YouTube full video: https://youtu.be/gg08H-6Z1Lo
#     created by upir, 2022
#
# https://micropython.org/download/RPI_PICO2/ for latest .uf2 preview
# https://docs.micropython.org/en/latest/esp8266/tutorial/ssd1306.html
# pycharm stubs
#   : https://micropython-stubs.readthedocs.io/en/main/packages.html#mp-packages
#
# TODOs
#  * add button #3 for switching between showing car on oled or

import time
from math import sqrt, log
from os import uname
from sys import implementation
from time import sleep as zzz

from bme680 import *

# import dht
import ds18x20
import machine
import onewire
import utime
# import RPi.GPIO as GPIO
from framebuf import FrameBuffer, MONO_HLSB
from machine import Pin, I2C
# ic2
# from machine import I2C
# from ssd1306 import SSD1306_I2C
from ssd1306 import SSD1306_SPI

# Constants for  setup, for >3 sensors need rewrite
DISP_WIDTH = 128
DISP_HEIGHT = 64
NUMBER_OF_SENSORS = 3


class SensorData:
    def __init__(self, echo_pin, trig_pin):
        self.trig_pin = trig_pin  # TRIG pin for the ultrasonic distance
        self.echo_pin = echo_pin  # ECHO pin for the ultrasonic distance
        self.cm = None  # measured distance in CM
        self.inch = None  # measured distance in inches
        self.label_xpos = 0  # x position of the distance label
        self.label_ypos = 0  # y position of the distance label
        self.label_width = 0  # calculated width of the distance string
        self.label_startpos_x = 0  # start X position for the label
        self.label_startpos_y = 0  # start Y position for the label
        self.label_endpos_x = 0  # end X position for the label
        self.label_endpos_y = 0  # end Y position for the label


# === PINS ===
# internal pins
on_pico_temp = machine.ADC(4)

# external pins
uart0 = machine.UART(0, 115200, tx=Pin(0), rx=Pin(1))
button_1 = Pin(2, Pin.IN, Pin.PULL_UP)  # interrupt cm/in button pins
button_2 = Pin(3, Pin.IN, Pin.PULL_UP)  # interrupt rear/front button pins
button_3 = Pin(15, Pin.IN, Pin.PULL_UP)  # extra


# https://www.tomshardware.com/how-to/oled-display-raspberry-pi-pico
# i2c=I2C(0,sda=Pin(12), scl=Pin(13), freq=400000)
# oled = SSD1306_I2C(DISP_WIDTH, DISP_HEIGHT, i2c)

# ssd1306 SDI SW setup for ssd1309 SDI
cs = machine.Pin(9)  # dummy (any un-used pin), no connection
# scl (SCLK) gp10
# SDA (MOSI) gp11
res = machine.Pin(12)  # RES (RST)  gp12
dc = machine.Pin(13)  # DC         gp13

# testing config: set up pins fo ultrasonic sensors using SensorData objects
# sensor = [
#     SensorData(echo_pin=4, trig_pin=3),
#     SensorData(echo_pin=20, trig_pin=21),
#     SensorData(echo_pin=16, trig_pin=17)
# ]
# final config: set up pins fo ultrasonic sensors using SensorData objects
sensor = [
    SensorData(echo_pin=20, trig_pin=21),
    SensorData(echo_pin=18, trig_pin=19),
    SensorData(echo_pin=16, trig_pin=17)
]

trigger = []
echo = []
# set up trigger & echo pings
for i in range(len(sensor)):
    trigger_pin = Pin(sensor[i].trig_pin, Pin.OUT)
    echo_pin =  Pin(sensor[i].echo_pin, Pin.IN)
    trigger.append(trigger_pin)
    echo.append(echo_pin)

ds_pin = machine.Pin(22)
led = Pin(25, Pin.OUT)
# Pin assignment  i2c1 
i2c = I2C(id=1, scl=Pin(27), sda=Pin(26))
buzzer = Pin(28, Pin.OUT)


bme = BME680_I2C(i2c=i2c)

ds_sensor = ds18x20.DS18X20(onewire.OneWire(ds_pin))
oled_spi = machine.SPI(1)
# print(f"oled_spi:{oled_spi}")
oled = SSD1306_SPI(DISP_WIDTH, DISP_HEIGHT, oled_spi, dc, res, cs)


# === Bitmap DISPLAY IMAGES ====
# image2cpp (convert png into C code): https://javl.github.io/image2cpp/
# const unsigned char bitmap_artcar_image[] PROGMEM = {0xc9,0x04,0x52, ...
# can be bitmap_artcar_image=bytearray(b'\xc9\x04\x59
#
# 'art-car-imag', 56x15px
bitmap_artcar_image_back = bytearray([
    0xc9,0x04,0x52,0x91,0x0c,0x08,0x43,0xc8,0x82,0x8e,0x90,0x93,0x10,0x93,0xe0,0x63,
    0x08,0x78,0xe0,0xe3,0x07,0xff,0x9f,0xff,0xff,0xff,0xfc,0xff,0xc0,0x00,0x00,0x00,
    0x00,0x00,0x03,0x40,0x1c,0xf3,0xe3,0x8e,0x78,0x02,0x42,0x22,0x88,0x84,0x51,0x44,
    0x42,0x42,0x22,0x88,0x84,0x11,0x44,0x42,0x42,0x22,0xf0,0x84,0x11,0x78,0x42,0x22,
    0x3e,0x88,0x84,0x1f,0x44,0x44,0x10,0x22,0x88,0x84,0x51,0x44,0x08,0x0c,0x22,0x88,
    0x83,0x91,0x44,0x30,0x03,0x00,0x00,0x00,0x00,0x00,0xc0,0x00,0xff,0xff,0xff,0xff,
    0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
])
# for front sensor
# 'art-car-image-flip', 56x15px
bitmap_artcar_image_front = bytearray([
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0xff,0x00,0x03,0x00,
    0x00,0x00,0x00,0x00,0xc0,0x0c,0x1c,0xf3,0xe3,0x8e,0x78,0x30,0x10,0x22,0x88,0x84,
    0x51,0x44,0x08,0x20,0x22,0x88,0x84,0x11,0x44,0x04,0x4f,0x22,0xf0,0x84,0x11,0x78,
    0xf2,0x4f,0x3e,0x88,0x84,0x1f,0x44,0xf2,0x4f,0x22,0x88,0x84,0x51,0x44,0xf2,0x40,
    0x22,0x88,0x83,0x91,0x44,0x02,0xc0,0x00,0x00,0x00,0x00,0x00,0x03,0xff,0x9f,0xff,
    0xff,0xff,0xf9,0xff,0xe0,0x63,0x08,0x78,0xe0,0xe7,0x07,0xc8,0x82,0x8e,0x90,0x93,
    0x10,0x93,0xc9,0x04,0x52,0x91,0x0c,0x08,0x43
])
# 'sensor_0a_off', 32x14px
bitmap_sensor_0a_off = bytearray([
    0x00,0x40,0x00,0x00,0x02,0xa8,0x00,0x00,0x05,0x55,0x00,0x00,0x02,0xaa,0xa0,0x00,
    0x05,0x55,0x54,0x00,0x0a,0xaa,0xaa,0xa0,0x15,0x55,0x55,0x50,0x0a,0xaa,0xaa,0xa0,
    0x01,0x55,0x55,0x40,0x00,0x2a,0xaa,0xa0,0x00,0x15,0x55,0x40,0x00,0x02,0xaa,0xa0,
    0x00,0x00,0x15,0x40,0x00,0x00,0x02,0xa0
])
# 'sensor_0a_on', 32x14px
bitmap_sensor_0a_on = bytearray([
    0x00,0xc0,0x00,0x00,0x03,0xf8,0x00,0x00,0x07,0xff,0x00,0x00,0x07,0xff,0xe0,0x00,
    0x0f,0xff,0xfe,0x00,0x0f,0xff,0xff,0xf0,0x1f,0xff,0xff,0xf0,0x0f,0xff,0xff,0xe0,
    0x03,0xff,0xff,0xe0,0x00,0x7f,0xff,0xe0,0x00,0x1f,0xff,0xe0,0x00,0x03,0xff,0xe0,
    0x00,0x00,0x3f,0xe0,0x00,0x00,0x03,0xe0
])
# 'sensor_0b_off', 32x16px
bitmap_sensor_0b_off = bytearray([
    0x02,0x00,0x00,0x00,0x05,0x00,0x00,0x00,0x02,0xa0,0x00,0x00,0x05,0x54,0x00,0x00,
    0x0a,0xaa,0x80,0x00,0x15,0x55,0x50,0x00,0x0a,0xaa,0xaa,0x80,0x15,0x55,0x55,0x50,
    0x0a,0xaa,0xaa,0xa8,0x05,0x55,0x55,0x50,0x00,0xaa,0xaa,0xa8,0x00,0x15,0x55,0x50,
    0x00,0x02,0xaa,0xa0,0x00,0x00,0x55,0x50,0x00,0x00,0x0a,0xa0,0x00,0x00,0x00,0x50
])
# 'sensor_0b_on', 32x16px
bitmap_sensor_0b_on = bytearray([
    0x02,0x00,0x00,0x00,0x07,0x80,0x00,0x00,0x07,0xf0,0x00,0x00,0x0f,0xfc,0x00,0x00,
    0x0f,0xff,0x80,0x00,0x1f,0xff,0xf8,0x00,0x1f,0xff,0xff,0x80,0x3f,0xff,0xff,0xf8,
    0x1f,0xff,0xff,0xf8,0x07,0xff,0xff,0xf8,0x01,0xff,0xff,0xf8,0x00,0x3f,0xff,0xf8,
    0x00,0x07,0xff,0xf0,0x00,0x00,0xff,0xf0,0x00,0x00,0x0f,0xf0,0x00,0x00,0x00,0xf0
])
# 'sensor_0c_off', 32x17px
bitmap_sensor_0c_off = bytearray([
    0x08,0x00,0x00,0x00,0x05,0x00,0x00,0x00,0x0a,0x80,0x00,0x00,0x15,0x50,0x00,0x00,
    0x2a,0xaa,0x00,0x00,0x15,0x55,0x40,0x00,0x2a,0xaa,0xaa,0x00,0x55,0x55,0x55,0x40,
    0x2a,0xaa,0xaa,0xaa,0x15,0x55,0x55,0x54,0x02,0xaa,0xaa,0xa8,0x00,0x55,0x55,0x54,
    0x00,0x0a,0xaa,0xa8,0x00,0x01,0x55,0x54,0x00,0x00,0x2a,0xa8,0x00,0x00,0x05,0x54,
    0x00,0x00,0x00,0x28
])
# 'sensor_0c_on', 32x17px
bitmap_sensor_0c_on = bytearray([
    0x08,0x00,0x00,0x00,0x0f,0x00,0x00,0x00,0x1f,0xc0,0x00,0x00,0x1f,0xf8,0x00,0x00,
    0x3f,0xff,0x00,0x00,0x3f,0xff,0xe0,0x00,0x7f,0xff,0xfe,0x00,0x7f,0xff,0xff,0xe0,
    0x7f,0xff,0xff,0xfe,0x1f,0xff,0xff,0xfe,0x03,0xff,0xff,0xfc,0x00,0xff,0xff,0xfc,
    0x00,0x1f,0xff,0xfc,0x00,0x03,0xff,0xfc,0x00,0x00,0x3f,0xfc,0x00,0x00,0x07,0xfc,
    0x00,0x00,0x00,0x3c
])
# 'sensor_0d_off', 32x18px
bitmap_sensor_0d_off = bytearray([
    0x00,0x00,0x00,0x00,0x28,0x00,0x00,0x00,0x55,0x00,0x00,0x00,0x2a,0x80,0x00,0x00,
    0x55,0x50,0x00,0x00,0xaa,0xaa,0x00,0x00,0x55,0x55,0x50,0x00,0xaa,0xaa,0xaa,0x00,
    0x55,0x55,0x55,0x50,0x2a,0xaa,0xaa,0xaa,0x05,0x55,0x55,0x54,0x02,0xaa,0xaa,0xaa,
    0x00,0x55,0x55,0x54,0x00,0x0a,0xaa,0xaa,0x00,0x01,0x55,0x54,0x00,0x00,0x2a,0xa8,
    0x00,0x00,0x01,0x54,0x00,0x00,0x00,0x08
])
# 'sensor_0d_on', 32x18px
bitmap_sensor_0d_on = bytearray([
    0x20,0x00,0x00,0x00,0x38,0x00,0x00,0x00,0x7f,0x00,0x00,0x00,0x7f,0xc0,0x00,0x00,
    0xff,0xf8,0x00,0x00,0xff,0xff,0x00,0x00,0xff,0xff,0xf0,0x00,0xff,0xff,0xfe,0x00,
    0x7f,0xff,0xff,0xf0,0x3f,0xff,0xff,0xfe,0x0f,0xff,0xff,0xfe,0x03,0xff,0xff,0xfe,
    0x00,0x7f,0xff,0xfe,0x00,0x1f,0xff,0xfe,0x00,0x01,0xff,0xfc,0x00,0x00,0x3f,0xfc,
    0x00,0x00,0x03,0xfc,0x00,0x00,0x00,0x1c
])
# 'sensor_1a_off', 32x9px
bitmap_sensor_1a_off = bytearray([
    0x05,0x55,0x55,0x40,0x0a,0xaa,0xaa,0xa0,0x05,0x55,0x55,0x40,0x0a,0xaa,0xaa,0xa0,
    0x05,0x55,0x55,0x40,0x0a,0xaa,0xaa,0xa0,0x05,0x55,0x55,0x40,0x0a,0xaa,0xaa,0xa0,
    0x15,0x55,0x55,0x50
])
# 'sensor_1a_on', 32x9px
bitmap_sensor_1a_on = bytearray([
    0x07,0xff,0xff,0xc0,0x0f,0xff,0xff,0xe0,0x0f,0xff,0xff,0xe0,0x0f,0xff,0xff,0xe0,
    0x0f,0xff,0xff,0xe0,0x0f,0xff,0xff,0xe0,0x0f,0xff,0xff,0xe0,0x0f,0xff,0xff,0xe0,
    0x1f,0xff,0xff,0xf0
])
# 'sensor_1b_off', 32x9px
bitmap_sensor_1b_off = bytearray([
    0x15,0x55,0x55,0x50,0x0a,0xaa,0xaa,0xa0,0x15,0x55,0x55,0x50,0x0a,0xaa,0xaa,0xa0,
    0x15,0x55,0x55,0x50,0x2a,0xaa,0xaa,0xa8,0x15,0x55,0x55,0x50,0x2a,0xaa,0xaa,0xa8,
    0x05,0x55,0x55,0x40
])
# 'sensor_1b_on', 32x9px
bitmap_sensor_1b_on = bytearray([
    0x1f,0xff,0xff,0xf0,0x1f,0xff,0xff,0xf0,0x1f,0xff,0xff,0xf0,0x1f,0xff,0xff,0xf0,
    0x3f,0xff,0xff,0xf8,0x3f,0xff,0xff,0xf8,0x3f,0xff,0xff,0xf8,0x3f,0xff,0xff,0xf8,
    0x0f,0xff,0xff,0xe0
])
# 'sensor_1c_off', 32x10px
bitmap_sensor_1c_off = bytearray([
    0x10,0x00,0x00,0x04,0x0a,0xaa,0xaa,0xa8,0x15,0x55,0x55,0x54,0x2a,0xaa,0xaa,0xaa,
    0x15,0x55,0x55,0x54,0x2a,0xaa,0xaa,0xaa,0x15,0x55,0x55,0x54,0x2a,0xaa,0xaa,0xaa,
    0x15,0x55,0x55,0x54,0x0a,0xaa,0xaa,0xa8
])
# 'sensor_1c_on', 32x10px
bitmap_sensor_1c_on = bytearray([
    0x18,0x00,0x00,0x0c,0x1f,0xff,0xff,0xfc,0x3f,0xff,0xff,0xfe,0x3f,0xff,0xff,0xfe,
    0x3f,0xff,0xff,0xfe,0x3f,0xff,0xff,0xfe,0x3f,0xff,0xff,0xfe,0x3f,0xff,0xff,0xfe,
    0x3f,0xff,0xff,0xfe,0x0f,0xff,0xff,0xf8
])
# 'sensor_1d_off', 40x10px
bitmap_sensor_1d_off = bytearray([
    0x02,0x80,0x00,0x00,0x28,0x01,0x55,0x55,0x55,0x50,0x02,0xaa,0xaa,0xaa,0xa8,0x01,
    0x55,0x55,0x55,0x50,0x02,0xaa,0xaa,0xaa,0xa8,0x05,0x55,0x55,0x55,0x54,0x02,0xaa,
    0xaa,0xaa,0xa8,0x05,0x55,0x55,0x55,0x54,0x02,0xaa,0xaa,0xaa,0xa8,0x00,0x55,0x55,
    0x55,0x40
])
# 'sensor_1d_on', 40x10px
bitmap_sensor_1d_on = bytearray([
    0x03,0x80,0x00,0x00,0x38,0x03,0xff,0xff,0xff,0xf8,0x03,0xff,0xff,0xff,0xf8,0x03,
    0xff,0xff,0xff,0xf8,0x03,0xff,0xff,0xff,0xf8,0x07,0xff,0xff,0xff,0xfc,0x07,0xff,
    0xff,0xff,0xfc,0x07,0xff,0xff,0xff,0xfc,0x07,0xff,0xff,0xff,0xfc,0x00,0xff,0xff,
    0xff,0xe0
])
# 'sensor_2a_off', 32x14px
bitmap_sensor_2a_off = bytearray([
    0x00,0x00,0x04,0x00,0x00,0x00,0x2a,0x80,0x00,0x01,0x55,0x40,0x00,0x0a,0xaa,0x80,
    0x00,0x55,0x55,0x40,0x0a,0xaa,0xaa,0xa0,0x15,0x55,0x55,0x50,0x0a,0xaa,0xaa,0xa0,
    0x05,0x55,0x55,0x00,0x0a,0xaa,0xa8,0x00,0x05,0x55,0x50,0x00,0x0a,0xaa,0x80,0x00,
    0x05,0x50,0x00,0x00,0x0a,0x80,0x00,0x00
])
# 'sensor_2a_on', 32x14px
bitmap_sensor_2a_on = bytearray([
    0x00,0x00,0x06,0x00,0x00,0x00,0x3f,0x80,0x00,0x01,0xff,0xc0,0x00,0x0f,0xff,0xc0,
    0x00,0xff,0xff,0xe0,0x1f,0xff,0xff,0xe0,0x1f,0xff,0xff,0xf0,0x0f,0xff,0xff,0xe0,
    0x0f,0xff,0xff,0x80,0x0f,0xff,0xfc,0x00,0x0f,0xff,0xf0,0x00,0x0f,0xff,0x80,0x00,
    0x0f,0xf8,0x00,0x00,0x0f,0x80,0x00,0x00
])
# 'sensor_2b_off', 32x16px
bitmap_sensor_2b_off = bytearray([
    0x00,0x00,0x00,0x40,0x00,0x00,0x00,0xa0,0x00,0x00,0x05,0x40,0x00,0x00,0x2a,0xa0,
    0x00,0x01,0x55,0x50,0x00,0x0a,0xaa,0xa8,0x01,0x55,0x55,0x50,0x0a,0xaa,0xaa,0xa8,
    0x15,0x55,0x55,0x50,0x0a,0xaa,0xaa,0xa0,0x15,0x55,0x55,0x00,0x0a,0xaa,0xa8,0x00,
    0x05,0x55,0x40,0x00,0x0a,0xaa,0x00,0x00,0x05,0x50,0x00,0x00,0x0a,0x00,0x00,0x00
])
# 'sensor_2b_on', 32x16px
bitmap_sensor_2b_on = bytearray([
    0x00,0x00,0x00,0x40,0x00,0x00,0x01,0xe0,0x00,0x00,0x0f,0xe0,0x00,0x00,0x3f,0xf0,
    0x00,0x01,0xff,0xf0,0x00,0x1f,0xff,0xf8,0x01,0xff,0xff,0xf8,0x1f,0xff,0xff,0xfc,
    0x1f,0xff,0xff,0xf8,0x1f,0xff,0xff,0xe0,0x1f,0xff,0xff,0x80,0x1f,0xff,0xfc,0x00,
    0x0f,0xff,0xe0,0x00,0x0f,0xff,0x00,0x00,0x0f,0xf0,0x00,0x00,0x0f,0x00,0x00,0x00
])
# 'sensor_2c_off', 32x17px
bitmap_sensor_2c_off = bytearray([
    0x00,0x00,0x00,0x10,0x00,0x00,0x00,0xa0,0x00,0x00,0x01,0x50,0x00,0x00,0x0a,0xa8,
    0x00,0x00,0x55,0x54,0x00,0x02,0xaa,0xa8,0x00,0x55,0x55,0x54,0x02,0xaa,0xaa,0xaa,
    0x55,0x55,0x55,0x54,0x2a,0xaa,0xaa,0xa8,0x15,0x55,0x55,0x40,0x2a,0xaa,0xaa,0x00,
    0x15,0x55,0x50,0x00,0x2a,0xaa,0x80,0x00,0x15,0x54,0x00,0x00,0x2a,0xa0,0x00,0x00,
    0x14,0x00,0x00,0x00
])
# 'sensor_2c_on', 32x17px
bitmap_sensor_2c_on = bytearray([
    0x00,0x00,0x00,0x10,0x00,0x00,0x00,0xf0,0x00,0x00,0x03,0xf8,0x00,0x00,0x1f,0xf8,
    0x00,0x00,0xff,0xfc,0x00,0x07,0xff,0xfc,0x00,0x7f,0xff,0xfe,0x07,0xff,0xff,0xfe,
    0x7f,0xff,0xff,0xfe,0x7f,0xff,0xff,0xf8,0x3f,0xff,0xff,0xc0,0x3f,0xff,0xff,0x00,
    0x3f,0xff,0xf8,0x00,0x3f,0xff,0xc0,0x00,0x3f,0xfc,0x00,0x00,0x3f,0xe0,0x00,0x00,
    0x3c,0x00,0x00,0x00
])
# 'sensor_2d_off', 32x18px
bitmap_sensor_2d_off = bytearray([
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x28,0x00,0x00,0x01,0x54,0x00,0x00,0x02,0xa8,
    0x00,0x00,0x15,0x54,0x00,0x00,0xaa,0xaa,0x00,0x15,0x55,0x54,0x00,0xaa,0xaa,0xaa,
    0x15,0x55,0x55,0x54,0xaa,0xaa,0xaa,0xa8,0x55,0x55,0x55,0x40,0xaa,0xaa,0xaa,0x80,
    0x55,0x55,0x54,0x00,0xaa,0xaa,0xa0,0x00,0x55,0x55,0x00,0x00,0x2a,0xa8,0x00,0x00,
    0x55,0x00,0x00,0x00,0x20,0x00,0x00,0x00
])
# 'sensor_2d_on', 32x18px
bitmap_sensor_2d_on = bytearray([
    0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x38,0x00,0x00,0x01,0xfc,0x00,0x00,0x07,0xfc,
    0x00,0x00,0x3f,0xfe,0x00,0x01,0xff,0xfe,0x00,0x1f,0xff,0xfe,0x00,0xff,0xff,0xfe,
    0x1f,0xff,0xff,0xfc,0xff,0xff,0xff,0xf8,0xff,0xff,0xff,0xe0,0xff,0xff,0xff,0x80,
    0xff,0xff,0xfc,0x00,0xff,0xff,0xf0,0x00,0x7f,0xff,0x00,0x00,0x7f,0xf8,0x00,0x00,
    0x7f,0x80,0x00,0x00,0x70,0x00,0x00,0x00
])
# Declare global variables for flipped 1 bitmaps
flipped_bitmap_sensor_0a_off = bytearray()
flipped_bitmap_sensor_0a_on = bytearray()
flipped_bitmap_sensor_0b_off = bytearray()
flipped_bitmap_sensor_0b_on = bytearray()
flipped_bitmap_sensor_0c_off = bytearray()
flipped_bitmap_sensor_0c_on = bytearray()
flipped_bitmap_sensor_0d_off = bytearray()
flipped_bitmap_sensor_0d_on = bytearray()
# Declare global variables for flipped 2 bitmaps
flipped_bitmap_sensor_1a_off = bytearray()
flipped_bitmap_sensor_1a_on = bytearray()
flipped_bitmap_sensor_1b_off = bytearray()
flipped_bitmap_sensor_1b_on = bytearray()
flipped_bitmap_sensor_1c_off = bytearray()
flipped_bitmap_sensor_1c_on = bytearray()
flipped_bitmap_sensor_1d_off = bytearray()
flipped_bitmap_sensor_1d_on = bytearray()
# Declare global variables for flipped 3 bitmaps
flipped_bitmap_sensor_2a_off = bytearray()
flipped_bitmap_sensor_2a_on = bytearray()
flipped_bitmap_sensor_2b_off = bytearray()
flipped_bitmap_sensor_2b_on = bytearray()
flipped_bitmap_sensor_2c_off = bytearray()
flipped_bitmap_sensor_2c_on = bytearray()
flipped_bitmap_sensor_2d_off = bytearray()
flipped_bitmap_sensor_2d_on = bytearray()
#
# rear 'unit_cm', 24x10px
bitmap_unit_cm = bytearray([
    0xf0,0x03,0xc0,0x80,0x00,0x40,0xbe,0xff,0x40,0xb6,0xdb,0x40,0x30,0xdb,0x00,0x30,
    0xdb,0x00,0xb6,0xdb,0x40,0xbe,0xdb,0x40,0x80,0x00,0x40,0xf0,0x03,0xc0
])
# 'unit in', 24x10px
bitmap_unit_in = bytearray([
    0xf0,0x03,0xc0,0x80,0x00,0x40,0x8c,0x7c,0x40,0x8c,0x6c,0x40,0x0c,0x6c,0x00,0x0c,
    0x6c,0x00,0x8c,0x6c,0x40,0x8c,0x6c,0x40,0x80,0x00,0x40,0xf0,0x03,0xc0
])
# 'degree-temp', 24x10px
degree_temp = bytearray([
    0xf0,0x00,0x0f,0x80,0x00,0x01,0x80,0x00,0x0d,0x80,0x00,0x0d,0x00,0x00,0x00,0x00,
    0x00,0x00,0x80,0x00,0x01,0x80,0x00,0x01,0x80,0x00,0x01,0xf0,0x00,0x0f
])
MIN_CM_DIST = 2.0  # cm min for tiles
MAX_CM_DIST = 100.0  # cm max where text stops moving for tiles
DWELL_MS_LOOP = 100
SPEED_SOUND_20C_70H = 343.294
PDX_SLP_1013 = 1011.00
OVER_TEMP_WARNING = 70.0

# Button debouncer with efficient interrupts, which don't take CPU cycles!
# https://electrocredible.com/raspberry-pi-pico-external-interrupts-button-micropython/
def callback(pin):
    global button_1_pushed, button_2_pushed, button_3_pushed
    global debounce_1_time, debounce_2_time, debounce_3_time
    if pin == button_1 and (int(time.ticks_ms()) - debounce_1_time) > 500:
        button_1_pushed = True
        debounce_1_time = time.ticks_ms()
    elif pin == button_2 and (int(time.ticks_ms()) - debounce_2_time) > 500:
        button_2_pushed = True
        debounce_2_time = time.ticks_ms()
    elif pin == button_3 and (int(time.ticks_ms()) - debounce_3_time) > 500:
        button_3_pushed = True
        debounce_3_time = time.ticks_ms()


button_1.irq(trigger=Pin.IRQ_FALLING, handler=callback)
button_2.irq(trigger=Pin.IRQ_FALLING, handler=callback)
button_3.irq(trigger=Pin.IRQ_FALLING, handler=callback)

# Functions =================================================

def button1():
    global button_1_pushed
    if button_1_pushed:
        button_1_pushed = False
        print("button 1 pushed")
        return True
    else:
        return False


def button2():
    global button_2_pushed
    if button_2_pushed:
        button_2_pushed = False
        print("button 2 pushed")
        return True
    else:
        return False
    
def button3():
    global button_3_pushed
    if button_3_pushed:
        button_3_pushed = False
        return True
    else:
        return False

def initialize_flipped_bitmaps():
    """
    vertically flip the bitmaps and store them in global variables.
    """
    global flipped_bitmap_sensor_0a_off, flipped_bitmap_sensor_0a_on
    global flipped_bitmap_sensor_0b_off, flipped_bitmap_sensor_0b_on
    global flipped_bitmap_sensor_0c_off, flipped_bitmap_sensor_0c_on
    global flipped_bitmap_sensor_0d_off, flipped_bitmap_sensor_0d_on
    global flipped_bitmap_sensor_1a_off, flipped_bitmap_sensor_1a_on
    global flipped_bitmap_sensor_1b_off, flipped_bitmap_sensor_1b_on
    global flipped_bitmap_sensor_1c_off, flipped_bitmap_sensor_1c_on
    global flipped_bitmap_sensor_1d_off, flipped_bitmap_sensor_1d_on
    global flipped_bitmap_sensor_2a_off, flipped_bitmap_sensor_2a_on
    global flipped_bitmap_sensor_2b_off, flipped_bitmap_sensor_2b_on
    global flipped_bitmap_sensor_2c_off, flipped_bitmap_sensor_2c_on
    global flipped_bitmap_sensor_2d_off, flipped_bitmap_sensor_2d_on

    # flip bitmaps: for front tile 1, use rear flipped tile 3
    flipped_bitmap_sensor_0a_off = flip_bitmap_vert(bitmap_sensor_2a_off, 32, 14)
    flipped_bitmap_sensor_0a_on = flip_bitmap_vert(bitmap_sensor_2a_on, 32, 14)
    flipped_bitmap_sensor_0b_off = flip_bitmap_vert(bitmap_sensor_2b_off, 32, 16)
    flipped_bitmap_sensor_0b_on = flip_bitmap_vert(bitmap_sensor_2b_on, 32, 16)
    flipped_bitmap_sensor_0c_off = flip_bitmap_vert(bitmap_sensor_2c_off, 32, 17)
    flipped_bitmap_sensor_0c_on = flip_bitmap_vert(bitmap_sensor_2c_on, 32, 17)
    flipped_bitmap_sensor_0d_off = flip_bitmap_vert(bitmap_sensor_2d_off, 32, 18)
    flipped_bitmap_sensor_0d_on = flip_bitmap_vert(bitmap_sensor_2d_on, 32, 18)
    # flip bitmaps: middle tile 2 is same for front/rear
    flipped_bitmap_sensor_1a_off = flip_bitmap_vert(bitmap_sensor_1a_off, 32, 9)
    flipped_bitmap_sensor_1a_on = flip_bitmap_vert(bitmap_sensor_1a_on, 32, 9)
    flipped_bitmap_sensor_1b_off = flip_bitmap_vert(bitmap_sensor_1b_off, 32, 9)
    flipped_bitmap_sensor_1b_on = flip_bitmap_vert(bitmap_sensor_1b_on, 32, 9)
    flipped_bitmap_sensor_1c_off = flip_bitmap_vert(bitmap_sensor_1c_off, 32, 10)
    flipped_bitmap_sensor_1c_on = flip_bitmap_vert(bitmap_sensor_1c_on, 32, 10)
    flipped_bitmap_sensor_1d_off = flip_bitmap_vert(bitmap_sensor_1d_off, 40, 10)
    flipped_bitmap_sensor_1d_on = flip_bitmap_vert(bitmap_sensor_1d_on, 40, 10)
    # flip bitmaps: for front tile 1, use rear flipped tile 3
    flipped_bitmap_sensor_2a_off = flip_bitmap_vert(bitmap_sensor_0a_off, 32, 14)
    flipped_bitmap_sensor_2a_on = flip_bitmap_vert(bitmap_sensor_0a_on, 32, 14)
    flipped_bitmap_sensor_2b_off = flip_bitmap_vert(bitmap_sensor_0b_off, 32, 16)
    flipped_bitmap_sensor_2b_on = flip_bitmap_vert(bitmap_sensor_0b_on, 32, 16)
    flipped_bitmap_sensor_2c_off = flip_bitmap_vert(bitmap_sensor_0c_off, 32, 17)
    flipped_bitmap_sensor_2c_on = flip_bitmap_vert(bitmap_sensor_0c_on, 32, 17)
    flipped_bitmap_sensor_2d_off = flip_bitmap_vert(bitmap_sensor_0d_off, 32, 18)
    flipped_bitmap_sensor_2d_on = flip_bitmap_vert(bitmap_sensor_0d_on, 32, 18)
    return


# Helper functions to replace Arduino-specific functions
def constrain(val, min_val, max_val):
    return max(min_val, min(val, max_val))


def map_range(value, in_min, in_max, out_min, out_max):
    return int((value - in_min) * (out_max - out_min) // (in_max - in_min) + out_min)


def get_str_width(text):
    # MicroPython has 8x8 fonts  x*9-1 ?
    return len(text)


def onboard_temperature():
    """
    # pico data pico 2 rp2350 data sheet on page 1068
    # data sheet says 27C  is 0.706v, with a slope of -1.721mV per degree
    """
    adc_value = on_pico_temp.read_u16()
    volt = (3.3 / 65535) * adc_value
    celsius = 27 - (volt - 0.706) / 0.001721
    if debug: print(f"on chip temp = {celsius:.3f}C")
    return celsius


def calc_sea_level_pressure(hpa, meters):
    """
    Calculate the sea level pressure from the hpa pressure at a known elevation
    
    :param :sea_level: sea level hpa from closest airport
    :returns: sea level hpa based on known altitude
    """
    sea_level_pressure_hpa = hpa / (1.0 - (meters / 44330.0)) ** 5.255

    return sea_level_pressure_hpa


def bme_temp_humid_hpa_iaq_alt(sea_level):
    """
    read temp, humidity, pressure, Indoor Air Qualtiy (IAQ) from the BME680 sensor
    measurement takes ~189ms
     IAQ:       0- 50 good
               51-100 average
              101-150 poor
              151-200 bad
              201-300 worse
              301-500 very bad
    
    :param :sea_level: sea level hpa from closest airport
    :returns: temp_c, percent_humidity, hpa_pressure, iaq, meters, error string
    """
    # Read sensor data
    debug = True
    try:    
        temp_c = bme.temperature
        percent_humidity = bme.humidity
        hpa_pressure = bme.pressure
        gas_resist = bme.gas/100
        
        # derived sensor data
        meters = 44330.0 * (1.0 - (hpa_pressure/sea_level)**(1.0/5.255) )
        iaq = log(gas_resist) + 0.04 * percent_humidity
        
        if debug:
            print(f"BME680 Temp °C = {temp_c:.2f} C")
            print(f"BME680 Humidity = {percent_humidity:.1f} %")
            print(f"BME680 Pressure = {hpa_pressure:.2f} hPA")
            print(f"BME680 iaq = {iaq:.1f} IAQ lower better")
            print(f"BME680 Alt = {meters * 3.28084:.2f} feet \n")
            
    except OSError as e:
        print("BME680: Failed to read sensor.")
        return None, None, None, None, None, "ERROR_BME680:" + str(e)
    
    return temp_c, percent_humidity, hpa_pressure, iaq, meters, None


def outside_temp_ds_init():
    """
    init onewire, this is special code for this use we are only asking outside temp
    every 3 seconds so do init in setup, then read temp and prepare for next in
    outside_temp_ds()

    :returns: error string if happens
    """
    try:
        ds_sensor.convert_temp()
    except onewire.OneWireError as e:
        print("DS temp onewire: " + str(e) + '\n')
        return "ERROR_DS_TEMP_ONEWIRE:" + str(e)
    return None


def outside_temp_ds():
    """
    The ouside temp DS sensor (waterproof DS18B20), measurement takes ~6ms
    Because we must call ds_sensor.convert_temp() at least 750ms before read data,
    we will call outside_temp_ds_init() in setup (it wraps this function)

    Notice at the end of this function we call ds_sensor.convert_temp() in order to
    have enough time before we read again.
    We ONLY call this function once every 3 seconds, which means there will
    always be at least 3 seconds between .convert_temp() & .read_temp(rom)

    :returns: temp Celsius & error string
    """
    celsius = False
    # NOTE: if function is used to both setup & test temp, need to uncomment code below
    #     try:
    #         ds_sensor.convert_temp()
    #     except onewire.OneWireError as e:
    #         print("DS temp onewire: " + str(e) + '\n')
    #         return None, "ERROR_DS_TEMP_ONEWIRE:"+ str(e)
    # # must sleep 750ms before read 1st value
    #     zzz(.75)
    for rom_id in roms:
        celsius = ds_sensor.read_temp(rom_id)
        if debug: print(f"rom_id={rom_id}\n  tempC={celsius:.2f}")

    # prepare for next call, which in this main loop is ~3 seconds later
    try:
        ds_sensor.convert_temp()
    except onewire.OneWireError as e:
        print("DS temp onewire: " + str(e) + '\n')
        return None, "ERROR_DS_TEMP_ONEWIRE:" + str(e)

    return celsius, None


def calc_speed_sound(celsius, percent_humidity):
    """
    calc to update speed of sound based on temp & humidity from the DHT22 sensor

    :param celsius: temp in Celsius
    :param percent_humidity: % humidity
    :returns: speed meter/sec, error string
    """
    if celsius > 0 and percent_humidity > 0:
        # Speed sound with temp correction: (20.05 * sqrt(273.16 + temp_c))
        # online temp/humid calc:
        #   http://resource.npl.co.uk/acoustics/techguides/speedair/
        # created spreadsheet of diffs between above temp formula and online temp/humid
        # did a 2d linear fit to create my own correction, error is now +/-0.07%
        # valid for 0C to 30C temp & 75 to 102 kPa pressure
        meter_per_sec = (20.05 * sqrt(273.16 + celsius)) \
                        + (0.0006545 * percent_humidity + 0.00475) * celsius \
                        + (0.001057 * percent_humidity + 0.07121)
        return meter_per_sec, None
    else:
        return None, f"ERROR_INVALID_SOUND_SPEED:temp={celsius}C,humidity={percent_humidity}%"


def calculate_checksum(data_buffer):
    """
    Function to calculate checksum, make sure only 1 byte returned
    """
    return (data_buffer[0] + data_buffer[1] + data_buffer[2]) & 0x00ff


def ultrasonic_distance_pwm(j, speed_of_sound, timeout=50000):
    """
    Get ultrasonic distance from a sensor where ping and measure with a timeout.

    HC-SR04: most Send a 10uS high to trigger (default mode 1)
    JSN-SR04T: Send a 20uS high to trigger, instead of 10
    A02YYUW: only outputs UART serial data, must use ultrasonic_distance_uart(i)
    https://dronebotworkshop.com/waterproof-ultrasonic/

    :param j: index for ultrasonic sensor.
    :param speed_of_sound: speed of sound for distance calculation
    :param timeout: Max usecs to wait for the echo signal return. default 50ms.
    :returns: cm distance or error string.
    """
    trigger[j].low()
    utime.sleep_us(2)
    trigger[j].high()
    utime.sleep_us(20)  # JSN-SR04T: best with 20uS high to trigger instead of 10
    trigger[j].low()

    # Wait for echo to go high, with timeout
    start = utime.ticks_us()
    while echo[j].value() == 0:
        if utime.ticks_diff(utime.ticks_us(), start) > timeout:
            return 0.0, f"ULTRASONIC_ERROR: Sensor {i} - too close or malfunction."
    signal_off = utime.ticks_us()

    # Wait for echo to go low, with timeout
    start = utime.ticks_us()
    while echo[j].value() == 1:
        if utime.ticks_diff(utime.ticks_us(), start) > timeout:
            return 300.0, f"ULTRASONIC_ERROR: Sensor {i} - no signal returned, too far."
    signal_on = utime.ticks_us()

    # Calculate cm distance
    distance_in_cm = utime.ticks_diff(signal_on, signal_off) \
                     * (speed_of_sound / 20000.0)
    return distance_in_cm, None


def flip_bitmap_vert(bitmap, width, height):
    """
    Flip a byte array bitmap vertically.

    :param bitmap: byte array of image
    :param width: pixel width of the image (must be divisible by 8)
    :param height: pixel height of the image
    :return: vertically flipped bytearray (top for bottom)
    """
    # Each row has (width // 8), 8 pixels = 1 byte
    row_size = width // 8
    flipped_bitmap = bytearray(len(bitmap))

    # reverse the order of each row
    for y in range(height):
        # Copy each row from the original image
        start_index = y * row_size
        end_index = start_index + row_size
        row_data = bitmap[start_index:end_index]

        # Place it in the flipped position in the new byte array
        flipped_start = (height - y - 1) * row_size
        flipped_bitmap[flipped_start:flipped_start + row_size] = row_data
    return flipped_bitmap


def blit_white_only(source_fb, w, h, x, y):
    """
    only send white pixels one by one
    :param source_fb: framebuffer to flip
    :param w: width of the framebuffer
    :param h: height of the framebuffer
    :param x: x-position for display
    :param y: y-position for display
    """
    for row in range(h):
        for col in range(w):
            # Extract pixel from the source frame buffer
            pixel = source_fb.pixel(col, row)
            if pixel == 1:  # Only copy white pixels
                oled.pixel(x + col, y + row, 1)  # Set the pixel on the OLED


def display_car(celsius, fahrenheit):
    """
    display_car & temp, Need oled.fill(0) before call & oled.show() after call

    :param celsius: temp in C to display
    :param fahrenheit: temp in F to display
    """
    if rear:
        oled.blit(FrameBuffer(bitmap_artcar_image_back, 56, 15, MONO_HLSB), 36, 0)
        oled.blit(FrameBuffer(degree_temp, 24, 10, MONO_HLSB), 104, 0)

        if metric:
            oled.blit(FrameBuffer(bitmap_unit_cm, 24, 10, MONO_HLSB), 0, 0)
            if celsius:
                oled.text(f"{celsius:.0f}", 108, 2)
            else:
                oled.text("xx", 108, 2)

        else:
            oled.blit(FrameBuffer(bitmap_unit_in, 24, 10, MONO_HLSB), 0, 0)
            if fahrenheit:
                oled.text(f"{fahrenheit:.0f}", 108, 2)
            else:
                oled.text("xx", 108, 2)
    else:
        oled.blit(FrameBuffer(bitmap_artcar_image_front, 56, 15, MONO_HLSB), 36, DISP_HEIGHT - 15)
        oled.blit(FrameBuffer(degree_temp, 24, 10, MONO_HLSB), 104, DISP_HEIGHT - 10)

        if metric:
            oled.blit(FrameBuffer(bitmap_unit_cm, 24, 10, MONO_HLSB), 0, DISP_HEIGHT - 10)
            if celsius:
                oled.text(f"{celsius:.0f}", 108, DISP_HEIGHT - 8)
            else:
                oled.text("xx", 108, DISP_HEIGHT - 8)
        else:
            oled.blit(FrameBuffer(bitmap_unit_in, 24, 10, MONO_HLSB), 0, DISP_HEIGHT - 10)
            if fahrenheit:
                oled.text(f"{fahrenheit:.0f}", 108, DISP_HEIGHT - 8)
            else:
                oled.text("xx", 108, DISP_HEIGHT - 8)
    return


def display_tiles_dist():
    """
    display_car & temp, Need oled.show() after call
    """
    if not working_ultrasonics:
        oled.text(" No Ultrasonic", 5, 30)
        oled.text("Sensors Working", 5, 40)
        return

    if rear:
        # Middle
        if 1 in working_ultrasonics:
            # Display bitmap for sensor 1, since no pixels overlap we can use blit directly
            if sensor[1].cm > dist_step_1:
                oled.blit(FrameBuffer(bitmap_sensor_1a_on, 32, 9, MONO_HLSB), 48, 23)
            else:
                oled.blit(FrameBuffer(bitmap_sensor_1a_off, 32, 9, MONO_HLSB), 48, 23)
            if sensor[1].cm > dist_step_2:
                oled.blit(FrameBuffer(bitmap_sensor_1b_on, 32, 9, MONO_HLSB), 48, 33)
            else:
                oled.blit(FrameBuffer(bitmap_sensor_1b_off, 32, 9, MONO_HLSB), 48, 33)
            if sensor[1].cm > dist_step_3:
                oled.blit(FrameBuffer(bitmap_sensor_1c_on, 32, 10, MONO_HLSB), 47, 42)
            else:
                oled.blit(FrameBuffer(bitmap_sensor_1c_off, 32, 10, MONO_HLSB), 47, 42)
            if sensor[1].cm > dist_step_4:
                oled.blit(FrameBuffer(bitmap_sensor_1d_on, 40, 10, MONO_HLSB), 42, 52)
            else:
                oled.blit(FrameBuffer(bitmap_sensor_1d_off, 40, 10, MONO_HLSB), 42, 52)

        # left sensor
        if 0 in working_ultrasonics:
            # Display bitmap for sensor 0
            if sensor[0].cm > dist_step_1:
                bmp_1a = FrameBuffer(bitmap_sensor_0a_on, 32, 14, MONO_HLSB)  # , 24, 17)
            else:
                bmp_1a = FrameBuffer(bitmap_sensor_0a_off, 32, 14, MONO_HLSB)  # , 24, 17)
            if sensor[0].cm > dist_step_2:
                bmp_1b = FrameBuffer(bitmap_sensor_0b_on, 32, 16, MONO_HLSB)  # , 21, 25)
            else:
                bmp_1b = FrameBuffer(bitmap_sensor_0b_off, 32, 16, MONO_HLSB)  # , 21, 25)
            if sensor[0].cm > dist_step_3:
                bmp_1c = FrameBuffer(bitmap_sensor_0c_on, 32, 17, MONO_HLSB)  # , 18, 34)
            else:
                bmp_1c = FrameBuffer(bitmap_sensor_0c_off, 32, 17, MONO_HLSB)  # , 18, 34)
            if sensor[0].cm > dist_step_4:
                bmp_1d = FrameBuffer(bitmap_sensor_0d_on, 32, 18, MONO_HLSB)  # , 16, 43)
            else:
                bmp_1d = FrameBuffer(bitmap_sensor_0d_off, 32, 18, MONO_HLSB)  # , 16, 43)
            blit_white_only(bmp_1a, 32, 14, 24, 17)
            blit_white_only(bmp_1b, 32, 16, 21, 25)
            blit_white_only(bmp_1c, 32, 17, 18, 34)
            blit_white_only(bmp_1d, 32, 18, 16, 43)

        # right sensor
        if 2 in working_ultrasonics:
            # Display bitmap for sensor 2
            if sensor[2].cm > dist_step_1:
                bmp_3a = FrameBuffer(bitmap_sensor_2a_on, 32, 14, MONO_HLSB)  # , 72, 17)
            else:
                bmp_3a = FrameBuffer(bitmap_sensor_2a_off, 32, 14, MONO_HLSB)  # , 72, 17)
            if sensor[2].cm > dist_step_2:
                bmp_3b = FrameBuffer(bitmap_sensor_2b_on, 32, 16, MONO_HLSB)  # , 74, 25)
            else:
                bmp_3b = FrameBuffer(bitmap_sensor_2b_off, 32, 16, MONO_HLSB)  # , 74, 25)
            if sensor[2].cm > dist_step_3:
                bmp_3c = FrameBuffer(bitmap_sensor_2c_on, 32, 17, MONO_HLSB)  # , 77, 34)
            else:
                bmp_3c = FrameBuffer(bitmap_sensor_2c_off, 32, 17, MONO_HLSB)  # , 77, 34)
            if sensor[2].cm > dist_step_4:
                bmp_3d = FrameBuffer(bitmap_sensor_2d_on, 32, 18, MONO_HLSB)  # , 80, 43)
            else:
                bmp_3d = FrameBuffer(bitmap_sensor_2d_off, 32, 18, MONO_HLSB)  # , 80, 43)
            blit_white_only(bmp_3a, 32, 14, 72, 17)
            blit_white_only(bmp_3b, 32, 16, 74, 25)
            blit_white_only(bmp_3c, 32, 17, 77, 34)
            blit_white_only(bmp_3d, 32, 18, 80, 43)

    # This is for flipped displays
    else:
        # middle sensor
        if 1 in working_ultrasonics:
            # Display bitmap for sensor 1, since not pixels overlap, we can blit directly
            if sensor[1].cm > dist_step_1:
                oled.blit(FrameBuffer(flipped_bitmap_sensor_1a_on, 32, 9, MONO_HLSB), 48, DISP_HEIGHT - 23 - 9)
            else:
                oled.blit(FrameBuffer(flipped_bitmap_sensor_1a_off, 32, 9, MONO_HLSB), 48, DISP_HEIGHT - 23 - 9)
            if sensor[1].cm > dist_step_2:
                oled.blit(FrameBuffer(flipped_bitmap_sensor_1b_on, 32, 9, MONO_HLSB), 48, DISP_HEIGHT - 33 - 9)
            else:
                oled.blit(FrameBuffer(flipped_bitmap_sensor_1b_off, 32, 9, MONO_HLSB), 48, DISP_HEIGHT - 33 - 9)
            if sensor[1].cm > dist_step_3:
                oled.blit(FrameBuffer(flipped_bitmap_sensor_1c_on, 32, 10, MONO_HLSB), 47, DISP_HEIGHT - 42 - 10)
            else:
                oled.blit(FrameBuffer(flipped_bitmap_sensor_1c_off, 32, 10, MONO_HLSB), 47, DISP_HEIGHT - 42 - 10)
            if sensor[1].cm > dist_step_4:
                oled.blit(FrameBuffer(flipped_bitmap_sensor_1d_on, 40, 10, MONO_HLSB), 42, DISP_HEIGHT - 52 - 10)
            else:
                oled.blit(FrameBuffer(flipped_bitmap_sensor_1d_off, 40, 10, MONO_HLSB), 42, DISP_HEIGHT - 52 - 10)

        # left sensor
        if 0 in working_ultrasonics:
            # Display bitmap for sensor 2
            if sensor[0].cm > dist_step_1:
                bmp_3a = FrameBuffer(flipped_bitmap_sensor_2a_on, 32, 14, MONO_HLSB)
            else:
                bmp_3a = FrameBuffer(flipped_bitmap_sensor_2a_off, 32, 14, MONO_HLSB)
            if sensor[0].cm > dist_step_2:
                bmp_3b = FrameBuffer(flipped_bitmap_sensor_2b_on, 32, 16, MONO_HLSB)
            else:
                bmp_3b = FrameBuffer(flipped_bitmap_sensor_2b_off, 32, 16, MONO_HLSB)
            if sensor[0].cm > dist_step_3:
                bmp_3c = FrameBuffer(flipped_bitmap_sensor_2c_on, 32, 17, MONO_HLSB)
            else:
                bmp_3c = FrameBuffer(flipped_bitmap_sensor_2c_off, 32, 17, MONO_HLSB)
            if sensor[0].cm > dist_step_4:
                bmp_3d = FrameBuffer(flipped_bitmap_sensor_2d_on, 32, 18, MONO_HLSB)
            else:
                bmp_3d = FrameBuffer(flipped_bitmap_sensor_2d_off, 32, 18, MONO_HLSB)
            blit_white_only(bmp_3a, 32, 14, 24, DISP_HEIGHT - 17 - 14)
            blit_white_only(bmp_3b, 32, 16, 21, DISP_HEIGHT - 25 - 16)
            blit_white_only(bmp_3c, 32, 17, 18, DISP_HEIGHT - 34 - 17)
            blit_white_only(bmp_3d, 32, 18, 16, DISP_HEIGHT - 43 - 18)

        # right sensor
        if 2 in working_ultrasonics:
            # Display bitmap for sensor 0
            if sensor[2].cm > dist_step_1:
                bmp_1a = FrameBuffer(flipped_bitmap_sensor_0a_on, 32, 14, MONO_HLSB)
            else:
                bmp_1a = FrameBuffer(flipped_bitmap_sensor_0a_off, 32, 14, MONO_HLSB)
            if sensor[2].cm > dist_step_2:
                bmp_1b = FrameBuffer(flipped_bitmap_sensor_0b_on, 32, 16, MONO_HLSB)
            else:
                bmp_1b = FrameBuffer(flipped_bitmap_sensor_0b_off, 32, 16, MONO_HLSB)
            if sensor[2].cm > dist_step_3:
                bmp_1c = FrameBuffer(flipped_bitmap_sensor_0c_on, 32, 17, MONO_HLSB)
            else:
                bmp_1c = FrameBuffer(flipped_bitmap_sensor_0c_off, 32, 17, MONO_HLSB)
            if sensor[2].cm > dist_step_4:
                bmp_1d = FrameBuffer(flipped_bitmap_sensor_0d_on, 32, 18, MONO_HLSB)
            else:
                bmp_1d = FrameBuffer(flipped_bitmap_sensor_0d_off, 32, 18, MONO_HLSB)
            blit_white_only(bmp_1a, 32, 14, 72, DISP_HEIGHT - 17 - 14)
            blit_white_only(bmp_1b, 32, 16, 74, DISP_HEIGHT - 25 - 16)
            blit_white_only(bmp_1c, 32, 17, 77, DISP_HEIGHT - 34 - 17)
            blit_white_only(bmp_1d, 32, 18, 80, DISP_HEIGHT - 43 - 18)



    # round to nearest
    for j in working_ultrasonics:
        if metric:
            int_string = str(int(sensor[j].cm + 0.5))
        else:
            int_string = str(int(sensor[j].inch + 0.5))
        digits = get_str_width(int_string)

        startpos_x = sensor[j].label_startpos_x + (3 - digits) * 4
        endpos_x = sensor[j].label_endpos_x + (3 - digits) * 4

        # Display distance label
        xpos = int(map_range(constrain(sensor[j].cm, MIN_CM_DIST, MAX_CM_DIST),
                             MIN_CM_DIST, MAX_CM_DIST, startpos_x, endpos_x))
        ypos = int(map_range(constrain(sensor[j].cm, MIN_CM_DIST, MAX_CM_DIST),
                             MIN_CM_DIST, MAX_CM_DIST, sensor[j].label_startpos_y, sensor[j].label_endpos_y))

        # print black box slightly larger than digits, then print digits
        if rear:
            oled.fill_rect(xpos, ypos - 1, 8 * digits, 9, 0)
            oled.text(int_string, xpos, ypos)
        else:
            oled.fill_rect(xpos, DISP_HEIGHT - ypos - 1 - 8, 8 * digits, 9, 0)
            oled.text(int_string, xpos, DISP_HEIGHT - ypos - 8)
    return


def display_environment_details(dist, buzz):
    """
    display just environment readings & car image(for fun)
    No need to oled.fill(0) before or oled.show() after call

    param:dist:  distance if dist = -1.0 then display error
    param:buzz:  distance if dist = -1.0 then display error
    """
    oled.fill(0)
    if dist < 12.0 and buzz:
        buzzer.on()
    elif buzz:
        buzzer.off()
    if temp_c:
        if metric:
            oled.text(f"Temp = {temp_c:.1f}C", 0, 0)
        else:
            oled.text(f"Temp = {temp_f:.1f}F", 0, 0)
        if humidity: oled.text(f"Humid= {humidity:.1f}%", 0, 12)
    else:
        oled.text(f"No Temp/Humidity", 0, 10)
        oled.text(f" Sensor Working", 0, 20)

    if metric:
        oled.text(f"Sound={speed_sound:.1f}m/s", 0, 24)
        if dist:
            oled.text(f"Dist= {dist:.0f}cm", 0, 55)
        else:
            oled.text(f"No ultrasonic", 0, 55)
    else:
        oled.text(f"Sound={speed_sound * 3.28084:.0f}ft/s", 0, 24)
        if dist:
            oled.text(f"Dist= {dist / 2.54:.1f}in", 0, 55)
        else:
            oled.text(f"No ultrasonic", 0, 55)

    oled.blit(FrameBuffer(bitmap_artcar_image_back, 56, 15, MONO_HLSB), 22, 36)
    oled.show()
    return


# =========================  startup code =========================
rear = True
show_env = False
buzzer_sound = True
metric = False
dht_error = False
ds_error = False
debug = False


dist_step_1 = MIN_CM_DIST + round((MAX_CM_DIST - MIN_CM_DIST) / 4.0 * 1.0)  # tile1 step
dist_step_2 = MIN_CM_DIST + round((MAX_CM_DIST - MIN_CM_DIST) / 4.0 * 2.0)  # tile2 step
dist_step_3 = MIN_CM_DIST + round((MAX_CM_DIST - MIN_CM_DIST) / 4.0 * 3.0)  # tile3 step
dist_step_4 = MIN_CM_DIST + round((MAX_CM_DIST - MIN_CM_DIST) / 4.0 * 4.0)  # tile4 step

# numeric label position, to print distance # over tiles
sensor[0].label_startpos_x = 30  # was 41 (-11)
sensor[0].label_startpos_y = 20
sensor[0].label_endpos_x = 18  # was 30, adjust 1 more for symmetry
sensor[0].label_endpos_y = 56  # was 58
sensor[1].label_startpos_x = 52  # was 63 (-11) 52px 3 dig, 56 2 dig, 60 for 1 digit
sensor[1].label_startpos_y = 23
sensor[1].label_endpos_x = 52  # was 63 (-11) 52px 3 dig, 56 2 dig, 60 for 1 digit
sensor[1].label_endpos_y = 56  # was 60
sensor[2].label_startpos_x = 74  # was 85 (-11)
sensor[2].label_startpos_y = 20
sensor[2].label_endpos_x = 86  # was 97 (-11)
sensor[2].label_endpos_y = 56  # was 57

button_1_pushed = False
button_2_pushed = False
button_3_pushed = False
debounce_1_time = 0
debounce_2_time = 0
debounce_3_time = 0

speed_sound = SPEED_SOUND_20C_70H
sea_level_pressure_hpa = PDX_SLP_1013
sea_level_pressure_hpa = 1010.70
temp_f = None
temp_c = None
humidity = None

print("Starting...")
print("====================================")
print(implementation[0], uname()[3],
      "\nrun on", uname()[4])
temp = onboard_temperature()
print(f"onboard Pico 2 temp = {temp:.1f}C")
print("====================================")
print(f"oled_spi:{oled_spi}")
print(f"{sea_level_pressure_hpa=}")

# roms will be a list of sensors on same GPIO pin
roms = ds_sensor.scan()
print('Found DS devices, roms: ', roms)

# call first time to set up onewire temp, we don't measure temp in this routine
error = outside_temp_ds_init()
if error:
    print(error)
else:
    print("Onewire outside temp found")

# # check status of DHT sensor, do not keep any measurements
# _, _, error = dht_temp_humidity()
# if error:
#     oled.text("Error DHT22", 0, 24)
#     oled.text(str(error), 0, 36)
#     oled.show()
#     temp_c = None
#     humidity = None

# at start display artcar image at top of oled
oled.fill(0)
display_car(None, None)
oled.show()

working_ultrasonics = []
nonworking_ultrasonics = []
for i in range(NUMBER_OF_SENSORS):
    distance, error = ultrasonic_distance_pwm(i, speed_sound, timeout=50000)
    if error:
        nonworking_ultrasonics.append(i)
    else:
        working_ultrasonics.append(i)
print(f"Working PWM Ultrasonic sensors: {working_ultrasonics}")
print(f"Non-working PWM Ultrasonic sensors: {nonworking_ultrasonics}")
print(f"Default Speed Sound: {speed_sound:.1f} m/s")

initialize_flipped_bitmaps()

if buzzer_sound: buzzer.on()
zzz(.2)
buzzer.off()

print("start of main loop\n")
# main loop
first_run = True
time_since_last_temp_update = time.ticks_ms()

try:
    while True:
        loop_time = time.ticks_ms()
        elapsed_time = time.ticks_diff(time.ticks_ms(), time_since_last_temp_update)
        if debug: print(f"Time since last temp ={elapsed_time}")
            
        # Button 1: cm/in
        if button1():
            metric = not metric  # Toggle between metric and imperial units

        # Button 2: Adjust altitude or sea level hpa to known values
        if button2():
            rear = not rear  # Toggle between rear/front
            print(f"{rear=}")
        
        # Button 3: show details of environment
        if button3():
            show_env = not show_env  # Toggle between rear/front
            print(f"{show_env=}")
            
        # EVERY 3 SECONDS, calc speed of sound based on
        # * onewire outside temp (16ms duration)
        # * dht22 temp & humidity (271ms duration)
        # Note: Temp & humidity correction is for the speed of sound
        #  - going from 0C to 30C goes from 331.48 m/s to 351.24 m/s (~ 6%)
        #  - at 30C, humidity 0% to 90% goes from 349.4m/s to 351.2m/s (~ 0.53%)
        # ...humidity effect is negligible, but I had a DHT22, so why not :)
        if first_run or elapsed_time > 3000:
            time_since_last_temp_update = time.ticks_ms()

            # check for over temperature onboard pico
            temp = onboard_temperature()
            if temp > OVER_TEMP_WARNING:
                print(f"WARNING: onboard Pico 2 temp = {temp:.1f}C")
            
            start = time.ticks_ms()
            # Outside temp from waterproof ds sensor
            outside_temp_c, error = outside_temp_ds()
            if error:
                print(f"No Outside Temp: {error}")
            else:
                if debug: print(f"Outside temp C = {outside_temp_c:.2f}")
            elapsed_time = time.ticks_diff(time.ticks_ms(), start)
            print(f"outside temp time = {elapsed_time}")
            
            start = time.ticks_ms()
            # Inside Temp & humidity dht sensor
#             inside_temp_c, humidity, error = dht_temp_humidity()
            inside_temp_c, humidity, pressure_hpa, iaq, altitude_m, _ = bme_temp_humid_hpa_iaq_alt(sea_level_pressure_hpa)        
            if error:
                print(f"No Inside Temp: {error}")
            else:
                if debug: print(f"Inside temp C = {inside_temp_c:.2f}")

            elapsed_time = time.ticks_diff(time.ticks_ms(), start)
            print(f"time inside temp = {elapsed_time}ms\n")

            # if no outside temp use inside temp
            # if no inside temp/humidity use 70%
            # if no temps use default speed of sound
            if outside_temp_c and inside_temp_c:
                temp_c = outside_temp_c
                speed_sound, _ = calc_speed_sound(temp_c, humidity)
            elif outside_temp_c:
                temp_c = outside_temp_c
                speed_sound, _ = calc_speed_sound(temp_c, 70.0)
                humidity = None
            elif inside_temp_c:
                temp_c = inside_temp_c
                speed_sound, _ = calc_speed_sound(temp_c, humidity)
            else:
                speed_sound = SPEED_SOUND_20C_70H
            if temp_c:
                temp_f = (temp_c * 9.0 / 5.0) + 32.0
            first_run = False

            if debug and outside_temp_c: print(f"outside={outside_temp_c:.3f}")
            if debug and inside_temp_c:
                print(f"inside={inside_temp_c:.3f}, humidity={humidity:.1f}, speed_sound={speed_sound:.1f}")
            if debug: print(f"temp_c={temp_c:.3f}\n")

        # get distance from pwm ultrasonic sensor, 30ms round trip maz ia 514cm or 202in
        # sensors: left front/rear = 0, middle=1  right front/rear =2
        for i in working_ultrasonics:
            sensor[i].cm, error = ultrasonic_distance_pwm(i, speed_sound, timeout=30000)
            if error:
                print(error)
            else:
                sensor[i].inch = sensor[i].cm / 2.54

        if show_env:
            # pick first working ultrasonic to show
            display_environment_details(sensor[working_ultrasonics[0]].cm if working_ultrasonics else -1.0, buzzer_sound)
        else:
            oled.fill(0)
            display_car(temp_c, temp_f)
            display_tiles_dist()
#             oled.text(f"6912\'", 0, 12)
            oled.show()

        # Every loop do this
        led.toggle()
        time.sleep_ms(DWELL_MS_LOOP)
        loop_elapsed_time = time.ticks_diff(time.ticks_ms(), loop_time)
#         print(f"loop time with {DWELL_MS_LOOP}ms delay={loop_elapsed_time}")

# if control-c at end clean up pico2
except KeyboardInterrupt:
    oled.fill(0)  # turn off oled
    oled.show()  # show on oled
    print("Exit: ctrl-c")
# except:
#     print ("Other error or exception occurred!")

