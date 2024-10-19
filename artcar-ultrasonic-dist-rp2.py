# Raspberry Pi Pico 2 project - ArtCar ultrasonic - Oct 2024
#
# BUT right now only 1 ultrasonic, oled SPI display: temp,humid,speedsound, & dist
#
# based on Brad's Arduino UNO and 128x64 OLED Display for rear&front parking sensor
#    - speed of sound with temp & humidity correction
#    - front & Rear facing changed with button
#      - on Raspberry Pi recalc offsets, and flip sensor graphics
#      - on arduino could rotate display and flip,text,some graphics(not sensor)
#    - in/cm F/C changed with button
#    - buttons are all rp2 interrupt driven for debounce (nice!)
#
#    Raspberry Pi GitHub: https://github.com/bradcar/artcar-ultrasonic-dist-rp2
#    Arduino GitHub:      https://github.com/bradcar/art-car-ultrasonic-dist
#
# project Based on GREAT work by upir!!!
#     youtube full video: https://youtu.be/gg08H-6Z1Lo
#     created by upir, 2022
#
# https://micropython.org/download/RPI_PICO2/ for latest .uf2 preview
# https://docs.micropython.org/en/latest/esp8266/tutorial/ssd1306.html

from machine import Pin, Timer
import machine
import dht
import time
import utime
from math import sqrt
from os import uname
from sys import implementation
from time import sleep as zzz

#ic2
#from machine import I2C
#from ssd1306 import SSD1306_I2C
from ssd1306 import SSD1306_SPI
from framebuf import FrameBuffer, MONO_HLSB

# Constants and setup
DISP_WIDTH=128
DISP_HEIGHT=64
# Define the number of sensors
NUMBER_OF_SENSORS = 3

class SensorData:
    def __init__(self, echo_pin, trig_pin):
        self.trig_pin = trig_pin    # TRIG pin for the ultrasonic distance 
        self.echo_pin = echo_pin    # ECHO pin for the ultrasonic distance 
        self.measured_ms = 0        # measured milliseconds
        self.cm = 0.0               # measured distance in CM
        self.inch = 0.0             # measured distance in inches
        self.label_xpos = 0         # x position of the distance label
        self.label_ypos = 0         # y position of the distance label
        self.label_width = 0        # calculated width of the distance string
        self.label_startpos_x = 0   # start X position for the label
        self.label_startpos_y = 0   # start Y position for the label
        self.label_endpos_x = 0     # end X position for the label
        self.label_endpos_y = 0     # end Y position for the label

MIN_DIST = 2.0
MAX_DIST = 100.0
SPEED_SOUND_20C_70H = 343.294

dist_step_01 = MIN_DIST + round((MAX_DIST - MIN_DIST) / 4.0 * 1.0)
dist_step_02 = MIN_DIST + round((MAX_DIST - MIN_DIST) / 4.0 * 2.0)
dist_step_03 = MIN_DIST + round((MAX_DIST - MIN_DIST) / 4.0 * 3.0)
dist_step_04 = MIN_DIST + round((MAX_DIST - MIN_DIST) / 4.0 * 4.0)

interrupt_1_flag=0
interrupt_2_flag=0
debounce_1_time=0
debounce_2_time=0

# === PINS ===
dht_pin = machine.Pin(2)
button_1 = Pin(5, Pin.IN, Pin.PULL_UP)  #interrupt cm/in button pins
button_2 = Pin(6, Pin.IN, Pin.PULL_UP)  #interrupt rear/front button pins

# https://www.tomshardware.com/how-to/oled-display-raspberry-pi-pico
# i2c=I2C(0,sda=Pin(0), scl=Pin(1), freq=400000)
# oled = SSD1306_I2C(DISP_WIDTH, DISP_HEIGHT, i2c)
# https://coxxect.blogspot.com/2024/10/multi-ssd1306-oled-on-raspberry-pi-pico.html
cs  = machine.Pin(9)    #dummy (any un-used pin), no connection
# scl (SCLK) gp10 
# SDA (MOSI) gp11
res = machine.Pin(12)   # RES (RST)  gp12
dc  = machine.Pin(13)   # DC         gp13

# Create a list to hold the sensor data for each sensor
#set up for (21,20, 19,18, 17,16)
sensor = [SensorData(trig_pin=21, echo_pin=20),
          SensorData(trig_pin=3, echo_pin=4),
          SensorData(trig_pin=17, echo_pin=16)]
# sensor = [SensorData(trig_pin=21, echo_pin=20),
#           SensorData(trig_pin=19, echo_pin=18),
#           SensorData(trig_pin=17, echo_pin=16)]

# Initialize empty lists for trigger and echo
trigger = []
echo = []

for i in range(NUMBER_OF_SENSORS):
    trigger_pin = Pin(sensor[i].trig_pin, Pin.OUT)
    echo_pin = Pin(sensor[i].echo_pin, Pin.IN)
    trigger.append(trigger_pin)
    echo.append(echo_pin)
led = Pin(25, Pin.OUT)

dht_sensor = dht.DHT22(dht_pin)
oled_spi = machine.SPI(1)
oled = SSD1306_SPI(DISP_WIDTH, DISP_HEIGHT, oled_spi, dc, res, cs)
print("oled_spi:", oled_spi)

#initialize label position data were manually defined in the Photoshop
sensor[0].label_startpos_x = 30  # was 41 (-11)
sensor[0].label_startpos_y = 20
sensor[0].label_endpos_x = 18    # was 30, adjust 1 more for symmetry
sensor[0].label_endpos_y = 56    # was 58
sensor[1].label_startpos_x = 52  # was 63 (-11) 52px 3 dig, 56 2 dig, 60 for 1 digit
sensor[1].label_startpos_y = 23
sensor[1].label_endpos_x = 52    # was 63 (-11) 52px 3 dig, 56 2 dig, 60 for 1 digit
sensor[1].label_endpos_y = 56    # was 60
sensor[2].label_startpos_x = 74  # was 85 (-11)
sensor[2].label_startpos_y = 20
sensor[2].label_endpos_x = 86    # was 97 (-11)
sensor[2].label_endpos_y = 56    # was 57

# DISPLAY IMAGES
# image2cpp (convert png into C code): https://javl.github.io/image2cpp/
# const unsigned char bitmap_artcar_image[] PROGMEM = {0xc9, 0x04, 0x52, ...
# can be bitmap_artcar_image=bytearray(b'\xc9\x04\x59
#
# 'art-car-imag', 56x15px
bitmap_artcar_image_back = bytearray([
  0xc9, 0x04, 0x52, 0x91, 0x0c, 0x08, 0x43, 0xc8, 0x82, 0x8e, 0x90, 0x93, 0x10, 0x93, 0xe0, 0x63, 
  0x08, 0x78, 0xe0, 0xe3, 0x07, 0xff, 0x9f, 0xff, 0xff, 0xff, 0xfc, 0xff, 0xc0, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x03, 0x40, 0x1c, 0xf3, 0xe3, 0x8e, 0x78, 0x02, 0x42, 0x22, 0x88, 0x84, 0x51, 0x44, 
  0x42, 0x42, 0x22, 0x88, 0x84, 0x11, 0x44, 0x42, 0x42, 0x22, 0xf0, 0x84, 0x11, 0x78, 0x42, 0x22, 
  0x3e, 0x88, 0x84, 0x1f, 0x44, 0x44, 0x10, 0x22, 0x88, 0x84, 0x51, 0x44, 0x08, 0x0c, 0x22, 0x88, 
  0x83, 0x91, 0x44, 0x30, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
])
# for front sensor
# 'art-car-image-flip', 56x15px
bitmap_artcar_image_front = bytearray([
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x03, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0xc0, 0x0c, 0x1c, 0xf3, 0xe3, 0x8e, 0x78, 0x30, 0x10, 0x22, 0x88, 0x84, 
  0x51, 0x44, 0x08, 0x20, 0x22, 0x88, 0x84, 0x11, 0x44, 0x04, 0x4f, 0x22, 0xf0, 0x84, 0x11, 0x78, 
  0xf2, 0x4f, 0x3e, 0x88, 0x84, 0x1f, 0x44, 0xf2, 0x4f, 0x22, 0x88, 0x84, 0x51, 0x44, 0xf2, 0x40, 
  0x22, 0x88, 0x83, 0x91, 0x44, 0x02, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0x9f, 0xff, 
  0xff, 0xff, 0xf9, 0xff, 0xe0, 0x63, 0x08, 0x78, 0xe0, 0xe7, 0x07, 0xc8, 0x82, 0x8e, 0x90, 0x93, 
  0x10, 0x93, 0xc9, 0x04, 0x52, 0x91, 0x0c, 0x08, 0x43
])
# 'sensor_1a_off', 32x14px
bitmap_sensor_1a_off = bytearray([
  0x00, 0x40, 0x00, 0x00, 0x02, 0xa8, 0x00, 0x00, 0x05, 0x55, 0x00, 0x00, 0x02, 0xaa, 0xa0, 0x00,
  0x05, 0x55, 0x54, 0x00, 0x0a, 0xaa, 0xaa, 0xa0, 0x15, 0x55, 0x55, 0x50, 0x0a, 0xaa, 0xaa, 0xa0,
  0x01, 0x55, 0x55, 0x40, 0x00, 0x2a, 0xaa, 0xa0, 0x00, 0x15, 0x55, 0x40, 0x00, 0x02, 0xaa, 0xa0,
  0x00, 0x00, 0x15, 0x40, 0x00, 0x00, 0x02, 0xa0
])
# 'sensor_1a_on', 32x14px
bitmap_sensor_1a_on = bytearray([
  0x00, 0xc0, 0x00, 0x00, 0x03, 0xf8, 0x00, 0x00, 0x07, 0xff, 0x00, 0x00, 0x07, 0xff, 0xe0, 0x00,
  0x0f, 0xff, 0xfe, 0x00, 0x0f, 0xff, 0xff, 0xf0, 0x1f, 0xff, 0xff, 0xf0, 0x0f, 0xff, 0xff, 0xe0,
  0x03, 0xff, 0xff, 0xe0, 0x00, 0x7f, 0xff, 0xe0, 0x00, 0x1f, 0xff, 0xe0, 0x00, 0x03, 0xff, 0xe0,
  0x00, 0x00, 0x3f, 0xe0, 0x00, 0x00, 0x03, 0xe0
])
# 'sensor_1b_off', 32x16px
bitmap_sensor_1b_off = bytearray([
  0x02, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x02, 0xa0, 0x00, 0x00, 0x05, 0x54, 0x00, 0x00,
  0x0a, 0xaa, 0x80, 0x00, 0x15, 0x55, 0x50, 0x00, 0x0a, 0xaa, 0xaa, 0x80, 0x15, 0x55, 0x55, 0x50,
  0x0a, 0xaa, 0xaa, 0xa8, 0x05, 0x55, 0x55, 0x50, 0x00, 0xaa, 0xaa, 0xa8, 0x00, 0x15, 0x55, 0x50,
  0x00, 0x02, 0xaa, 0xa0, 0x00, 0x00, 0x55, 0x50, 0x00, 0x00, 0x0a, 0xa0, 0x00, 0x00, 0x00, 0x50
])
# 'sensor_1b_on', 32x16px
bitmap_sensor_1b_on = bytearray([
  0x02, 0x00, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 0x07, 0xf0, 0x00, 0x00, 0x0f, 0xfc, 0x00, 0x00,
  0x0f, 0xff, 0x80, 0x00, 0x1f, 0xff, 0xf8, 0x00, 0x1f, 0xff, 0xff, 0x80, 0x3f, 0xff, 0xff, 0xf8,
  0x1f, 0xff, 0xff, 0xf8, 0x07, 0xff, 0xff, 0xf8, 0x01, 0xff, 0xff, 0xf8, 0x00, 0x3f, 0xff, 0xf8,
  0x00, 0x07, 0xff, 0xf0, 0x00, 0x00, 0xff, 0xf0, 0x00, 0x00, 0x0f, 0xf0, 0x00, 0x00, 0x00, 0xf0
])
# 'sensor_1c_off', 32x17px
bitmap_sensor_1c_off = bytearray([
  0x08, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x0a, 0x80, 0x00, 0x00, 0x15, 0x50, 0x00, 0x00,
  0x2a, 0xaa, 0x00, 0x00, 0x15, 0x55, 0x40, 0x00, 0x2a, 0xaa, 0xaa, 0x00, 0x55, 0x55, 0x55, 0x40,
  0x2a, 0xaa, 0xaa, 0xaa, 0x15, 0x55, 0x55, 0x54, 0x02, 0xaa, 0xaa, 0xa8, 0x00, 0x55, 0x55, 0x54,
  0x00, 0x0a, 0xaa, 0xa8, 0x00, 0x01, 0x55, 0x54, 0x00, 0x00, 0x2a, 0xa8, 0x00, 0x00, 0x05, 0x54,
  0x00, 0x00, 0x00, 0x28
])
# 'sensor_1c_on', 32x17px
bitmap_sensor_1c_on = bytearray([
  0x08, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x1f, 0xc0, 0x00, 0x00, 0x1f, 0xf8, 0x00, 0x00,
  0x3f, 0xff, 0x00, 0x00, 0x3f, 0xff, 0xe0, 0x00, 0x7f, 0xff, 0xfe, 0x00, 0x7f, 0xff, 0xff, 0xe0,
  0x7f, 0xff, 0xff, 0xfe, 0x1f, 0xff, 0xff, 0xfe, 0x03, 0xff, 0xff, 0xfc, 0x00, 0xff, 0xff, 0xfc,
  0x00, 0x1f, 0xff, 0xfc, 0x00, 0x03, 0xff, 0xfc, 0x00, 0x00, 0x3f, 0xfc, 0x00, 0x00, 0x07, 0xfc,
  0x00, 0x00, 0x00, 0x3c
])
# 'sensor_1d_off', 32x18px
bitmap_sensor_1d_off = bytearray([
  0x00, 0x00, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, 0x55, 0x00, 0x00, 0x00, 0x2a, 0x80, 0x00, 0x00,
  0x55, 0x50, 0x00, 0x00, 0xaa, 0xaa, 0x00, 0x00, 0x55, 0x55, 0x50, 0x00, 0xaa, 0xaa, 0xaa, 0x00,
  0x55, 0x55, 0x55, 0x50, 0x2a, 0xaa, 0xaa, 0xaa, 0x05, 0x55, 0x55, 0x54, 0x02, 0xaa, 0xaa, 0xaa,
  0x00, 0x55, 0x55, 0x54, 0x00, 0x0a, 0xaa, 0xaa, 0x00, 0x01, 0x55, 0x54, 0x00, 0x00, 0x2a, 0xa8,
  0x00, 0x00, 0x01, 0x54, 0x00, 0x00, 0x00, 0x08
])
# 'sensor_2a_off', 32x9px
bitmap_sensor_2a_off = bytearray([
  0x05, 0x55, 0x55, 0x40, 0x0a, 0xaa, 0xaa, 0xa0, 0x05, 0x55, 0x55, 0x40, 0x0a, 0xaa, 0xaa, 0xa0,
  0x05, 0x55, 0x55, 0x40, 0x0a, 0xaa, 0xaa, 0xa0, 0x05, 0x55, 0x55, 0x40, 0x0a, 0xaa, 0xaa, 0xa0,
  0x15, 0x55, 0x55, 0x50
])
# 'sensor_1d_on', 32x18px
bitmap_sensor_1d_on = bytearray([
  0x20, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x7f, 0x00, 0x00, 0x00, 0x7f, 0xc0, 0x00, 0x00,
  0xff, 0xf8, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00, 0xff, 0xff, 0xf0, 0x00, 0xff, 0xff, 0xfe, 0x00,
  0x7f, 0xff, 0xff, 0xf0, 0x3f, 0xff, 0xff, 0xfe, 0x0f, 0xff, 0xff, 0xfe, 0x03, 0xff, 0xff, 0xfe,
  0x00, 0x7f, 0xff, 0xfe, 0x00, 0x1f, 0xff, 0xfe, 0x00, 0x01, 0xff, 0xfc, 0x00, 0x00, 0x3f, 0xfc,
  0x00, 0x00, 0x03, 0xfc, 0x00, 0x00, 0x00, 0x1c
])
# 'sensor_2a_on', 32x9px
bitmap_sensor_2a_on = bytearray([
  0x07, 0xff, 0xff, 0xc0, 0x0f, 0xff, 0xff, 0xe0, 0x0f, 0xff, 0xff, 0xe0, 0x0f, 0xff, 0xff, 0xe0,
  0x0f, 0xff, 0xff, 0xe0, 0x0f, 0xff, 0xff, 0xe0, 0x0f, 0xff, 0xff, 0xe0, 0x0f, 0xff, 0xff, 0xe0,
  0x1f, 0xff, 0xff, 0xf0
])
# 'sensor_2b_off', 32x9px
bitmap_sensor_2b_off = bytearray([
  0x15, 0x55, 0x55, 0x50, 0x0a, 0xaa, 0xaa, 0xa0, 0x15, 0x55, 0x55, 0x50, 0x0a, 0xaa, 0xaa, 0xa0,
  0x15, 0x55, 0x55, 0x50, 0x2a, 0xaa, 0xaa, 0xa8, 0x15, 0x55, 0x55, 0x50, 0x2a, 0xaa, 0xaa, 0xa8,
  0x05, 0x55, 0x55, 0x40
])
# 'sensor_2b_on', 32x9px
bitmap_sensor_2b_on = bytearray([
  0x1f, 0xff, 0xff, 0xf0, 0x1f, 0xff, 0xff, 0xf0, 0x1f, 0xff, 0xff, 0xf0, 0x1f, 0xff, 0xff, 0xf0,
  0x3f, 0xff, 0xff, 0xf8, 0x3f, 0xff, 0xff, 0xf8, 0x3f, 0xff, 0xff, 0xf8, 0x3f, 0xff, 0xff, 0xf8,
  0x0f, 0xff, 0xff, 0xe0
])
# 'sensor_2c_off', 32x10px
bitmap_sensor_2c_off = bytearray([
  0x10, 0x00, 0x00, 0x04, 0x0a, 0xaa, 0xaa, 0xa8, 0x15, 0x55, 0x55, 0x54, 0x2a, 0xaa, 0xaa, 0xaa,
  0x15, 0x55, 0x55, 0x54, 0x2a, 0xaa, 0xaa, 0xaa, 0x15, 0x55, 0x55, 0x54, 0x2a, 0xaa, 0xaa, 0xaa,
  0x15, 0x55, 0x55, 0x54, 0x0a, 0xaa, 0xaa, 0xa8
])
# 'sensor_2c_on', 32x10px
bitmap_sensor_2c_on = bytearray([
  0x18, 0x00, 0x00, 0x0c, 0x1f, 0xff, 0xff, 0xfc, 0x3f, 0xff, 0xff, 0xfe, 0x3f, 0xff, 0xff, 0xfe,
  0x3f, 0xff, 0xff, 0xfe, 0x3f, 0xff, 0xff, 0xfe, 0x3f, 0xff, 0xff, 0xfe, 0x3f, 0xff, 0xff, 0xfe,
  0x3f, 0xff, 0xff, 0xfe, 0x0f, 0xff, 0xff, 0xf8
])
# 'sensor_2d_off', 40x10px
bitmap_sensor_2d_off = bytearray([
  0x02, 0x80, 0x00, 0x00, 0x28, 0x01, 0x55, 0x55, 0x55, 0x50, 0x02, 0xaa, 0xaa, 0xaa, 0xa8, 0x01,
  0x55, 0x55, 0x55, 0x50, 0x02, 0xaa, 0xaa, 0xaa, 0xa8, 0x05, 0x55, 0x55, 0x55, 0x54, 0x02, 0xaa,
  0xaa, 0xaa, 0xa8, 0x05, 0x55, 0x55, 0x55, 0x54, 0x02, 0xaa, 0xaa, 0xaa, 0xa8, 0x00, 0x55, 0x55,
  0x55, 0x40
])
# 'sensor_2d_on', 40x10px
bitmap_sensor_2d_on = bytearray([
  0x03, 0x80, 0x00, 0x00, 0x38, 0x03, 0xff, 0xff, 0xff, 0xf8, 0x03, 0xff, 0xff, 0xff, 0xf8, 0x03,
  0xff, 0xff, 0xff, 0xf8, 0x03, 0xff, 0xff, 0xff, 0xf8, 0x07, 0xff, 0xff, 0xff, 0xfc, 0x07, 0xff,
  0xff, 0xff, 0xfc, 0x07, 0xff, 0xff, 0xff, 0xfc, 0x07, 0xff, 0xff, 0xff, 0xfc, 0x00, 0xff, 0xff,
  0xff, 0xe0
])
# 'sensor_3a_off', 32x14px
bitmap_sensor_3a_off = bytearray([
  0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x2a, 0x80, 0x00, 0x01, 0x55, 0x40, 0x00, 0x0a, 0xaa, 0x80,
  0x00, 0x55, 0x55, 0x40, 0x0a, 0xaa, 0xaa, 0xa0, 0x15, 0x55, 0x55, 0x50, 0x0a, 0xaa, 0xaa, 0xa0,
  0x05, 0x55, 0x55, 0x00, 0x0a, 0xaa, 0xa8, 0x00, 0x05, 0x55, 0x50, 0x00, 0x0a, 0xaa, 0x80, 0x00,
  0x05, 0x50, 0x00, 0x00, 0x0a, 0x80, 0x00, 0x00
])
# 'sensor_3b_on', 32x16px
bitmap_sensor_3b_on = bytearray([
  0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x01, 0xe0, 0x00, 0x00, 0x0f, 0xe0, 0x00, 0x00, 0x3f, 0xf0,
  0x00, 0x01, 0xff, 0xf0, 0x00, 0x1f, 0xff, 0xf8, 0x01, 0xff, 0xff, 0xf8, 0x1f, 0xff, 0xff, 0xfc,
  0x1f, 0xff, 0xff, 0xf8, 0x1f, 0xff, 0xff, 0xe0, 0x1f, 0xff, 0xff, 0x80, 0x1f, 0xff, 0xfc, 0x00,
  0x0f, 0xff, 0xe0, 0x00, 0x0f, 0xff, 0x00, 0x00, 0x0f, 0xf0, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00
])
# 'sensor_3a_on', 32x14px
bitmap_sensor_3a_on = bytearray([
  0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x3f, 0x80, 0x00, 0x01, 0xff, 0xc0, 0x00, 0x0f, 0xff, 0xc0,
  0x00, 0xff, 0xff, 0xe0, 0x1f, 0xff, 0xff, 0xe0, 0x1f, 0xff, 0xff, 0xf0, 0x0f, 0xff, 0xff, 0xe0,
  0x0f, 0xff, 0xff, 0x80, 0x0f, 0xff, 0xfc, 0x00, 0x0f, 0xff, 0xf0, 0x00, 0x0f, 0xff, 0x80, 0x00,
  0x0f, 0xf8, 0x00, 0x00, 0x0f, 0x80, 0x00, 0x00
])
# 'sensor_3b_off', 32x16px
bitmap_sensor_3b_off = bytearray([
  0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x00, 0x05, 0x40, 0x00, 0x00, 0x2a, 0xa0,
  0x00, 0x01, 0x55, 0x50, 0x00, 0x0a, 0xaa, 0xa8, 0x01, 0x55, 0x55, 0x50, 0x0a, 0xaa, 0xaa, 0xa8,
  0x15, 0x55, 0x55, 0x50, 0x0a, 0xaa, 0xaa, 0xa0, 0x15, 0x55, 0x55, 0x00, 0x0a, 0xaa, 0xa8, 0x00,
  0x05, 0x55, 0x40, 0x00, 0x0a, 0xaa, 0x00, 0x00, 0x05, 0x50, 0x00, 0x00, 0x0a, 0x00, 0x00, 0x00
])
# 'sensor_3c_off', 32x17px
bitmap_sensor_3c_off = bytearray([
  0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x00, 0x01, 0x50, 0x00, 0x00, 0x0a, 0xa8,
  0x00, 0x00, 0x55, 0x54, 0x00, 0x02, 0xaa, 0xa8, 0x00, 0x55, 0x55, 0x54, 0x02, 0xaa, 0xaa, 0xaa,
  0x55, 0x55, 0x55, 0x54, 0x2a, 0xaa, 0xaa, 0xa8, 0x15, 0x55, 0x55, 0x40, 0x2a, 0xaa, 0xaa, 0x00,
  0x15, 0x55, 0x50, 0x00, 0x2a, 0xaa, 0x80, 0x00, 0x15, 0x54, 0x00, 0x00, 0x2a, 0xa0, 0x00, 0x00,
  0x14, 0x00, 0x00, 0x00
])
# 'sensor_3d_on', 32x18px
bitmap_sensor_3d_on = bytearray([
  0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x01, 0xfc, 0x00, 0x00, 0x07, 0xfc,
  0x00, 0x00, 0x3f, 0xfe, 0x00, 0x01, 0xff, 0xfe, 0x00, 0x1f, 0xff, 0xfe, 0x00, 0xff, 0xff, 0xfe,
  0x1f, 0xff, 0xff, 0xfc, 0xff, 0xff, 0xff, 0xf8, 0xff, 0xff, 0xff, 0xe0, 0xff, 0xff, 0xff, 0x80,
  0xff, 0xff, 0xfc, 0x00, 0xff, 0xff, 0xf0, 0x00, 0x7f, 0xff, 0x00, 0x00, 0x7f, 0xf8, 0x00, 0x00,
  0x7f, 0x80, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00
])
# 'sensor_3d_off', 32x18px
bitmap_sensor_3d_off = bytearray([
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28, 0x00, 0x00, 0x01, 0x54, 0x00, 0x00, 0x02, 0xa8,
  0x00, 0x00, 0x15, 0x54, 0x00, 0x00, 0xaa, 0xaa, 0x00, 0x15, 0x55, 0x54, 0x00, 0xaa, 0xaa, 0xaa,
  0x15, 0x55, 0x55, 0x54, 0xaa, 0xaa, 0xaa, 0xa8, 0x55, 0x55, 0x55, 0x40, 0xaa, 0xaa, 0xaa, 0x80,
  0x55, 0x55, 0x54, 0x00, 0xaa, 0xaa, 0xa0, 0x00, 0x55, 0x55, 0x00, 0x00, 0x2a, 0xa8, 0x00, 0x00,
  0x55, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00
])
# 'sensor_3c_on', 32x17px
bitmap_sensor_3c_on = bytearray([
  0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x03, 0xf8, 0x00, 0x00, 0x1f, 0xf8,
  0x00, 0x00, 0xff, 0xfc, 0x00, 0x07, 0xff, 0xfc, 0x00, 0x7f, 0xff, 0xfe, 0x07, 0xff, 0xff, 0xfe,
  0x7f, 0xff, 0xff, 0xfe, 0x7f, 0xff, 0xff, 0xf8, 0x3f, 0xff, 0xff, 0xc0, 0x3f, 0xff, 0xff, 0x00,
  0x3f, 0xff, 0xf8, 0x00, 0x3f, 0xff, 0xc0, 0x00, 0x3f, 0xfc, 0x00, 0x00, 0x3f, 0xe0, 0x00, 0x00,
  0x3c, 0x00, 0x00, 0x00
])
# rear 'unit_cm', 24x10px
bitmap_unit_cm = bytearray([
  0xf0, 0x03, 0xc0, 0x80, 0x00, 0x40, 0xbe, 0xff, 0x40, 0xb6, 0xdb, 0x40, 0x30, 0xdb, 0x00, 0x30,
  0xdb, 0x00, 0xb6, 0xdb, 0x40, 0xbe, 0xdb, 0x40, 0x80, 0x00, 0x40, 0xf0, 0x03, 0xc0
])
# 'unit in', 24x10px
bitmap_unit_in = bytearray([
  0xf0, 0x03, 0xc0, 0x80, 0x00, 0x40, 0x8c, 0x7c, 0x40, 0x8c, 0x6c, 0x40, 0x0c, 0x6c, 0x00, 0x0c,
  0x6c, 0x00, 0x8c, 0x6c, 0x40, 0x8c, 0x6c, 0x40, 0x80, 0x00, 0x40, 0xf0, 0x03, 0xc0
])
# 'degree-temp', 24x10px
degree_temp = bytearray([
  0xf0, 0x00, 0x0f, 0x80, 0x00, 0x01, 0x80, 0x00, 0x0d, 0x80, 0x00, 0x0d, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x80, 0x00, 0x01, 0x80, 0x00, 0x01, 0x80, 0x00, 0x01, 0xf0, 0x00, 0x0f
])

rear = True
show_env = False
metric = False
dht_error = False
debug = False

# Button debouncer
# https://electrocredible.com/raspberry-pi-pico-external-interrupts-button-micropython/
def callback_1(pin):
    global interrupt_1_flag, debounce_1_time
    if (time.ticks_ms() - debounce_1_time) > 500:
        interrupt_1_flag= 1
        debounce_1_time=time.ticks_ms()
        
def callback_2(pin):
    global interrupt_2_flag, debounce_2_time
    if (time.ticks_ms() - debounce_2_time) > 500:
        interrupt_2_flag= 1
        debounce_2_time=time.ticks_ms()

button_1.irq(trigger=Pin.IRQ_FALLING, handler=callback_1)
button_2.irq(trigger=Pin.IRQ_FALLING, handler=callback_2)

# Helper functions to replace Arduino-specific functions
def constrain(val, min_val, max_val):
    return max(min_val, min(val, max_val))

def map_range(value, in_min, in_max, out_min, out_max):
    return int((value - in_min) * (out_max - out_min) // (in_max - in_min) + out_min)

def get_str_width(text):
    # In MicroPython has 8x8 fonts  x*9-1 ?
    return len(text) 
    
def calc_speed_sound(speed_sound, temp_c, temp_f, humidity):
    """
    calc speed of sound based on temp & humidity from the DHT22 sensor
    
    param:speed_sound (float): speed of sound in cm.
    param:speed of sound (float): speed of sound in cm.
    param:temp_c (float): temparature in C
    param:temp_f (float): temparature in F
    param:humidity (float): humidity %  
    """
    global dht_error, debug
    
    try:
        dht_sensor.measure()
        dht_error = False
        temp_c = dht_sensor.temperature()
        temp_f = (temp_c * 9.0 / 5.0) + 32.0
        humidity = dht_sensor.humidity()
        
        # Speed sound with temp correction: (20.05 * sqrt(273.16 + temp_c))
        # online temp/humid calc: http://resource.npl.co.uk/acoustics/techguides/speedair/
        # created spreadsheet of diffs between online temp/humid and temp formula
        # did a 2d linear fit to create my own correction, error is now +/-0.07%
        # valid for 0C to 30C temp & 75 to 102 kPa pressure
        speed_sound = (20.05 * sqrt(273.16 + temp_c)) \
                      + (0.0006545 * humidity + 0.00475) * temp_c \
                      + (0.001057 * humidity + 0.07121)

        if debug:
            #print(f"Temp °C : {temp_c:.2f}")
            print(f"Temp °F : {temp_f:.2f}")
            print(f"Humidity: {humidity:.2f}%")
            print(f"Speed Sound: {speed_sound:.1f} m/s\n")
    except Exception as e:
        print("Error reading DHT22:", str(e))
    return(speed_sound, temp_c, temp_f, humidity)

    
def ultrasonic_distance(i, speed, timeout=50000):
    """
    Measure the distance using an ultrasonic sensor with a timeout.
    
    :param i (int): index for ultrasonic sensor.
    :param speed (float): Speed of sound in m/s.
    :param timeout (int): Max usecs to wait for the echo signal return.
    :returns: cm distance or an error value.
    """
    trigger[i].low()
    utime.sleep_us(2)
    trigger[i].high()
    utime.sleep_us(5)
    trigger[i].low()

    # Wait for echo to go high, with timeout
    start = utime.ticks_us()
    while echo[i].value() == 0:
        if utime.ticks_diff(utime.ticks_us(), start) > timeout:
            print(f"Error: Sensor {i} - too close or malfunction.")
            return 0.0  # Return max distance or an error

    signal_off = utime.ticks_us()

    # Wait for echo to go low, with timeout
    start = utime.ticks_us()
    while echo[i].value() == 1:
        if utime.ticks_diff(utime.ticks_us(), start) > timeout:
            print(f"Error: Sensor {i} - no signal returned, too far.")
            return 300.0  # Return max distance

    signal_on = utime.ticks_us()

    # Calculate cm distance
    return (signal_on - signal_off) * (speed / 20000.0)

def display_environment(temp_c, temp_f, humidity, speed_sound, dist):
    """
    display just environment readings & car image for fun,
    No need to oled.fill(0) before or oled.show() after call

    param:speed of sound (float): speed of sound in cm.
    param:temp_c (float): temparature in C
    param:temp_f (float): temparature in F
    param:humidity (float): humidity %
    param:dist (float):  distance if dist = -1.0 then display error
    param:speed_sound (float): speed of sound in cm.
    """
    global metric, working_sensors, dht_error, debug
    
    oled.fill(0)
    if not dht_error:
        temp_f = (temp_c * 9.0 / 5.0) + 32.0
        oled.fill(0)
        if metric:
            oled.text(f"Temp = {temp_c:.1f}C", 0, 0)
        else:
            oled.text(f"Temp = {temp_f:.1f}F", 0, 0)
        oled.text(f"Humid= {humidity:.1f}%", 0, 12)
        if metric:
            oled.text(f"Sound={speed_sound:.1f}m/s", 0, 24)
            if dist != -1.0:
                oled.text(f"Dist= {dist:.0f}cm", 0, 55)
            else:
                oled.text(f"No ultrasonic", 0, 55)
        else:
            oled.text(f"Sound={speed_sound*3.28084:.0f}ft/s", 0, 24)
            if dist != -1.0:
                oled.text(f"Dist= {dist/2.54:.1f}in", 0, 55)
            else:
                oled.text(f"No ultrasonic", 0, 55)
    else:
        oled.text(f"No Temp/Humidity", 0, 10)
        oled.text(f" Sensor Working",  0, 20) 
        
    oled.blit(FrameBuffer(bitmap_artcar_image_back,56,15, MONO_HLSB),22,36)
    oled.show()
    return


def flip_bitmap_vert(bitmap, width, height):
    """
    Flip a byte array bitmap vertically.

    :param bitmap: byte array of image
    :param width: The width of the image in pixels (must be divisible by 8)
    :param height: The height of the image in pixels
    :return: new byte array with image flipped vertically (top for bottom)
    """
    # Each row has `width // 8` bytes (because 8 pixels = 1 byte).
    row_size = width // 8
    flipped_bitmap = bytearray(len(bitmap))

    # reverse the order of each row.
    for y in range(height):
        # Copy each row from the original image.
        start_index = y * row_size
        end_index = start_index + row_size
        row_data = bitmap[start_index:end_index]

        # Place it in the flipped position in the new byte array.
        flipped_start = (height - y - 1) * row_size
        flipped_bitmap[flipped_start:flipped_start + row_size] = row_data

    return flipped_bitmap


def blit_white_only(oled, source_fb, w, h, x, y):
    """
    only send white pixels one by one
    :param oled: display
    :param source_fb:
    :param w: width of the framebuffer
    :param h: height of the framebuffer
    :x: x-position for display
    :x: y-position for display
    """
    for row in range(h):
        for col in range(w):
            # Extract pixel from the source frame buffer
            pixel = source_fb.pixel(col, row)
            if pixel == 1:  # Only copy white pixels
                oled.pixel(x + col, y + row, 1)  # Set the pixel on the OLED

def blit_white_only_flip(oled, source_fb, w, h, x, y):
    """
    Blit the frame buffer as if it is vertically flipped, and only send white pixels one by one.
    :param oled: display
    :param source_fb: source frame buffer
    :param w: width of the framebuffer
    :param h: height of the framebuffer
    :param x: x-position for display
    :param y: y-position for display
    :return 
    """
    for row in range(h):
        for col in range(w):
            # Extract pixel from the vertically flipped source frame buffer
            flipped_row = h - row - 1
            pixel = source_fb.pixel(col, flipped_row)
            if pixel == 1:  # Only copy white pixels
                oled.pixel(x + col, y + row, 1)  # Set the pixel on the OLED
                
def display_car (temp_c, temp_f):
    """
    display_car & temp, Need oled.fill(0) before call & oled.show() after call

    param:temp_c (float): temparature in C
    param:temp_f (float): temparature in F
    """
    global rear, metric, dht_error, debug
    
    if rear:
        oled.blit(FrameBuffer(bitmap_artcar_image_back,56,15, MONO_HLSB), 36, 0)
        oled.blit(FrameBuffer(degree_temp,24,10, MONO_HLSB), 104, 0)
        
        if metric:
            oled.blit(FrameBuffer(bitmap_unit_cm,24,10, MONO_HLSB), 0, 0)
            if not dht_error:
                oled.text(f"{temp_c:.0f}", 108, 2)
            else:
                oled.text("xx", 108, 2)
        else:
            oled.blit(FrameBuffer(bitmap_unit_in,24,10, MONO_HLSB), 0, 0)
            if not dht_error:
                oled.text(f"{temp_f:.0f}", 108, 2)
            else:
                oled.text("xx", 108, 2)
    else:
        oled.blit(FrameBuffer(bitmap_artcar_image_front,56,15, MONO_HLSB), 36, DISP_HEIGHT-15)
        oled.blit(FrameBuffer(degree_temp,24,10, MONO_HLSB), 104, DISP_HEIGHT-10)
        
        if metric:
            oled.blit(FrameBuffer(bitmap_unit_cm,24,10, MONO_HLSB), 0, DISP_HEIGHT-10)
            if not dht_error:
                oled.text(f"{temp_c:.0f}", 108, DISP_HEIGHT-8)
            else:
                oled.text("xx", 108, DISP_HEIGHT-8)
        else:
            oled.blit(FrameBuffer(bitmap_unit_in,24,10, MONO_HLSB), 0, DISP_HEIGHT-10)
            if not dht_error:
                oled.text(f"{temp_f:.0f}", 108, DISP_HEIGHT-8)
            else:
                oled.text("xx", 108, DISP_HEIGHT-8)
    return

def display_tiles_dist():
    """
    display_car & temp, Need oled.show() after call
    """
    global sensor, working_sensors, rear, dht_error, debug, dist_step_01, dist_step_02, dist_step_03, dist_step_04

    if not working_sensors:
        oled.text(" No Ultrasonic",  5, 30)
        oled.text("Sensors Working", 5, 40)
        return

    for i in working_sensors:
        if rear:
            # Display bitmap for sensor 2
            if sensor[1].cm > dist_step_01:
                oled.blit(FrameBuffer(bitmap_sensor_2a_on, 32, 9, MONO_HLSB), 48, 23)
            else:
                oled.blit(FrameBuffer(bitmap_sensor_2a_off, 32, 9, MONO_HLSB), 48, 23)
            if sensor[1].cm > dist_step_02:
                oled.blit(FrameBuffer(bitmap_sensor_2b_on, 32, 9, MONO_HLSB), 48, 33)
            else:
                oled.blit(FrameBuffer(bitmap_sensor_2b_off, 32, 9, MONO_HLSB), 48, 33)
            if sensor[1].cm > dist_step_03:
                oled.blit(FrameBuffer(bitmap_sensor_2c_on, 32, 10, MONO_HLSB), 47, 42)
            else:
                oled.blit(FrameBuffer(bitmap_sensor_2c_off, 32, 10, MONO_HLSB), 47, 42)
            if sensor[1].cm > dist_step_04:
                oled.blit(FrameBuffer(bitmap_sensor_2d_on, 40, 10, MONO_HLSB), 42, 52)
            else:
                oled.blit(FrameBuffer(bitmap_sensor_2d_off, 40, 10, MONO_HLSB), 42, 52)
            
            # DO NOT LIKE THIS SOLUTION, but couldn't find anotehr way to only
            # have white pixels updating screen, blit forces all pixes B & W to OLED
            # NEED FIX black pixels overwrite white, need to 'or' the bits together
    #         oled.blit(FrameBuffer(bitmap_sensor_1a_on, 32, 14, MONO_HLSB), 24, 17) 
    #         oled.blit(FrameBuffer(bitmap_sensor_1b_on, 32, 16, MONO_HLSB), 21, 25) 
    #         oled.blit(FrameBuffer(bitmap_sensor_1c_on, 32, 17, MONO_HLSB), 18, 34)
    #         oled.blit(FrameBuffer(bitmap_sensor_1d_on, 32, 18, MONO_HLSB), 16, 43)
    # 
    #         oled.blit(FrameBuffer(bitmap_sensor_2a_on, 32,  9, MONO_HLSB), 48, 23)
    #         oled.blit(FrameBuffer(bitmap_sensor_2b_on, 32,  9, MONO_HLSB), 48, 33)
    #         oled.blit(FrameBuffer(bitmap_sensor_2c_on, 32, 10, MONO_HLSB), 47, 42)
    #         oled.blit(FrameBuffer(bitmap_sensor_2d_on, 40, 10, MONO_HLSB), 42, 52) 
    # 
    #         oled.blit(FrameBuffer(bitmap_sensor_3a_on, 32, 14, MONO_HLSB), 72, 17)
    #         oled.blit(FrameBuffer(bitmap_sensor_3b_on, 32, 16, MONO_HLSB), 74, 25)
    #         oled.blit(FrameBuffer(bitmap_sensor_3c_on, 32, 17, MONO_HLSB), 77, 34)
    #         oled.blit(FrameBuffer(bitmap_sensor_3d_on, 32, 18, MONO_HLSB), 80, 43)

            # Display bitmap for sensor 1
            if sensor[1].cm > dist_step_01:
                bmp_1a = FrameBuffer(bitmap_sensor_1a_on, 32, 14, MONO_HLSB)  #, 24, 17)
            else:
                bmp_1a = FrameBuffer(bitmap_sensor_1a_off, 32, 14, MONO_HLSB)  #, 24, 17)
            if sensor[1].cm > dist_step_02:
                bmp_1b = FrameBuffer(bitmap_sensor_1b_on, 32, 16, MONO_HLSB)  #, 21, 25)
            else:
                bmp_1b = FrameBuffer(bitmap_sensor_1b_off, 32, 16, MONO_HLSB)  #, 21, 25)
            if sensor[1].cm > dist_step_03:
                bmp_1c = FrameBuffer(bitmap_sensor_1c_on, 32, 17, MONO_HLSB)  #, 18, 34)
            else:
                bmp_1c = FrameBuffer(bitmap_sensor_1c_off, 32, 17, MONO_HLSB)  #, 18, 34)
            if sensor[1].cm > dist_step_04:
                bmp_1d = FrameBuffer(bitmap_sensor_1d_on, 32, 18, MONO_HLSB)  #, 16, 43)
            else:
                bmp_1d = FrameBuffer(bitmap_sensor_1d_off, 32, 18, MONO_HLSB)  #, 16, 43)
            blit_white_only(oled, bmp_1a, 32, 14, 24, 17)
            blit_white_only(oled, bmp_1b, 32, 16, 21, 25)
            blit_white_only(oled, bmp_1c, 32, 17, 18, 34)
            blit_white_only(oled, bmp_1d, 32, 18, 16, 43)

            # Display bitmap for sensor 3
            if sensor[1].cm > dist_step_01:
                bmp_3a = FrameBuffer(bitmap_sensor_3a_on, 32, 14, MONO_HLSB)  #, 72, 17)
            else:
                bmp_3a = FrameBuffer(bitmap_sensor_3a_off, 32, 14, MONO_HLSB)  #, 72, 17)
            if sensor[1].cm > dist_step_02:
                bmp_3b = FrameBuffer(bitmap_sensor_3b_on, 32, 16, MONO_HLSB)  #, 74, 25)
            else:
                bmp_3b = FrameBuffer(bitmap_sensor_3b_off, 32, 16, MONO_HLSB)  #, 74, 25)
            if sensor[1].cm > dist_step_03:
                bmp_3c = FrameBuffer(bitmap_sensor_3c_on, 32, 17, MONO_HLSB)  #, 77, 34)
            else:
                bmp_3c = FrameBuffer(bitmap_sensor_3c_off, 32, 17, MONO_HLSB)  #, 77, 34)
            if sensor[1].cm > dist_step_04:
                bmp_3d = FrameBuffer(bitmap_sensor_3d_on, 32, 18, MONO_HLSB)  #, 80, 43)
            else:
                bmp_3d = FrameBuffer(bitmap_sensor_3d_off, 32, 18, MONO_HLSB)  #, 80, 43)
            blit_white_only(oled, bmp_3a, 32, 14, 72, 17)
            blit_white_only(oled, bmp_3b, 32, 16, 74, 25)
            blit_white_only(oled, bmp_3c, 32, 17, 77, 34)
            blit_white_only(oled, bmp_3d, 32, 18, 80, 43)
        else:   
            # Display bitmap for aensor 2
            if sensor[1].cm > dist_step_01:
                flipped = flip_bitmap_vert(bitmap_sensor_2a_on, 32, 9)
                oled.blit(FrameBuffer(flipped, 32, 9, MONO_HLSB), 48, DISP_HEIGHT-23-9)
            else:
                flipped = flip_bitmap_vert(bitmap_sensor_2a_off, 32, 9)
                oled.blit(FrameBuffer(flipped, 32, 9, MONO_HLSB), 48, DISP_HEIGHT-23-9)
            if sensor[1].cm > dist_step_02:
                flipped = flip_bitmap_vert(bitmap_sensor_2b_on, 32, 9)
                oled.blit(FrameBuffer(flipped, 32, 9, MONO_HLSB), 48, DISP_HEIGHT-33-9)
            else:
                flipped = flip_bitmap_vert(bitmap_sensor_2b_off, 32, 9)
                oled.blit(FrameBuffer(flipped, 32, 9, MONO_HLSB), 48, DISP_HEIGHT-33-9)
            if sensor[1].cm > dist_step_03:
                flipped = flip_bitmap_vert(bitmap_sensor_2c_on, 32, 10)
                oled.blit(FrameBuffer(flipped, 32, 10, MONO_HLSB), 47, DISP_HEIGHT-42-10)
            else:
                flipped = flip_bitmap_vert(bitmap_sensor_2c_off, 32, 10)
                oled.blit(FrameBuffer(flipped, 32, 10, MONO_HLSB), 47, DISP_HEIGHT-42-10)
            if sensor[1].cm > dist_step_04:
                flipped = flip_bitmap_vert(bitmap_sensor_2d_on, 40, 10)
                oled.blit(FrameBuffer(flipped, 40, 10, MONO_HLSB), 42, DISP_HEIGHT-52-10)
            else:
                flipped = flip_bitmap_vert(bitmap_sensor_2d_off, 40, 10)
                oled.blit(FrameBuffer(flipped, 40, 10, MONO_HLSB), 42, DISP_HEIGHT-52-10)
                
            # Display bitmap for sensor 3 (notice using images for sendor 3
            if sensor[1].cm > dist_step_01:
                bmp_3a = FrameBuffer(bitmap_sensor_3a_on, 32, 14, MONO_HLSB)  #, 72, 17)
            else:
                bmp_3a = FrameBuffer(bitmap_sensor_3a_off, 32, 14, MONO_HLSB)  #, 72, 17)
            if sensor[1].cm > dist_step_02:
                bmp_3b = FrameBuffer(bitmap_sensor_3b_on, 32, 16, MONO_HLSB)  #, 74, 25)
            else:
                bmp_3b = FrameBuffer(bitmap_sensor_3b_off, 32, 16, MONO_HLSB)  #, 74, 25)
            if sensor[1].cm > dist_step_03:
                bmp_3c = FrameBuffer(bitmap_sensor_3c_on, 32, 17, MONO_HLSB)  #, 77, 34)
            else:
                bmp_3c = FrameBuffer(bitmap_sensor_3c_off, 32, 17, MONO_HLSB)  #, 77, 34)
            if sensor[1].cm > dist_step_04:
                bmp_3d = FrameBuffer(bitmap_sensor_3d_on, 32, 18, MONO_HLSB)  #, 80, 43)
            else:
                bmp_3d = FrameBuffer(bitmap_sensor_3d_off, 32, 18, MONO_HLSB)  #, 80, 43)
            blit_white_only_flip(oled, bmp_3a, 32, 14, 72, DISP_HEIGHT-17-14)
            blit_white_only_flip(oled, bmp_3b, 32, 16, 74, DISP_HEIGHT-25-16)
            blit_white_only_flip(oled, bmp_3c, 32, 17, 77, DISP_HEIGHT-34-17)
            blit_white_only_flip(oled, bmp_3d, 32, 18, 80, DISP_HEIGHT-43-18)
            
            # Display bitmap for sensor 3 (notice using images for sendor 3
            if sensor[1].cm > dist_step_01:
                bmp_1a = FrameBuffer(bitmap_sensor_1a_on, 32, 14, MONO_HLSB)  #, 72, 17)
            else:
                bmp_1a = FrameBuffer(bitmap_sensor_1a_off, 32, 14, MONO_HLSB)  #, 72, 17)
            if sensor[1].cm > dist_step_02:
                bmp_1b = FrameBuffer(bitmap_sensor_1b_on, 32, 16, MONO_HLSB)  #, 74, 25)
            else:
                bmp_1b = FrameBuffer(bitmap_sensor_1b_off, 32, 16, MONO_HLSB)  #, 74, 25)
            if sensor[1].cm > dist_step_03:
                bmp_1c = FrameBuffer(bitmap_sensor_1c_on, 32, 17, MONO_HLSB)  #, 77, 34)
            else:
                bmp_1c = FrameBuffer(bitmap_sensor_1c_off, 32, 17, MONO_HLSB)  #, 77, 34)
            if sensor[1].cm > dist_step_04:
                bmp_1d = FrameBuffer(bitmap_sensor_1d_on, 32, 18, MONO_HLSB)  #, 80, 43)
            else:
                bmp_1d = FrameBuffer(bitmap_sensor_1d_off, 32, 18, MONO_HLSB)  #, 80, 43)
            blit_white_only_flip(oled, bmp_1a, 32, 14, 24, DISP_HEIGHT-17-14)
            blit_white_only_flip(oled, bmp_1b, 32, 16, 21, DISP_HEIGHT-25-16)
            blit_white_only_flip(oled, bmp_1c, 32, 17, 18, DISP_HEIGHT-34-17)
            blit_white_only_flip(oled, bmp_1d, 32, 18, 16, DISP_HEIGHT-43-18)

    for i in working_sensors:
        if metric:
            int_string = str(int(sensor[i].cm))
        else:
            int_string = str(int(sensor[i].inch))
        digits = get_str_width(int_string)
        
        startpos_x = sensor[i].label_startpos_x + (3-digits)*4
        endpos_x = sensor[i].label_endpos_x + (3-digits)*4

        # Display distance label    
        xpos = int(map_range(constrain(sensor[i].cm, MIN_DIST, MAX_DIST),
                                          MIN_DIST, MAX_DIST, startpos_x, endpos_x))
        ypos = int(map_range(constrain(sensor[i].cm, MIN_DIST, MAX_DIST),
                                          MIN_DIST, MAX_DIST, sensor[i].label_startpos_y, sensor[i].label_endpos_y))    
        if rear:
            oled.fill_rect(xpos, ypos-1, 8*digits, 9, 0) 
            oled.text(int_string, xpos, ypos)
        else:
            oled.fill_rect(xpos, DISP_HEIGHT-ypos-1-8, 8*digits, 9, 0) 
            oled.text(int_string, xpos, DISP_HEIGHT-ypos-8)
    return


# startup code
print("Starting...")
print("====================================")
print(implementation[0], uname()[3],
      "\nrun on", uname()[4])
print("====================================")

print(f"Default Speed Sound: {SPEED_SOUND_20C_70H:.1f} m/s\n")

# at start display artcar image at top of oled
temp_c = 20.56
temp_f = 69.0
humidity = 70.0
speed_sound = SPEED_SOUND_20C_70H
oled.fill(0)
display_car(temp_c, temp_f)
oled.show()
zzz(3)

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
    dht_error = True

working_sensors = []
nonworking_sensors = []
for i in range(NUMBER_OF_SENSORS):
    distance = ultrasonic_distance(i, speed_sound)  # speed of sound in air is ~343 m/s
    if 0.0 < distance < 300.0:
        working_sensors.append(i)
    else:
        nonworking_sensors.append(i)
print(f"Working Ultrasonic sensors: {working_sensors}")
print(f"Non-working Ultrasonic sensors: {nonworking_sensors}")

# main loop
loop_time = time.ticks_ms()
while True:
    elapsed_time = time.ticks_diff(time.ticks_ms(), loop_time)
    if debug: print(f"Loop time duration ={elapsed_time}")
    
    # Button 1: cm/in
    if interrupt_1_flag == 1:
        interrupt_1_flag = 0
        if debug: print("button 1 Interrupt Detected: in/cm")
        metric = not metric  # Toggle between metric and imperial units

    # Button 2: rear sensors or front sensors
    if interrupt_2_flag == 1:
        interrupt_2_flag = 0
        if debug: print("button 2 Interrupt Detected: rear/front")
        rear = not rear   # Toggle between rear /front

    # every 3 sec, calc speed of sound based on temp & humidity
    if elapsed_time > 3000:
        loop_time = time.ticks_ms()
        speed_sound, temp_c, temp_f, humidity = calc_speed_sound(speed_sound, temp_c, temp_f, humidity)
    
    for i in working_sensors:
        #get distance from ultrasonic sensor, 30ms round trip 514cm or 202in
        sensor[i].cm = ultrasonic_distance(i, speed_sound, timeout=30000)
        sensor[i].inch = sensor[i].cm/2.54
    # dummy up three sensors
    sensor[0].cm = sensor[1].cm
    sensor[0].inch = sensor[1].inch
    sensor[2].cm = sensor[1].cm
    sensor[2].inch = sensor[1].inch
    
    if show_env:
        if working_sensors:
            display_environment(temp_c, temp_f, humidity, speed_sound, sensor[working_sensors[0]].cm)
        else:
            display_environment(temp_c, temp_f, humidity, speed_sound, -1.0)
    else:
        oled.fill(0)
        display_car(temp_c, temp_f)
        display_tiles_dist()
        oled.show()

    #Every loop do this
    led.toggle()
    time.sleep_ms(300)
    
# # AT SOME POINT
# try:
# # where the action happens  
#     while True:
# #
# except KeyboardInterrupt:
#     oled.fill(0)   # turn off oled
#     oled.show()    # show on oled
#     GPIO.cleanup()
