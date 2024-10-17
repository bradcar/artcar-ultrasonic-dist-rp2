from machine import Pin, Timer
import machine
import dht
import time
import utime
import math
#from writer import Writer
from time import sleep as zzz
from writer_short import Writer
import freesans14 as font0
import freesans17 as font1
import freesans20 as font2
#ic2
from machine import Pin, I2C
from ssd1306 import SSD1306_I2C
#
import framebuf

# https://coxxect.blogspot.com/2024/10/multi-ssd1306-oled-on-raspberry-pi-pico.html

# pins
# https://www.tomshardware.com/how-to/oled-display-raspberry-pi-pico
i2c=I2C(0,sda=Pin(0), scl=Pin(1), freq=400000)

led = Pin(25, Pin.OUT)

oled = SSD1306_I2C(128, 64, i2c)

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

# projects: https://www.tomshardware.com/news/raspberry-pi
# fonts: https://www.youtube.com/watch?v=kxlN1knBpQ0

# startup code
print("Starting...")

# at start display artcar image at top of oled
oled.fill(0)
fb = framebuf.FrameBuffer(bitmap_artcar_image,56,15, framebuf.MONO_HLSB)
oled.blit(fb,40,8)
oled.text("12345678901", 0, 24)

font_writer = Writer(oled,font2, verbose=False)

for char in range (32, 126):
    font_writer.set_textpos(0,32)
    font_writer.printstring(f"{char}: {chr(char)} ")
    print(f"{char}")
    zzz(.2)
    oled.show()

# main loop
while True:
    led.toggle()
    time.sleep_ms(500)

