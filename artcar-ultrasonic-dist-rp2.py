from machine import Pin, Timer
import machine
import dht
import time
import math
#ic2
from machine import Pin, I2C
from ssd1306 import SSD1306_I2C
#
import framebuf
'''
from luma.core.interface.serial import spi
from luma.oled.device import ssd1309
from luma.core.render import canvas
from PIL import ImageFont
'''

# pins
dht_pin = machine.Pin(2)
led = Pin(25, Pin.OUT)

# ic2 pins
# https://www.tomshardware.com/how-to/oled-display-raspberry-pi-pico

# spi pins
# vcc = 3.3v
# gnd
# scl (SCLK) GPIO 11 p15
# SDA (MOSI) GPIO 10 p14
# RES (RST)  GPIO 25? RESET p30
# DC GPIO 24? GPIO 9 p12
# CS chip select CE chip enable GPIO 8 p11

'''
#display SSD spi
#serial = spi(device=0, port=0, gpio_SCLK=15, gpio_SDA=14, gpio_RST=30, gpio_DC=12, gpio_CE=11)
serial = spi(port=0, device=0, gpio_DC=12, gpio_RST=25, gpio_CS=11)
device = ssd1309(serial)
'''
i2c=I2C(0,sda=Pin(0), scl=Pin(1), freq=400000)
oled = SSD1306_I2C(128, 64, i2c)

timecall = Timer()


def tick(timer):
    global led
    led.toggle()


timecall.init(freq=2.5, mode=Timer.PERIODIC, callback=tick)
dht_sensor = dht.DHT22(dht_pin)
loopTime = time.ticks_ms()
# init soundSpeed if dht22 sensor fails
soundSpeed = 344.11
debug = True

# startup code
print("Starting...")
print(f"Speed Sound: {soundSpeed:.1f} m/s")
print()
bitmap_artcar_image=bytearray(b'\xc9\x04\x59\x11\x0c\x08\x43\xc8\x82\x8e\x90\x93\x10\x93\xe0\x63\x00\x78\xe0\xe3\x07\xff\x9f\xff\xff\xff\xfc\xff\xc0\x00\x00\x00\x00\x00\x03\x40\x1c\xf3\xe3\x8e\x78\x02\x47\x22\x88\x84\x51\x44\xe2\x45\x22\x88\x84\x11\x44\xa2\x47\x22\xf0\x84\x11\x78\xe2\x20\x3e\x88\x84\x1f\x44\x04\x10\x22\x88\x84\x51\x44\x08\x0c\x22\x88\x83\x91\x44\x30\x03\x00\x00\x00\x00\x00\xc0\x00\xff\xff\xff\xff\xff\x00\x00\x00\x00\x00\x00\x00\x00')
fb = framebuf.FrameBuffer(bitmap_artcar_image,56,15, framebuf.MONO_HLSB)
oled.blit(fb,36,0)
oled.show()
'''
with canvas(device) as draw:
    draw.rectangle(device.bounding_box, outline="white", fill="black")
    draw.text((30, 40), "Hello World", fill="white")
'''


# main loop
while True:
    elapsed_time = time.ticks_diff(time.ticks_ms(), loopTime)
    print("Elapsed time:", elapsed_time, "msec\n")
    loopTime = time.ticks_ms()
    
    #update speed of sound using temp & humidity from dht22
    try:
        dht_sensor.measure()
        tempC = dht_sensor.temperature()
        tempF = (tempC * 9.0 / 5.0) + 32.0
        humidity = dht_sensor.humidity()
        # Speed sound with temp, best I could find was: (20.05 * sqrt(273.16 + tempC))
        # Speed sound temp & humidity calculator: http://resource.npl.co.uk/acoustics/techguides/speedair/
        # sing online calculated constructed spreadsheet of values
        # hten 2d linear fit with formula above to add correction, error is +/-0.07% in range
        # valid over the temp range 0 to 30C temp & 75 to 102 kPa pressure
        soundSpeed = (20.05 * math.sqrt(273.16 + tempC)) + (0.0006545 * humidity + 0.00475) * tempC + (0.001057 * humidity + 0.07121)
        #
        if debug:
            print(f"Temp °C : {tempC:.2f}")
            print(f"Temp °F : {tempF:.2f}")
            print(f"Humidity: {humidity:.2f}%")
            print(f"Speed Sound: {soundSpeed:.1f} m/s")
        # fill with black and the reprint
        oled.fill(0)
        oled.text(f"Sound={soundSpeed:.1f}m/s", 0, 0)
        oled.text(f"TempF= {tempF:.1f}F", 0, 12)
        oled.text(f"humid= {humidity:.1f}%", 0, 24)
        # don't need to change PROGMEM DATA
        #const unsigned char bitmap_artcar_image[] PROGMEM = {0xc9, 0x04, 0x59, ...
        # goes to bitmap_artcar_image=bytearray(b'\xc9\x04\x59
        bitmap_artcar_image=bytearray(b'\xc9\x04\x59\x11\x0c\x08\x43\xc8\x82\x8e\x90\x93\x10\x93\xe0\x63\x00\x78\xe0\xe3\x07\xff\x9f\xff\xff\xff\xfc\xff\xc0\x00\x00\x00\x00\x00\x03\x40\x1c\xf3\xe3\x8e\x78\x02\x47\x22\x88\x84\x51\x44\xe2\x45\x22\x88\x84\x11\x44\xa2\x47\x22\xf0\x84\x11\x78\xe2\x20\x3e\x88\x84\x1f\x44\x04\x10\x22\x88\x84\x51\x44\x08\x0c\x22\x88\x83\x91\x44\x30\x03\x00\x00\x00\x00\x00\xc0\x00\xff\xff\xff\xff\xff\x00\x00\x00\x00\x00\x00\x00\x00')
        fb = framebuf.FrameBuffer(bitmap_artcar_image,56,15, framebuf.MONO_HLSB)
        oled.blit(fb,0,36)
        oled.show()
    except Exception as e:
        print("Error reading DHT22:", str(e))
    #
    #time.sleep(0.1)

    