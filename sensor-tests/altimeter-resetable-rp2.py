# Raspberry Pi Pico 2 project - Altimeter Settable to know elevation or sea level pressure - Oct 2024
#
# Uses BME680, uses oled SPI display to show measured temp, humidity, pressure, IAQ, altitude
#
# based on Brad's Arduino UNO and
#    - 128x64 OLED Display SDI
#    - in/cm F/C changed with button #1
#    - set known altitude, input with potentiameter, or
#    - set known sea kevel pressure from nearest airport, input with potentiameter
#    - buttons debounced with efficient rp2 interrupts -- nice!
#    - ssd1309 SDI or I2C code (sw is ssd1306)
# my home office is
#    ~361 feet elevation, 110.03m
#    first bme680 says 303.5 feet, 92.51 m
#    +57.5' +17.5m correction needed)
# my garage is at <todo> feet elevation

#
#  by bradcar
#
# Key links
#     https://micropython.org/download/RPI_PICO2/ for latest .uf2 preview
#     https://docs.micropython.org/en/latest/esp8266/tutorial/ssd1306.html
# pycharm stubs
#   : https://micropython-stubs.readthedocs.io/en/main/packages.html#mp-packages
#
# TODOs
#  * add button #3 for ???
import time
import utime

from machine import Pin, I2C
from os import uname
from sys import implementation, stdin
from time import sleep as zzz
from math import log

from bme680 import BME680_I2C
from framebuf import FrameBuffer, MONO_HLSB

# ic2
# from machine import I2C
# from ssd1306 import SSD1306_I2C
from ssd1306 import SSD1306_SPI

# Constants for  setup
DISP_WIDTH = 128
DISP_HEIGHT = 64


# === PINS ===
# internal pins
on_pico_temp = machine.ADC(4)

# external pins
uart0 = machine.UART(0, 115200, tx=Pin(0), rx=Pin(1))
# dht_pin = machine.Pin(2)
button_1 = Pin(2, Pin.IN, Pin.PULL_UP)  # interrupt cm/in button pins
button_2 = Pin(3, Pin.IN, Pin.PULL_UP)  # interrupt rear/front button pins
button_3 = Pin(4, Pin.IN, Pin.PULL_UP)  # interrupt rear/front button pins
buzzer = Pin(5, Pin.OUT)

# https://www.tomshardware.com/how-to/oled-display-raspberry-pi-pico
# i2c=I2C(0,sda=Pin(12), scl=Pin(13), freq=400000)
# oled = SSD1306_I2C(DISP_WIDTH, DISP_HEIGHT, i2c)

# ssd1306 SDI SW setup for ssd1309 SDI
cs = machine.Pin(9)  # dummy (any un-used pin), no connection
# scl (SCLK) gp10
# SDA (MOSI) gp11
res = machine.Pin(12)  # RES (RST)  gp12
dc = machine.Pin(13)  # DC         gp13

led = Pin(25, Pin.OUT)
# Pin assignment  i2c1 
i2c = I2C(id=1, scl=Pin(27), sda=Pin(26))
ds_pin = machine.Pin(28)

bme = BME680_I2C(i2c=i2c)

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
DWELL_MS_LOOP = 100
PDX_SLP_1013 = 1009.90
OVER_TEMP_WARNING = 70.0

# Button debouncer with efficient interrupts, which don't take CPU cycles!
# https://electrocredible.com/raspberry-pi-pico-external-interrupts-button-micropython/
def callback(pin):
    global interrupt_1_flag, interrupt_2_flag, debounce_1_time, debounce_2_time
    if pin == button_1 and (int(time.ticks_ms()) - debounce_1_time) > 500:
        interrupt_1_flag = 1
        debounce_1_time = time.ticks_ms()
    elif pin == button_2 and (int(time.ticks_ms()) - debounce_2_time) > 500:
        interrupt_2_flag = 1
        debounce_2_time = time.ticks_ms()


button_1.irq(trigger=Pin.IRQ_FALLING, handler=callback)
button_2.irq(trigger=Pin.IRQ_FALLING, handler=callback)


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

def iaq_quality(iaq_rating):
    """
    Calculate text to append to IAQ numerical rating
     IAQ: (0- 50 low, 51-100 ave, 101-150 poor, 151-200 bad, 201-300 VBad, 301-500 danger)

    :param :iaq_rating: the rating to score
    :returns: string to add context to the number
    """
    if iaq_rating < 50:
        return "low"
    elif iaq_rating < 100:
        return "ave"
    elif iaq_rating < 150:
        return "poor"
    elif iaq_rating < 200:
        return "bad"
    elif iaq_rating < 300:
        return "V Bad"
    else:
        return "DANGER"

def calc_sea_level_pressure(hpa, meters):
    """
    Calculate the sea level pressure from the hpa pressure at a known elevation
    
    :param :sea_level: sea level hpa from closest airport
    :returns: sea level hpa based on known altitude
    """
    sea_level_pressure_hpa = hpa / (1.0 - (meters / 44330.0)) ** 5.255

    return sea_level_pressure_hpa


def calc_altitude(hpa, slp):
    """
    Calculate the Altitude from sea level pressure and hpa pressure 
    
    :param :sea_level: sea level hpa from closest airport
    :returns: sea level hpa based on known altitude
    """
    meters = 44330.0 * (1.0 - (hpa/slp)**(1.0/5.255) )
    return meters


def bme_temp_humid_hpa_iaq_alt(sea_level):
    """
    read temp, humidity, pressure, Indoor Air Qualtiy (IAQ) from the BME680 sensor
    measurement takes ~189ms
    function iaq_quality(iaq) will show the textual scale
     IAQ: (0- 50 low, 51-100 ave, 101-150 poor, 151-200 bad, 201-300 VBad, 301-500 danger)
    
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
        
        # derive altitude from pressure & sea level pressure
        #    meters = 44330.0 * (1.0 - (hpa_pressure/sea_level)**(1.0/5.255) )
        meters = calc_altitude(hpa_pressure, sea_level)
        iaq = log(gas_resist) + 0.04 * percent_humidity
        
        if debug:
            print(f"BME680 Temp °C = {temp_c:.2f} C")
            print(f"BME680 Humidity = {percent_humidity:.1f} %")
            print(f"BME680 Pressure = {hpa_pressure:.2f} hPA")
            print(f"BME680 iaq = {iaq:.1f} {iaq_quality(iaq)}")
            print(f"BME680 Alt = {meters * 3.28084:.2f} feet \n")
            
    except OSError as e:
        print("BME680: Failed to read sensor.")
        return None, None, None, None, None, "ERROR_BME680:" + str(e)
    
    return temp_c, percent_humidity, hpa_pressure, iaq, meters, None


def display_car(celsius, fahrenheit):
    """
    display_car & temp, Need oled.fill(0) before call & oled.show() after call

    :param celsius: temp in C to display
    :param fahrenheit: temp in F to display
    """

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
    return


def display_environment(buzz):
    """
    display just environment readings & car image(for fun)
    No need to oled.fill(0) before or oled.show() after call

    param:dist:  distance if dist = -1.0 then display error
    param:buzz:  distance if dist = -1.0 then display error
    """
    oled.fill(0)
#     if dist < 120.0 and buzz:
#         buzzer.on()
#     elif buzz:
#         buzzer.off()
    if temp_c:
        if metric:
            oled.text(f"Temp = {temp_c:.1f}C", 0, 0)
        else:
            oled.text(f"Temp = {temp_f:.1f}F", 0, 0)
        if humidity: oled.text(f"Humid= {humidity:.1f}%", 0, 12)
    else:
        oled.text(f"No Temp/Humidity", 0, 10)
        oled.text(f" Sensor Working", 0, 20)
        
    oled.text(f"IAQ = {iaq:.0f} {iaq_quality(iaq)}", 0, 24)

    if metric:
        oled.text(f"Bar = {pressure_hpa:.1f} hpa", 0, 36)
    else:
        oled.text(f"Bar = {pressure_hpa * 0.02953:.2f}\"", 0, 36)

    if metric:
        oled.text(f"Alt = {altitude_m:.1f}m", 0, 52)
    else:
        oled.text(f"Alt = {altitude_m * 3.28084:.0f}\'", 0, 52)

#     oled.blit(FrameBuffer(bitmap_artcar_image_back, 56, 15, MONO_HLSB), 22, 36)
    oled.show()
    return


def button2_not_pushed():
    global interrupt_2_flag
    debug = True
    if interrupt_2_flag == 1:
        interrupt_2_flag = 0
        return False
    else:
        return True


def update_numbers(alt, press, x):
    """
    oled display of alt & press,
    input one is hightlighted by white box/black letters
    the other one shown in black box/white letters
    
    :params pressure: if 1 update altitude if 0 update pressure
    :params pressure: if 1 update altitude if 0 update pressure
    :params x: if 1 update altitude if 0 update pressure
    """
    if metric:
        string = f"{alt:.0f}m"
    else:
        string = f"{alt * 3.28084:.0f}\'"
    oled.text(f"Alt = ", 0, 20, 1)
    # if x=1 updating alt, which is white box & black letters
    oled.fill_rect(45, 19, 128-45, 9, x) 
    oled.text(f"{string}", 45, 20, 1-x)
        
    if metric:
        string = f"{press:.1f} hpa"
    else:
        string = f"{press * 0.02953:.2f}\""
    oled.text(f"Sea = ", 0, 36, 1)
    # if x=1 updating alt, which is black box & white letters
    oled.fill_rect(45, 35, 128-45, 9, 1-x) 
    oled.text(f"{string}", 45, 36, x)     
    oled.show()
    return


def input_known_values(buzz):
    """
    set known

    param:buzz: buzz if move to next input
    """
    error = None
    if buzz:
        buzzer.on()
        zzz(.2)
        buzzer.off()
    #
    oled.fill(0)
    oled.text(f"setting...", 0, 0)
    update_numbers(altitude_m, pressure_hpa, 1)
        
#### Enter in New Altitude
    not_stop_loop = True
#     while (button2_not_pushed()):
    while (not_stop_loop):
        new_alt = int(input("Enter desired Alt:\n"))
        new_pressure = calc_sea_level_pressure(pressure_hpa, new_alt)
        if debug:
            print(f"{new_alt=}")
            print(f"{sea_level_pressure_hpa=}")
            print(f"{new_pressure=}")
        update_numbers(new_alt, new_pressure, 1)
        not_stop_loop = False
        
    update_numbers(new_alt, new_pressure, 0)
    
#### Enter in New Pressure
    not_stop_loop = True
#     while (button2_not_pushed()):
    while (not_stop_loop):
        new_pressure = int(input("Enter desired Pressure:\n"))
        new_alt = calc_altitude(pressure_hpa, new_pressure)
        if debug:
            print(f"{new_pressure=}")
            print(f"{altitude_m=}")
            print(f"{new_alt=}")
        update_numbers(new_alt, new_pressure, 0)
        not_stop_loop = False
        
    update_numbers(new_alt, new_pressure, 0)
        
    # TODO update settings for main loop, now we just see them here
    #return
    zzz(10)
    return error


# =========================  startup code =========================
show_env = True
set_known = False
buzzer_sound = True
metric = True
debug = False

interrupt_1_flag = 0
interrupt_2_flag = 0
debounce_1_time = 0
debounce_2_time = 0

sea_level_pressure_hpa = PDX_SLP_1013
sea_level_pressure_hpa = 1002.6
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

# at start display artcar image at top of oled
oled.fill(0)
oled.blit(FrameBuffer(bitmap_artcar_image_back, 56, 15, MONO_HLSB), 22, 36)
oled.show()

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
        if interrupt_1_flag == 1:
            interrupt_1_flag = 0
            if debug: print("button 1 Interrupt Detected: in/cm")
            metric = not metric  # Toggle between metric and imperial units

        # Button 2: rear sensors or front sensors
        if interrupt_2_flag == 1:
            interrupt_2_flag = 0
            if debug: print("button 2 Interrupt Detected: input known values")
            error = input_known_values (True)
            if error:
                print(f"Error setting known values: {error}")      

        # EVERY 3 SECONDS, calc speed of sound based on
        #  * BME680 temp & humidity (189ms duration)

        if first_run or elapsed_time > 3000:
            time_since_last_temp_update = time.ticks_ms()

            # check for over temperature onboard pico
            temp = onboard_temperature()
            if temp > OVER_TEMP_WARNING:
                print(f"WARNING: onboard Pico 2 temp = {temp:.1f}C")
            
            temp_c, humidity, pressure_hpa, iaq, altitude_m, error = bme_temp_humid_hpa_iaq_alt(sea_level_pressure_hpa)        
            if error:
                print(f"No Inside Temp: {error}")
            else:
                if debug: print(f"{temp_c=:.2f}")
                temp_f = (temp_c * 9.0 / 5.0) + 32.0

            first_run = False


        if show_env:
            display_environment(buzzer_sound)
        else:
            print("... show_env = {show_env}")

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

