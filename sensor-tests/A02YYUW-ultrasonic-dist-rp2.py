# Raspberry Pi Pico 2 project - A02YYUW UART ultrasonic - Oct 2024
#
# Uses 3 ultrasonic sensors, uses oled SPI display to show measured
# temp & humid and uses them to calculate speeed of sound for accurate
# distance measurement.
#
# based on Brad's Arduino UNO and 128x64 OLED Display for rear&front parking sensor
#    - speed of sound with temp & humidity correction
#    - front- & Rear-facing changed with button #2

#    - in/cm F/C changed with button #1
#    - buttons debounced with efficient rp2 interrupts -- nice!
#    - ssd1309 SDI or I2C code (sw is ssd1306)
#
#    Raspberry Pi GitHub: https://github.com/bradcar/artcar-ultrasonic-dist-rp2
#    # by bradcar
#
# https://micropython.org/download/RPI_PICO2/ for latest .uf2 preview
# https://docs.micropython.org/en/latest/esp8266/tutorial/ssd1306.html
# pycharm stubs at: https://micropython-stubs.readthedocs.io/en/main/packages.html#mp-packages
#
# TODOs
#  * expand from one a02yyuw waterproof UART sensor to 3, or switch to a02yyuw PWM
#  * add button #3 for switching between showing car on oled or

import time
from math import sqrt
from os import uname
from sys import implementation
from time import sleep as zzz

import dht
import ds18x20
import machine
import onewire
import utime
# import RPi.GPIO as GPIO
import rp2
from framebuf import FrameBuffer, MONO_HLSB
from machine import Pin
# ic2
# from machine import I2C
# from ssd1306 import SSD1306_I2C
from ssd1306 import SSD1306_SPI

# Constants for  setup, for >3 sensors need rewrite
DISP_WIDTH = 128
DISP_HEIGHT = 64

# Only 1 UART Ultrasonic can be directly connected
NUMBER_OF_SENSORS = 1


# === PINS ===
# internal pins
on_pico_temp = machine.ADC(4)  #ADC4 has no external pins

# external pins
uart0 = machine.UART(0, 115200, tx=Pin(0), rx=Pin(1))
dht_pin = machine.Pin(2)
button_1 = Pin(5, Pin.IN, Pin.PULL_UP)  # interrupt cm/in button pins
button_2 = Pin(6, Pin.IN, Pin.PULL_UP)  # interrupt rear/front button pins

# https://www.tomshardware.com/how-to/oled-display-raspberry-pi-pico
# i2c=I2C(0,sda=Pin(12), scl=Pin(13), freq=400000)
# oled = SSD1306_I2C(DISP_WIDTH, DISP_HEIGHT, i2c)

# ssd1306 SDI SW setup for ssd1309 SDI
cs = machine.Pin(9)  # dummy (any un-used pin), no connection
# scl (SCLK) gp10
# SDA (MOSI) gp11
res = machine.Pin(12)  # RES (RST)  gp12
dc = machine.Pin(13)  # DC         gp13

uart1 = machine.UART(1, 9600, tx=Pin(20), rx=Pin(21))
led = Pin(25, Pin.OUT)
ds_pin = machine.Pin(28)
buzzer = Pin(27, Pin.OUT)

dht_sensor = dht.DHT22(dht_pin)
ds_sensor = ds18x20.DS18X20(onewire.OneWire(ds_pin))
oled_spi = machine.SPI(1)
# print(f"oled_spi:{oled_spi}")
oled = SSD1306_SPI(DISP_WIDTH, DISP_HEIGHT, oled_spi, dc, res, cs)


DWELL_ADDED_MS = 100
SPEED_SOUND_20C_70H = 343.294
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


def dht_temp_humidity():
    """
    read temp & humidity from the DHT22 sensor, , measurement takes ~271ms

    :returns: celsius, percent_humidity, error string
    """
    try:
        dht_sensor.measure()
        celsius = dht_sensor.temperature()
        percent_humidity = dht_sensor.humidity()
        if debug:
            print(f"DHT Temp °C : {temp_c:.2f}")
            print(f"DHT Temp °F : {temp_f:.2f}")
            print(f"DHT Humidity: {humidity:.2f}%")
    except Exception as e:
        return None, None, "ERROR_TEMP_HUMID:" + str(e)
    return celsius, percent_humidity, None


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
    Get the outside temp DS sensor (waterproof DS18B20), measurement takes ~6ms
    Because we must call ds_sensor.convert_temp() at least 750ms before read data,
    we will call outside_temp_ds_init() in setup (it wraps this function)

    Notice at the end of this function we call ds_sensor.convert_temp() in order to
    have enough time before we read again.
    We ONLY call this function once every 3 seconds, which means there will
    always be at least 3 seconds between ds_sensor.convert_temp() and ds_sensor.read_temp(rom)

    :returns: temp Celsius & error string
    """
    celsius = False
    # NOTE: if this routine is called to both setup and test temp, need to uncomment this code
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
        # online temp/humid calc: http://resource.npl.co.uk/acoustics/techguides/speedair/
        # created spreadsheet of diffs between above temp formula and online temp/humid
        # did a 2d linear fit to create my own correction, error is now +/-0.07%
        # valid for 0C to 30C temp & 75 to 102 kPa pressure
        meter_per_sec = (20.05 * sqrt(273.16 + celsius)) \
                        + (0.0006545 * percent_humidity + 0.00475) * celsius \
                        + (0.001057 * percent_humidity + 0.07121)
        return meter_per_sec, None
    else:
        return None, f"ERROR_INVALID_SOUND_SPEED:temp={celsius},humidity={percent_humidity}"


def calculate_checksum(data_buffer):
    """
    Function to calculate checksum, make sure only 1 byte returned
    """
    return (data_buffer[0] + data_buffer[1] + data_buffer[2]) & 0x00ff


def ultrasonic_distance_uart():
    """
    Receive ultrasonic distance results on UART, automatic temp correction in A02YYUW sensor

    :returns: distance in cm (float), The sensor returns mm in integer

    The four bytes of data sent by the sensor:
    Byte 0 – Header – always a value of xFF, start of a block of data.
    Byte 1 – Data 1 – high end of the 16-bit data, with the distance value in millimeters.
    Byte 2 – Data 0 – low end of the 16-bit data.
    Byte 3 – Checksum – addition of previous 3 (B0 + B1 + B2). Only lower 8-bits are held here.

    A02YYUW: 3.3v to 5v, +/- 0.2cm, 3cm to 450cm (1.2" to 177") $16
     - only outputs UART serial data, RS485 output, 8mA, connector: JST PH 4-pin
     - Sensor wires (Red=Vcc, Blk=Gnd, Yellow=tx1=GP20, White=rx1=GP21)
     - RX pin controls the data output.
       - HIGH or not connected (internal pulled up) then results every 300ms.
       - RX pin held LOW then results every 100ms. (less accurate)
       -
    JSN-SR04T: +/1 1.0mc, 20cm to 600cm, $10,
     - Mode 0(default): must measure time, use ultrasonic_distance_pwm() code instead
     - Mode 1: outputs serial data UART, measurement every 200ms. You read the data on the Echo pin
     - Mode 2: need to send #55, then it outputs serial data as UART and one reads from Echo pin
    https://dronebotworkshop.com/waterproof-ultrasonic/
    https://www.amazon.com/Ultrasonic-Distance-Controlled-Detector-Waterproof/dp/B0CFFTS71Y/

    :returns: cm distance and error string
    """
    # REQUIRED write !!!! to get uart1 to send data, two writes then delay not needed
    uart1.write(b'\xff')
    uart1.write(b'\xff')
    #    time.sleep_ms(100)  # Small delay to ensure complete data packet reception

    if uart1.any():
        # Read the first byte to check for the packet header (0xFF)
        if uart1.read(1) == b'\xff':
            # Create an array to store the data buffer
            data_buffer = bytearray(4)
            data_buffer[0] = 0xff

            # Read the next 3 bytes
            data_buffer[1:4] = uart1.read(3)
            if debug: print(f"data_buffer = {data_buffer}")

            # Compute checksum
            checksum = calculate_checksum(data_buffer)
            if debug: print(f"checksum={checksum:x}")

            # Verify if the checksum matches the last byte in the packet
            if data_buffer[3] == checksum:
                # Calculate distance in mm from the data bytes
                mm_distance = (data_buffer[1] << 8) + data_buffer[2]
                if debug: print(f"distance={mm_distance} mm\n")
                return mm_distance / 10.0, None

    # if here, then error state
    return None, f"ULTRASONIC_ERROR: UART Sensor- no results"


def display_environment(dist, buzz):
    """
    display just environment readings & car image(for fun)
    No need to oled.fill(0) before or oled.show() after call

    param:dist:  distance if dist = -1.0 then display error
    param:buzz:  flag if should have distance warning sound
    """
    oled.fill(0)
    if dist < 69.0 and buzz:
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
        oled.text(f"Sound= {speed_sound:.1f} m/s", 0, 24)
        if dist:
            oled.text(f"Dist= {dist:.0f}cm", 0, 40)
        else:
            oled.text(f"No ultrasonic", 0, 40)
    else:
        oled.text(f"Sound= {speed_sound * 3.28084:.0f} ft/s", 0, 24)
        if dist:
            oled.text(f"Dist= {dist / 2.54:.1f}in", 0, 40)
        else:
            oled.text(f"No ultrasonic", 0, 40)
    oled.text(f"Alt = xx,xxx ft", 0, 55)

    oled.show()
    return


# =========================  startup code =========================
buzzer_sound = True
metric = False
dht_error = False
ds_error = False
debug = False

interrupt_1_flag = 0
interrupt_2_flag = 0
debounce_1_time = 0
debounce_2_time = 0

speed_sound = SPEED_SOUND_20C_70H
temp_f = None
temp_c = None
humidity = None

print("Starting...")
print("====================================")
print(implementation[0], uname()[3],
      "\nrun on", uname()[4])
print(uart1)
temp = onboard_temperature()
print(f"onboard Pico 2 temp = {temp:.1f}C")
print("====================================")
print(f"oled_spi:{oled_spi}")

# roms will be a list of sensors on same GPIO pin
roms = ds_sensor.scan()
print('Found DS devices, roms: ', roms)

# call first time to set up onewire temp, we don't measure temp in this routine
error = outside_temp_ds_init()
if error:
    print(error)
else:
    print("Onewire outside temp found")

# check status of DHT sensor, do not keep any measurements
_, _, error = dht_temp_humidity()
if error:
    oled.text("Error DHT22", 0, 24)
    oled.text(str(error), 0, 36)
    oled.show()
    temp_c = None
    humidity = None

working_ultrasonics = [1]
nonworking_ultrasonics = [0, 2]
print(f"Working PWM Ultrasonic sensors: {working_ultrasonics}")
print(f"Non-working PWM Ultrasonic sensors: {nonworking_ultrasonics}")

# Check UART Ultrasonics
_, _ = ultrasonic_distance_uart()
zzz(.5)
_, error = ultrasonic_distance_uart()
if not error: print("UART ultrasonic [1] found")


if buzzer_sound: buzzer.on()
zzz(0.1)
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
            if debug: print("button 2 Interrupt Detected: rear/front")
            rear = not rear  # Toggle between rear /front

        # EVERY 3 SECONDS, calc speed of sound based on
        # * onewire outside temp (16ms duration)
        # * dht22 temp & humidity (271ms duration)
        # Note: Temp & humidity correction is for the speed of sound
        # speed of sound going from 0C to 30C goes from 331.48 m/s to 351.24 m/s (~ 6%)
        # speed of sound at 30C goes with a humidity of 0% to 90% goes from 349.38 m/s to 351.24 m/s (~ 0.53%)
        # ...humidity effect is negligible, but I had a dht22 which does both, so why not :)
        if first_run or elapsed_time > 3000:
            time_since_last_temp_update = time.ticks_ms()

            start = time.ticks_ms()
            # check for over temperature onboard pico
            temp = onboard_temperature()
            if temp > OVER_TEMP_WARNING: print(f"WARNING: onboard Pico 2 temp = {temp:.1f}C")

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
            inside_temp_c, humidity, error = dht_temp_humidity()
            if error:
                print(f"No Inside Temp: {error}")
            else:
                if debug: print(f"Inside temp C = {inside_temp_c:.2f}")
            elapsed_time = time.ticks_diff(time.ticks_ms(), start)
            print(f"inside temp time = {elapsed_time}\n")

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

# one uart sensor
#         working_ultrasonics = [1]
#         nonworking_ultrasonics = [0, 2]
        uart_cm, error = ultrasonic_distance_uart()
        if error:
            print(error)

        display_environment(uart_cm, buzzer_sound)

        # Every loop do this
        led.toggle()
        time.sleep_ms(DWELL_ADDED_MS)
        loop_elapsed_time = time.ticks_diff(time.ticks_ms(), loop_time)
#         print(f"loop_elapsed_time with {DWELL_ADDED_MS}ms delay={loop_elapsed_time}")

# if control-c at end clean up pico2
except KeyboardInterrupt:
    oled.fill(0)  # turn off oled
    oled.show()  # show on oled
    buzzer.off()
    print("Exit: ctrl-c")
except:
    buzzer.off()
    print ("Other error or exception occurred!")

