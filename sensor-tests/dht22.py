# Raspberry Pi Pico 2 project - ArtCar ultrasonic - Oct 2024
#
# dht22 -temp & humidity - calculate the speed of sound based on temp/humidity
# by bradcar

from machine import Pin
import machine
import dht
from math import sqrt
from os import uname
from sys import implementation
from time import sleep as zzz

# default Speed of sound at 20C and 70% humidity
SPEED_SOUND_20C_70H = 343.294

# === PINS ===
dht_pin = machine.Pin(2)
led = Pin(25, Pin.OUT)

dht_sensor = dht.DHT22(dht_pin)

# functions that can be used in other codes
# #
# def calc_speed_sound(celsius, percent_humidity):
#     """
#     calc to update speed of sound based on temp & humidity from the DHT22 sensor
# 
#     :param celsius: temp in Celsius
#     :param percent_humidity: % humidity
#     :returns: speed meter/sec, error string
#     """
#     if celsius > 0 and percent_humidity > 0:
#         # Speed sound with temp correction: (20.05 * sqrt(273.16 + temp_c))
#         # online temp/humid calc:
#         #   http://resource.npl.co.uk/acoustics/techguides/speedair/
#         # created spreadsheet of diffs between above temp formula and online temp/humid
#         # did a 2d linear fit to create my own correction, error is now +/-0.07%
#         # valid for 0C to 30C temp & 75 to 102 kPa pressure
#         meter_per_sec = (20.05 * sqrt(273.16 + celsius)) \
#                         + (0.0006545 * percent_humidity + 0.00475) * celsius \
#                         + (0.001057 * percent_humidity + 0.07121)
#         return meter_per_sec, None
#     else:
#         return None, f"ERROR_INVALID_SOUND_SPEED:temp={celsius}C,humidity={percent_humidity}%"
# #    
# def dht_temp_humidity():
#     """
#     read temp & humidity from the DHT22 sensor, measurement takes ~271ms
# 
#     :returns: celsius, percent_humidity, error string
#     """
#     try:
#         dht_sensor.measure()
#         celsius = dht_sensor.temperature()
#         percent_humidity = dht_sensor.humidity()
#         if debug:
#             print(f"DHT Temp °C : {temp_c:.2f}")
#             print(f"DHT Temp °F : {temp_f:.2f}")
#             print(f"DHT Humidity: {humidity:.2f}%")
#     except Exception as e:
#         return None, None, "ERROR_TEMP_HUMID:" + str(e)
#     return celsius, percent_humidity, None

# startup code
print("Starting...")
print("====================================")
print(implementation[0], uname()[3],
      "\nrun on", uname()[4])
print("====================================")
print(f"Default Speed Sound: {SPEED_SOUND_20C_70H:.1f} m/s\n")


# init variables
temp_c = 0.0
temp_f = 0.0
humidity = 0.0
speed_sound = SPEED_SOUND_20C_70H

# main loop
while True:
    try:
        dht_sensor.measure()
        temp_c = dht_sensor.temperature()
        temp_f = (temp_c * 9.0 / 5.0) + 32.0
        humidity = dht_sensor.humidity()
        
        print(f"Temp °C = {temp_c:.2f}C")
        print(f"Temp °F = {temp_f:.2f}F")
        print(f"Humidity= {humidity:.2f}%")
        
        # Speed sound with temp correction: (20.05 * sqrt(273.16 + temp_c))
        speed_sound = (20.05 * sqrt(273.16 + temp_c))
        print(f"Speed Sound with temp correction            : {speed_sound:.4f} m/s")

        # Speed sound with temp correction: (20.05 * sqrt(273.16 + temp_c))
        # online temp/humid calc: http://resource.npl.co.uk/acoustics/techguides/speedair/
        # created spreadsheet of diffs between online temp/humid and temp formula
        # did a 2d linear fit to create my own correction, error is now +/-0.07%
        # valid for 0C to 30C temp & 75 to 102 kPa pressure
        speed_sound = (20.05 * sqrt(273.16 + temp_c)) \
                      + (0.0006545 * humidity + 0.00475) * temp_c \
                      + (0.001057 * humidity + 0.07121)
        print(f"Speed Sound with temp & humidity correction : {speed_sound:.4f} m/s\n")
        
    except Exception as e:
        print("Error reading DHT22:", str(e))
    
    # Every loop do this
    led.toggle()
    zzz(1)
    
