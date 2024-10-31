# bme280 sensor test code
#
# the BME280.py library must be installed on Raspberry Pi in order for this to work
# library source:
#    https://github.com/RuiSantosdotme/ESP-MicroPython/blob/master/code/WiFi/HTTP_Client_IFTTT_BME280/BME280.py
#    However it returns strings, so turned this into floats in my version
#
# PDX sea level pressure every 2 mins
# by bradcar

# ****** UNTESTED  *****  by bradcar  TODO when I get the sensor

from machine import Pin, I2C
from time import sleep
import BME280

# Initialize I2C communication
i2c = I2C(id=1,  sda=Pin(26), scl=Pin(27), freq=10000)


def calc_sea_level_pressure(hpa, meters):
    sea_level_pressure_hpa = hpa / (1.0 - (meters / 44330.0)) ** 5.255

    return sea_level_pressure_hpa


while True:
    try:       
        # change this to match the pressure (hPa) at sea level, use nearest airport
        # https://community.bosch-sensortec.com/t5/Question-and-answers/How-to-calculate-the-altitude-from-the-pressure-sensor-data/qaq-p/5702
        # https://www.weather.gov/wrh/timeseries?site=KPDX
        # hPa = mB
        sea_level_pressure_hpa = 1013.20
        
        # Initialize BME280 sensor
        bme = BME280.BME280(i2c=i2c)
        
        # Read sensor data
        temperature_c = bme.temperature
        humidity = bme.humidity
        pressure_hpa = bme.pressure        
        
        # Convert temperature to Fahrenheit
        temperature_f = temperature_c * (9.0/5.0) + 32.0
        
        # calculate altitude
        altitude_m = 44330.0 * (1.0 - (pressure_hpa/sea_level_pressure_hpa)**(1.0/5.255) )
        altitude_f = altitude_m * 3.28084
        
        # Print sensor readings
        print(f"{sea_level_pressure_hpa=}")
        print(f"Temp C = {temperature_c:.1f} C")
        print(f"Temp F = {temperature_f:.1f} F")
        print(f"Humidity = {humidity:.1f} %")
        print(f"Pressure= {pressure_hpa:.2f} hPa")
        print(f"Altitude= {altitude_m:.1f} m")
        print(f"Altitude= {altitude_f:.1f} f")
        
        # from the Altitude, calc sea level pressure
        predict_slp = calc_sea_level_pressure(pressure_hpa, altitude_m)
        print(f"Sea Level Pressure= {predict_slp:.1f} hPa\n")
        
    except Exception as e:
        # Handle any exceptions during sensor reading
        print("An error occurred:", e)

    sleep(5)
