from machine import Pin, Timer
import machine
import dht
import time
import math

dht_pin = machine.Pin(2)
led = Pin(25, Pin.OUT)

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
        # I did a 2d linear fit with formula above to add correction, error is +/-0.07% in range
        # valid over the temp range 0 to 30C temp (273.15 - 303.15 K) and for the pressure range 75 - 102 kPa
        soundSpeed = (20.05 * math.sqrt(273.16 + tempC)) + (0.0006545 * humidity + 0.00475) * tempC + (0.001057 * humidity + 0.07121)
        #
        if debug:
            print(f"Temp °C : {tempC:.2f}")
            print(f"Temp °F : {tempF:.2f}")
            print(f"Humidity: {humidity:.2f}%")
            print(f"Speed Sound: {soundSpeed:.1f} m/s")
    except Exception as e:
        print("Error reading DHT22:", str(e))
    #
    time.sleep(0.1)

    