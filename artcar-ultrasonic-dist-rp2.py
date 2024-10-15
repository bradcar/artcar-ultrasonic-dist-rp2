from machine import Pin, Timer
import machine
import dht
import time

dht_pin = machine.Pin(2)
led = Pin(25, Pin.OUT)

timecall = Timer()


def tick(timer):
    global led
    led.toggle()


timecall.init(freq=2.5, mode=Timer.PERIODIC, callback=tick)
dht_sensor = dht.DHT22(dht_pin)

while True:
    try:
        dht_sensor.measure()
        tempC = dht_sensor.temperature()
        humidity = dht_sensor.humidity()
        print(f"Temp °C : {tempC:.2f}")
        print(f"Humidity: {humidity:.2f}%")
    except Exception as e:
        print("Error reading DHT22:", str(e))
    time.sleep(0.1)
