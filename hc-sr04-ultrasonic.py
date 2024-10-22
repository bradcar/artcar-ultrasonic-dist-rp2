# Raspberry Pi Pico 2 project - ArtCar ultrasonic - Oct 2024
#
# HC-SR04 ultrasonic sensor in subroutine
# by bradcar

from machine import Pin
import utime
from os import uname
from sys import implementation
from time import sleep as zzz

MIN_DIST = 2.0
MAX_DIST = 100.0
SPEED_SOUND_20C_70H = 343.294

# === PINS ===
trigger = Pin(3, Pin.OUT)
echo = Pin(4, Pin.IN)

led = Pin(25, Pin.OUT)
    
def ultrasonic_distance_pwm(timeout=50000):
    """
    Get ultrasonic distance from a sensor where ping and measure with a timeout.
    
    HC-SR04: most Send a 10uS high to trigger
    JSN-SR04T: Send a 20uS high to trigger, instead of 10
    A02YYUW: only outputs UART serial data, instead use ultrasonic_distance_uart(i) 
    https://dronebotworkshop.com/waterproof-ultrasonic/
    
    :param timeout: Max usecs to wait for the echo signal return. default 50ms.
    :returns: cm distance or an error value.
    """
    trigger.low()
    utime.sleep_us(2)
    trigger.high()
    utime.sleep_us(20)  # JSN-SR04T: best with 20uS high to trigger instead of 10 
    trigger.low()

    # Wait for echo to go high, with timeout
    start = utime.ticks_us()
    while echo.value() == 0:
        if utime.ticks_diff(utime.ticks_us(), start) > timeout:
            print(f"Error: Sensor - too close or malfunction.\n")
            return 0.0  # Return max distance or an error

    signal_off = utime.ticks_us()

    # Wait for echo to go low, with timeout
    start = utime.ticks_us()
    while echo.value() == 1:
        if utime.ticks_diff(utime.ticks_us(), start) > timeout:
            print(f"Error: Sensor - no signal returned, too far.\n")
            return 300.0  # Return max distance

    signal_on = utime.ticks_us()

    # Calculate cm distance
    return utime.ticks_diff(signal_on, signal_off) * (speed_sound / 20000.0)

# startup code
print("Starting...")
print("====================================")
print(implementation[0], uname()[3],
      "\nrun on", uname()[4])
print("====================================")
print(f"Default Speed Sound: {SPEED_SOUND_20C_70H:.1f} m/s\n")

speed_sound = SPEED_SOUND_20C_70H

# main loop
while True:   
    #get distance from ultrasonic sensor, 30ms round trip maz ia 514cm or 202in
    # sensors: left front/rear = 0, middle=1  right front/rear =2
    cm = ultrasonic_distance_pwm(timeout=30000)
    inch = cm/2.54
    
    if cm != -1.0:
        print(f"Dist= {cm:.3f} cm")
        print(f"Dist= {inch:.3f} in\n")
    else:
        print(f"No ultrasonic\n")    

    # Every loop do this
    led.toggle()
    zzz(1)
