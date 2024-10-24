from machine import UART, Pin
from os import uname
from sys import implementation
import time

# A02YYUW Waterproof sensor, UART-version (ex: A0221A, may be others)
# connected with RS-485
#
# Notes:
# * Pico has two UARTs (uart(0), and uart(1)
# * Define uart1 pins for ultrasonic, uart0 typical for REPL so don't use that one
# * Sensor wires (Red=Vcc, Blk=Gnd, Yellow=tx1=GP20, White=rx1=GP21)
# * Evidently need to write a byte each time to get sensor to respond
# * https://docs.micropython.org/en/latest/library/machine.UART.html
# * https://dronebotworkshop.com/waterproof-ultrasonic/  (note diff wire colors on sensor)
#
#   The four bytes of data sent by the sensor:
#     Byte 0 – Header – always a value of xFF, start of a block of data.
#     Byte 1 – Data 1 – high end of the 16-bit data, with the distance value in millimeters.
#     Byte 2 – Data 0 – low end of the 16-bit data.
#     Byte 3 – Checksum – addition of previous 3 (B0 + B1 + B2). Only lower 8-bits transmitted.
#
# DEBUG NEEDED, can't get this to work: "with Rx on the sensor floating, it will output data ever 300ms"

debug =False

uart1 = machine.UART(1, 9600, tx=20, rx=21)
# expect: UART(1, baudrate=9600, bits=8, parity=None, stop=1, tx=20, rx=21, txbuf=256, rxbuf=256, timeout=0, timeout_char=2, invert=None, irq=0)
print(uart1)

# Function to calculate checksum, make sure only 1 byte returned
def calculate_checksum(data_buffer):
    return (data_buffer[0] + data_buffer[1] + data_buffer[2]) & 0x00ff

def ultrasonic_distance_uart():
    """
    :returns: distance in cm (float), The sensor returns mm in integer
    """
    # REQUIRED write !!!! to get uart1 to send data
    uart1.write(b'\xff')
    time.sleep_ms(100)  # Small delay to ensure complete data packet reception
    
    if uart1.any():
        
        if debug: print("uart1: reading bytes")

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
                distance = (data_buffer[1] << 8) + data_buffer[2]
                if debug: print(f"distance={distance} mm\n")
                return distance/10.0
    return None

# startup code
print("Starting...")
print("====================================")
print(implementation[0], uname()[3],
      "\nrun on", uname()[4])
print("====================================")

# A02YYUW seems to need 1 call in setup before use, else get errors the first time
cm = ultrasonic_distance_uart()

# Main loop
while True:
    cm = ultrasonic_distance_uart()
    if cm:
        print(f"Distance= {cm:.1f} cm")
    else:
        print("ERROR A02YYUW: No valid data received.")
    
    time.sleep(1)  # arbitrary One second between measurements
