# artcar-ultrasonic-dist-rp2
Raspberry Pi Pico 2 ultrasonic distance - three sensors, can be front/back-facing and has temp/humidity correction, 2024 version
* It was refactored from my Arduino code: https://github.com/bradcar/artcar-ultrasonic-dist-arduino

Features of this Raspberry Pi Pico 2 code:
* SSD1309 - uses SSD1306, have SDI & I2C code, SDI faster
* button debounce using efficient interrupt code (does not use CPU cycles & sleep, yay!)
* DHT22 - for temp & humidity
* uses ssd1306 print & blit(FrameBuffer) of bitmap images
* up to 3x Ultrasonic sensors are polled for distances (PWM - timed for distances)
* TODO: up to 3x Ultrasonic sensors are polled for distances (UART - read distances)

My work is based on UPIR's GREAT work: https://github.com/upiir/arduino_parking_sensor created by upir, 2022, modified by bradcarlile 2024
* UPIR youtube channel: https://www.youtube.com/upir_upir
* UPIR youtube full video: https://youtu.be/gg08H-6Z1Lo
* UPIR Github: https://github.com/upiir/arduino_parking_sensor
* UPIR full simulation https://wokwi.com/projects/348388602879672914

To see my Art Car, it's the 5th image down in https://en.wikipedia.org/wiki/Art_car  , the one with the caption: "Unofficial BMW Art Car by Tom Cramer..."

Useful sites:
* image2cpp (convert images into C code): https://javl.github.io/image2cpp/
* Going to use: Foriot's TXS0108E 8 Channel Level Converter Module Support 3.3V to 5V to protect Pico from 5v signals for Ultrasonic accuracy and SSD1309 power
  * SSD1309 SDI is fast, so I need one that can keep up https://www.amazon.com/gp/product/B0CFL9KN7L
* Powering Pico: https://www.youtube.com/watch?v=3PH9jzRsb5E -- Useful because I'm going to need 3.3v for Pico, 5v for ultrasonics, and power in my car.
  * get Buck converter 12v to 5v (usb-b to micro-usb)
  * Also add MosFET protection
    * RP recommended DMG-2305ux, but this is surface mount MosFET
    * investigating RLB8721, that way can hook laptop up in car to update SW

Other useful sites (but not used in this code):
* MicroPython Fonts:  https://github.com/peterhinch/micropython-font-to-py/tree/master -- Didn't use it for this project

Note: Temp & humidity correction for the speed of sound, when using
* speed of sound going from 0C to 30C goes from 331.48 m/s to 351.24 m/s (~ 6%)
* speed of sound at 30C goes with a humidity of 0% to 90% goes from 349.38 m/s to 351.24 m/s (~ 0.53%)
* ...humidity effect is negligible, but I had a dht22 which does both, so why not :)
