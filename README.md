# artcar-ultrasonic-dist-rp2
Raspberry Pi Pico 2 ultrasonic distance - three sensors, can be front/back-facing and has temp/humidity correction, 2024 version
* It was refactored from my Arduino code: https://github.com/bradcar/artcar-ultrasonic-dist-arduino

My work is based on UPIR's GREAT work: https://github.com/upiir/arduino_parking_sensor created by upir, 2022, modified by bradcarlile 2024
* UPIR youtube channel: https://www.youtube.com/upir_upir
* UPIR youtube full video: https://youtu.be/gg08H-6Z1Lo
* UPIR Github: https://github.com/upiir/arduino_parking_sensor
* UPIR full simulation https://wokwi.com/projects/348388602879672914

To see my Art Car, it's the 5th image down in https://en.wikipedia.org/wiki/Art_car  , the one with the caption: "Unofficial BMW Art Car by Tom Cramer..."

Other Useful sites:
* image2cpp (convert images into C code): https://javl.github.io/image2cpp/
* MicroPython Fonts:  https://github.com/peterhinch/micropython-font-to-py/tree/master -- Didn't use it for this project
* Powering Pico: https://www.youtube.com/watch?v=3PH9jzRsb5E -- Useful because I'm going to need 3.3v for Pico, 5v for ultrasonics, and power in my car.
* Going to use: Foriot's TXS0108E 8 Channel Level Converter Module Support 3.3V to 5V to protect Pico from 5v signals (Ultrasonic's and SSD1306 SDI is fast so I need one that can keep up) https://www.amazon.com/gp/product/B0CFL9KN7L
