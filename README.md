# lr-openauto-hat
RPi hat suitable for HUDIY. I used it in Land Rover Discovery II.

![PCB](PCB.png)

Functions:
 - self power control with latch-up
 - steering wheel resistance button read
 - power control of RPi
 - rotary encoder (volume)
 - light sensor (control backlight/da/night mode)
 - heater power on by timing with RTC (NOT IMPLEMENTED IN HUDIY YET)
 - webasto RX/TX interface (NOT TESTED YET)
 - connector for I2C gyroscope for inclinometer (NOT IMPLEMENTED IN HUDIY YET)

Python code is used for communicating between HUDIY and arduino.
