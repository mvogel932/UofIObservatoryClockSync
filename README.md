# UofIObservatoryClockSync
Self-winding clock synchronizer with accurate timebase

Self-winding clock synchronizer:

Documentation/software:  https://github.com/mvogel932/UofIObservatoryClockSync



Status:

5/20/2023
Got Adafruit Feather HUZZAH w/ESP8266 WiFi board up and running with Arduino IDE 2.1.0.  Was able to connect to the board via micro USB cable and run Arduino sketches.  Loaded sketch_may20b.ino, which connects to my home WiFi and then connects to an NTP server, gets the current time, and prints out the day and time once a second.

7/8/2023
Got clock synchronizer circuit (Rev 1 Hardware) hooked up to the self-winding clock in the Observatory. Got the synchronizer working. The 3D printed case is a little too small, so need to make it a little bigger so the lithium-ion battery fits better.

2/5/2024
Since I have been unable to get WiFi access in the Observatory, decided to switch to using GPS for accurate time source.  The existing Rev 1 hardware with the ESP8266 processor is not compatible with the Adafruit GPS module, so I am switching to the ESP32-S2 Reverse TFT Feather development board from Adafruit.  Ordered it along with the GPS module.


To Do:
- Make new 3D-printed case for new rev 2 hardware
- Add power switch for battery so device can be turned off
- Write updated software for rev 2 hardware and GPS module



General info:

Master clock provides a one-second long pulse once per hour.  It connects the positive power lead to a terminal for that one second.  So what the clock sync circuit needs to do is close a relay for one second once an hour, exactly at 59 min 59 seconds past the hour, ending exactly at the hour.

Per email from Mike Svec on 5/7/23, the self-winding clock uses 3V power.   Bruce Hannon added a transformer on the inside to power the rewind.

Based on my research, it appears the sync coil on the self-winding clock should have a DC resistance of about 6 ohms.  So it will draw about 0.5 amps at 3 volts.



Rev 2 Hardware (as of 2/5/2024):

Controller board:  Adafruit ESP32-S2 Reverse TFT Feather
From <https://www.adafruit.com/product/5345> 

It's basically our ESP32-S2 TFT Feather but with the 240x135 color TFT display on the back-side not the front-side. That makes it great for panel-mounted projects, particularly since we've also got some space for 3 buttons to go along. It's like an all-in-one display interface dev board, powered by the fantastic ESP32-S2 WIFI module.
This feather comes with native USB and 4 MB flash + 2 MB of PSRAM, so it is perfect for use with CircuitPython or Arduino with low-cost WiFi. Native USB means it can act like a keyboard or a disk drive. WiFi means it's awesome for IoT projects. And Feather means it works with the large community of Feather Wings for expandability.
The ESP32-S2 is a highly-integrated, low-power, 2.4 GHz Wi-Fi System-on-Chip (SoC) solution that now has built-in native USB as well as some other interesting new technologies like Time of Flight distance measurements. With its state-of-the-art power and RF performance, this SoC is an ideal choice for a wide variety of application scenarios relating to the Internet of Things (IoT), wearable electronics, and smart homes.
Please note the Feather ESP32-S2 has a single-core 240 MHz chip, so it won't be as fast as ESP32's with dual-core. Also, there is no Bluetooth support. However, we are super excited about the ESP32-S2's native USB which unlocks a lot of capabilities for advanced interfacing! This ESP32-S2 mini-module we are using on the Feather comes with 4 MB flash and 2 MB PSRAM so you can buffer massive JSON files for parsing!
The color TFT is connected to the SPI pins, and uses additional pins for control that are not exposed to the breakout pads. It's the same display as you see here, with 240x135 pixels and is IPS so you get bright color at any angle. The backlight is also connected to a separate pin so you can PWM the backlight up and down as desired.
For low power usage, the Feather has a second RT9080 regulator. The regulator is controlled with a GPIO pin on the enable line and can shut off power to the Stemma QT port and TFT. There is also a separate power pin for the NeoPixel that can be used to disable it for even lower quiescent power. With everything off and in deep sleep mode, the TFT feather uses about 40uA of current.
Features:
	• ESP32-S2 240MHz Tensilica processor - the next generation of ESP32, now with native USB so it can act like a keyboard/mouse, MIDI device, disk drive, etc!
	• Mini module has FCC/CE certification and comes with 4 MByte of Flash and 2 MByte of PSRAM - you can have huge data buffers
	• Color 1.14" IPS TFT with 240x135 pixels - bright and colorful display with ST7789 chipset that can be viewed at any angle.
	• Three User Tactile buttons - D0, D1, and D2. D0/BOOT0 is also used for entering ROM bootloader mode if necessary.
	• Power options - USB type C or Lipoly battery
	• Built-in battery charging when powered over USB-C
	• LiPoly battery monitor - MAX17048 chip actively monitors your battery for voltage and state of charge / percentage reporting over I2C
	• Reset and DFU (BOOT0) buttons to get into the ROM bootloader (which is a USB serial port so you don't need a separate cable!)
	• Serial debug output pin (optional, for checking the hardware serial debug console)
	• STEMMA QT connector for I2C devices, with switchable power, so you can go into low power mode.
	• On/Charge/User LEDs + status NeoPixel with pin-controlled power for low power usage
	• Low Power friendly! In deep sleep mode, we can get down to 40~50uA of current draw from the Lipoly connection. Quiescent current is from the power regulator, ESP32-S2 chip, and Lipoly monitor. Turn off the NeoPixel and external I2C/TFT power for the lowest quiescent current draw.
	• Works with Arduino or CircuitPython

From <https://www.adafruit.com/product/5345> 



Relay board:  Adafruit power relay FeatherWing
https://www.adafruit.com/product/3191
Omron G5LE-14 3 VDC relay
Coil resistance:  22.5 Ohms
Coil current:  133 mA
Must operate voltage: 2.25 VDC
Must release voltage: 0.3 VDC
Operate time:  10 ms
Release time:  5 ms
Inductive load max:  4A at 30 VDC



Power input(s) to sync circuit:  TBD
Output signal to clock:   1 second pulse, 3V, 0.5 amps to power sync solenoid
Source of accurate time:  GPS
Software:  Arduino running on ESP32-S2
Arduino sketch:  Custom software written by Matt Vogel, TBD name, TBD location on github
Arduino GPS library:  Adafruit GPS Library



-------------------------------------------------------------------------------------

OLD INFO FOR REV 1 HARDWARE (prior to 2/5/2024):

Controller board:  Adafruit Feather HUZZAH w.ESP8266  WiFi with stacking headers
https://www.adafruit.com/product/3213
3.3V power supply on the board can supply about 500 mA.  The ESP8266 uses about 250 mA, so other circuits can draw about 250 mA.

Relay board:  Adafruit power relay FeatherWing
https://www.adafruit.com/product/3191
Omron G5LE-14 3 VDC relay
Coil resistance:  22.5 Ohms
Coil current:  133 mA
Must operate voltage: 2.25 VDC
Must release voltage: 0.3 VDC
Operate time:  10 ms
Release time:  5 ms
Inductive load max:  4A at 30 VDC

Power input(s) to sync circuit:  TBD
Output signal to clock:   1 second pulse, 3V, 0.5 amps to power sync solenoid
Source of accurate time:  NTP via WiFi
Software:  Arduino running on ESP8266.  See https://github.com/esp8266/Arduino
Arduino sketch:  Custom software written by Matt Vogel, TBD name, TBD location on github
Arduino NTP library:  NTPClient by Fabrice Weinberg, version 3.2.1

