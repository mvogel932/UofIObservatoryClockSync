# UofIObservatoryClockSync
Self-winding clock synchronizer with accurate timebase

General info:

Master clock provides a one-second long pulse once per hour.  It connects the positive power lead to a terminal for that one second.
So what this clock sync circuit needs to do is close a relay for one second once an hour, exactly at 59 min 59 seconds past the hour,
ending exactly at the hour.

Per email from Mike Svec on 5/7/23, the self-winding clock uses 3V power.   Bruce Hannon added a transformer on the inside to power
the rewind of the self-winding clock.

Based on my research, it appears the sync coil on the self-winding clock should have a DC resistance of about 6 ohms.  So it will
draw about 0.5 amps at 3 volts.


Self-winding clock synchronizer:

Documentation/software:  https://github.com/mvogel932/UofIObservatoryClockSync

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

Power input(s) to sync circuit:  Wall transformer, charges a built-in lithium-ion battery.

Output signal to clock:   1 second pulse, 3V, 0.5 amps to power sync solenoid

Source of accurate time:  Orignially was going to use NTP via WiFi, but could not get WiFi access in the Observatory, so going to use GPS
module to get accurate time.

Software:  Arduino running on ESP8266.  See https://github.com/esp8266/Arduino
Arduino sketch:  Custom software written by Matt Vogel
Arduino GPS library:  TBD
Arduino NTP library:  NTPClient by Fabrice Weinberg, version 3.2.1 (no longer needed now that we are using GPS)


Status:

5/20/2023
Got Adafruit Feather HUZZAH w/ESP8266 WiFi board up and running with Arduino IDE 2.1.0.  Was able to connect to the board via micro USB cable
and run Arduino sketches.  Loaded sketch_may20b.ino, which connects to my home WiFi and then connects to an NTP server, gets the current time,
and prints out the day and time once a second.

7/8/2023
Got clock synchronizer circuit hooked up to the self-winding clock in the Observatory.  Got the synchronizer working.  The 3D printed case is a
little too small, so need to make it a little bigger so the lithium-ion battery fits better.


To Do:
Make 3D printed case bigger so the lithium-ion battery fits
Add a power switch so the synchronizer can be shut off
Add GPS module as an accurate time source
