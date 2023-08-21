# V1.0.0 
## Major Changes 
removed I2C driver install from sx1509::begin()
updated to release 0.9.4 of UncleRus/esp-idf-lib

## Bugfixes
changed master/slave language to client/server per the Modbus consortium where it didn't affect the interfaces
FIXME(?) remove depencency on UncleRus' i2cdev and esp_idf_lib_helpers
* FIXME sx1509.hpp:52:39: warning: comparison is always false due to limited range of 

## Known Issues
* It's probable that the code isn't turning on the back light.

## Troubleshooting
05-1000193 REV A #1, 05-1000219 REV - #2.
* Flashed 1.0.0
* 93-0000222: IO test failed. All LEDs work, but super dim. Display no backlight, only shows solid top line. All buttons respond.
* Flashed 99-0002000 dev version. Everything works
* Re-flash 99-0000222 v 1.0.0. Everything works, except button 4.
* Connect 05-1000219 REV - #1: Current draw 0.10 A @ 29.0V, 0.0 V at RMK connector. LED on I/O connector was shorting to pullup resistor on I/O.
* Removed short and buttons work, and current draw 0.02 A. Display only shows top solid bar.
* Swap in Display A. Same thing. SCL 3.6 V, SDA 3.6V at Display connector and RMK connector.
* Flash 99-0002000. Everything works.
