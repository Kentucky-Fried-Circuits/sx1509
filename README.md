sx1509 IO Expander ESP-IDF Library
========================================
ported from
[![SparkFun SX1509 IO Expander Breakout](https://cdn.sparkfun.com//assets/parts/1/0/9/5/6/13601-01.jpg)](https://www.sparkfun.com/products/13601).
Uses esp-idf-lib for thread-safe operations. 
Not every operation is thread-safe yet.

ESP-IDF library for the sx1509 16-I/O expander. Capable of driving LEDs - with blink, and breathe functions - or monitoring up to 64 buttons in an 8x8 array.

Are you low on I/O? No problem! The sx1509 is a 16-channel GPIO expander with an I2C interface – that means with just two wires, your microcontroller can interface with 16 fully configurable digital input/output pins. But the SX1509 can do so much more than just simple digital pin control. It can produce PWM signals, so you can dim LEDs. It can be set to blink or even breathe pins at varying rates. This breakout is similar to a multiplexer or "mux," in that it allows you to get more IO from less pins. And, with a built-in keypad engine, it can interface with up to 64 buttons set up in an 8x8 matrix.

Since the I/O banks can operate between 1.2V and 3.6V (5.5V tolerant) independent of both the core and each other, this device can also work as a level-shifter. 

Repository Contents
-------------------

* ** - Source files for the library (.cpp, .h).
* **keywords.txt** - Keywords from this library that would be highlighted in the Arduino IDE, but may not work in Platform IO.
* **library.properties** - General library properties for the Arduino package manager, which is probably meaningless to Platform IO.

Documentation
--------------

* **[SparkFun SX1509 Breakout Board Hookup Guide](https://learn.sparkfun.com/tutorials/sx1509-io-expander-breakout-hookup-guide)** - SparkFun tutorial demonstrating how to hook up the SX1509 Breakout and use the original library.

Version History
---------------
see RELEASE.md

License Information
-------------------
see LICENSE.txt
