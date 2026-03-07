# FrskySbus

A simple library for parsing Frsky SBUS messages. In order to utilize this library, you will need to invert the serial data coming from the SBUS OUT port. We like to use this SBUS inverter from Ready To Fly Quads: https://www.readytoflyquads.com/rtf-sbus-inverter.

Please refer to the [FrskySbusParse](https://github.com/copperpunk-arduino/frsky-sbus/tree/main/examples/FrskySbusParse) example if you would like to quickly test your hardware setup. It uses the above-mentioned SBUS inverter connected to an SAMD21 Arduino board ([Adafruit QT Py](https://www.adafruit.com/product/4600) pictured) and prints the received channels to the USB Serial port.
<p align="center"><img src="https://static.wixstatic.com/media/07c139_eee0c8345b474a68be78c42cd4d4c1e5~mv2.jpg" width=50%"></p> 
<p align="center"><img src="https://static.wixstatic.com/media/07c139_1a05e7ec24a4435fb8c5396430b2c14a~mv2.jpg" width=50%></p>

# Contact Us
If you have any suggestions for improving this library, there are a few ways to get in touch:

*   Create a new issue
*   Submit a pull request
*   Virtual meeting using our [Open Source Office Hours](https://www.copperpunk.com/service-page/open-source-office-hours)