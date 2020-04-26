------------------------------------------------------------------------------------------------------------------------------------
ESP Wheel Project. 
Version 0.3. 30 July 2015. Copyright Ai-Wave 2015. http://www.aiwave.fr

------------------------------------------- How to compile the project -------------------------------------------------------------

Copy the ESPWheel arduino-1.0.5 folder content to your arduino-1.0.5 instalation folder, this will add a specific core and the needed libraries.
Open the RFRWheel project in Arduino 1.0.5 IDE or in visual studio + visual micro environement.
Choose your board/core : Arduino Leonardo (FFB HID)
Configure the digital/analog pins according to your real use.
Compile, download, et voila.

- FW customistations :

In config.h you can customise the firmware according to the pins/components/inputs you want to use.
To change the pins of the HX711 shield -> 'HX711.h' in the HX711 library.
To change the pins of the quadrature encoder- >  'QuadEncoder.h' in the project.

------------------------------------------- How to use -----------------------------------------------------------------------------

The FW will create a HID gaming device and a serial port.
By sending commands to the serial port you can configure the wheel.

- Available serial commands :

'c' : Center the wheel. The setting will be saved.
'r' : Reset the wheel and starts a new calibration process.

------------------------------------------- Hardware notes -------------------------------------------------------------------------

- on the yourduino HX711 shield, in order to have 80 SPS instead of 10 SPS you must unsolder the #15 pin (RATE) of the chip which is grounded, and connect it to +5V (pin 16).

------------------------------------------------------------------------------------------------------------------------------------

I don't own the rights of this software. The Distribution is allowed in the license
Source code found on may of 2017 at http://www.aiwave.fr/wc_updates/
