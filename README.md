Firmata-w-input_pullup
======================

Altered Firmata to support the Input Pull-up on arduino boards

Install
==================

to install just move the Firamata.h file to the libraries of your arduino install

on a pc it might be located here:
C:\Program Files (x86)\Arduino\libraries\Firmata

for OSX

right click on the arduino.app and show package contents. 
The folder will be in contents/Resources/Java/libraries/Firamta

You can either modify one of the firmata examples 
or just use the StandardFirmataEdit sketch included
and upload to an arduino board


Extras
++++++++++++++

also includes replacement .h and .cpp files to be used with openframeworks
just replace the old ofArduino files in the libs/openframeworks/communication folder

can be used just like normal just use

sendDigitalPinMode(pin, ARD_INPUT_PULLUP);
