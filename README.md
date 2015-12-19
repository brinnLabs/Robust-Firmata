Robust Firmata
======================

Firmata with the addition of the firmata stepper addition written by @soundanalogous with some tweaks to allow for limit switches. Robust firmata also includes the Encoder and OneWire protocols though only the Encoder one has been tested. 

Install
==================

to install just move the all the files except the openframeworks folders to the utility folder of your arduino Firmata install

on a pc it might be located here:
C:\Program Files (x86)\Arduino\libraries\Firmata

for OSX

right click on the arduino.app and show package contents. 
The folder will be in contents/Resources/Java/libraries/Firamta

I would suggest using the RobustFirmata sketch included here and then just removing features you don't need.


Extras
++++++++++++++

Reworking the openframeworks addon, currently the ofArduino located in my repo at:
https://github.com/DomAmato/openFrameworks/tree/ofArduino_Updates/libs/openFrameworks/communication
will enable all funcitonality except some of the non-standard stepper calls
