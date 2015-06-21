# chickenhouse
The automatic chicken house door controller opens and closes the door of a
chicken house based on daylight and time. It opens the door in morning and closes it 
in the evening, fully automatically. The controller drives relais, to which a motor for
the hatch is attached. The controller may be hooked up to the internet using the 
arduino ethernet shield, which allows the device to be remote controlled.

Additionally some extra sensors like a temperature and/or humidity sensors may be attached 
and there are spare relais channels, that can be used to switch other devices like light 
or a heater. So, the door controller can also serve as a weather monitor.

## hardware
Hardware used in this project.
 * [arduino UNO](http://www.arduino.cc/en/Main/ArduinoBoardUno)
 * [ethernet shield](http://www.arduino.cc/en/Main/ArduinoEthernetShield)
 * [relay board](http://www.sainsmart.com/4-channel-5v-relay-module-for-pic-arm-avr-dsp-arduino-msp430-ttl-logic.html)
 * [DHT22 temperature and humidity sensor](https://www.sparkfun.com/products/10167)
 * [DC motor with gearbox](http://www.mfacomodrills.com/gearboxes/919d_series.html)

### watchdog and bootloader
If you enable the watchdog ([learn about the watchdog](http://playground.arduino.cc/Main/ArduinoReset)), which I highly recommend, you need to use a bootloader the can deal with a watchdog reset, otherwise the arduino will get stuck in an infinite loop upon watchdog reset. I use [optiboot](https://github.com/Optiboot/optiboot) for all my arduino projects. You should test if your bootloader can deal with the watchdog timer before enabling it.

## libraries
This code depends on the following libraries, which are not part of the arduino IDE
 * [Webduino](https://github.com/sirleech/Webduino)
 * [Streaming](http://arduiniana.org/libraries/streaming/)
 
## compile
To compile and upload the programm to the arduino, I use the [arduino IDE](http://www.arduino.cc/Main/Software). Just follow the instructions of the [arduino website](http://www.arduino.cc/en/Guide/HomePage).


