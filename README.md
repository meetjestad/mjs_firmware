Meet je stad sensor firmare
===========================
This repository contains the firmware for the Meet je stad! sensor
boards. These boards contain a number of sensors and a LoRa radio
module they use to send their readings to a central collection.

More information about het project, as well as the data collected from
the sensors is available on http://www.meetjestad.net

The hardware that this runs on is available from
https://github.com/meetjestad/mjs_pcb. This sketch expects a node id and
encryption keys to be preprogrammed into EEPROM, see
https://github.com/meetjestad/mjs_programmer for the tool that is used
for this.

To compile and upload this sketch, you will need:
 - The Arduino IDE (latest version is recommended)
 - The Meet je stad board files installed in the Arduino IDE (see
   https://github.com/meetjestad/mjs_boards for instructions).
 - The [SparkFun HTU21D Humidity and Temperature Sensor
   Breakout](https://github.com/sparkfun/SparkFun_HTU21D_Breakout_Arduino_Library)
   Arduino library. This can be installed through the library manager.
 - The [NeoGPS](https://github.com/meetjestad/NeoGPS) Arduino library
   (mjs-specific version). To install, download [this
   zipfile](https://github.com/meetjestad/NeoGPS/archive/mjs.zip) and
   install it using `Sketch -> Include Library -> Add .ZIP Library...`.
 - The
   [Adafruit_SleepyDog](https://github.com/adafruit/Adafruit_SleepyDog)
   Arduino library. This can be installed through the library manager.
 - The [IBM LMIC](https://github.com/meetjestad/arduino-lmic) Arduino
   library (mjs-specific version). To install, download [this
   zipfile](https://github.com/meetjestad/arduino-lmic/archive/mjs.zip)
   and install it using `Sketch -> Include Library -> Add .ZIP
   Library...`.

See also [the Arduino documentation on library
installation](https://www.arduino.cc/en/Guide/Libraries). Instead of
installing these libraries through the library manager or through a
zipfile, you can also make a git clone in your libraries directory.

After installing the above, download this sketch (by making a clone from
github, or [downloading the
zip](https://github.com/meetjestad/mjs_firmware/archive/master.zip)),
open the sketch, select the "MJS meetplatform v1" as the board and hit
upload.
