LoRaWAN / TTNMapper Rangetesting sketch
---------------------------------------
This sketch is intended to run on the Meetjestad sensor node and
takes GPS readings and sends them over LoRaWAN. These measurements are
intended to be received by The Things Network, decoded there and
forwarded to http://ttnmapper.org using their HTTP integration.

More information about het project, as well as the data collected from
the sensors is available on http://www.meetjestad.net

To compile and upload this sketch, you will need:
 - Install the board files to allow uploading to the board.
   (See https://github.com/meetjestad/mjs_boards)
 - Install the [NeoGPS](https://github.com/meetjestad/NeoGPS) Arduino
   library (mjs-rangetester-specific version). To install, download [this
   zipfile](https://github.com/meetjestad/NeoGPS/archive/mjs-rangetest.zip) and
   install it using `Sketch -> Include Library -> Add .ZIP Library...`.

   Note that this version is different from the one used for our main
   `mjs_firmware` sketch, this one enables HDOP parsing, while that one
   does not need that.
 - The [IBM LMIC](https://github.com/meetjestad/arduino-lmic) Arduino
   library (mjs-specific version). To install, download [this
   zipfile](https://github.com/meetjestad/arduino-lmic/archive/mjs.zip)
   and install it using `Sketch -> Include Library -> Add .ZIP
   Library...

Then upload this sketch to the board. If you have a Meetjestad sensor
node, with pre-registerd numbered chip, it should work right away (the
server side of things is already set up).

If you use different hardware, or do not have a preregistered
microcontroller, you might need to do some more work:
 - Change the NeoGPS library configuration to suit your GPS hardware.
 - Remove the `SW_GND_PIN` handling (which is a pin that can be used to
   power the GPS off by disconnecting its ground on the mjs sensor
   board).
 - Put your device's TTN credentials in `mjs_lmic.h` (normally it loads
   them from EEPROM).
 - On the TTN side, you need to set up a decoder (see decoder.js) and
   configure the TTN Mapper integration.  To see your data, you can use
   the TTN Mapper [Advanced
   map](https://ttnmapper.org/special_maps.php). You can use the device
   ID (which by default is "meetstation-123" for Meetjestad node number
   123).
