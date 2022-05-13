Meet je stad station firmware
===========================
This repository contains the firmware (in the form of an Arduino sketch)
for the Meet je stad! sensor boards. These boards contain a number of
sensors and a LoRa radio module they use to send their readings to a
central collection.

More information about het project, as well as the data collected from
the sensors, see https://www.meetjestad.net

USB-to-serial board for programming (original board only)
---------------------------------------------------------
To program the sensor boards, you will need to connect them to a
computer. This is done through a serial connection, typically using a
cheap USB-to-serial converter (often called a "programmer", though it is
not a full-blown, and often more expensive "ISP programmer").

As for the actual USB-to-serial converter to use, most will work. A lot
of them use a standard 6-pin connector often called the "FTDI serial
connector" or similar (referring to the FTDI manufacturer whose chips
were originally used) and the board is designed to work with such a
6-pin connector.

An example of such an adapter is
[here](https://www.sparkfun.com/products/9716) and
[here](https://store.arduino.cc/arduino-usb-2-serial-micro), though most
of them will work. It is important to have an adapter that runs on 5V
and that has a DTR (or possibly CTS - we did not test this) pin
available to connect to the sensor board reset pin.

Basic usage
-----------
If you already have a sensor station, and just want to upload (and
possibly modify) this Arduino sketch, this section is for you. The
instructions below make use of a zip file that conveniently bundles
various components you will need. This zipfile is manually made, so it
does not neccesarily contain the most recent version from this
repository, but it usually contains a recent stable version.

To set this up:
 1. Install the Arduino IDE. It can be downloaded from https://arduino.cc.

    Note that you need the (offline) Arduino IDE, not the (online)
    Arduino web editor (the latter only works for official Arduino
    boards).
 2. Figure out where your "sketchbook" directory is. When you start the
    Arduino IDE and go to File -> Preferences..., you will see the
    "Sketchbook location" at the top. By default, the sketchbook is a
    directory called "Arduino" in your documents directory or homedir.
 3. Download the zip bundle:
    https://www.meetjestad.net/meten/files/arduino-bundle-v5.zip
 4. Unpack the zipfile into your sketchbook directory. The zipfile contains
    a directory called "hardware" and "libraries", which may already
    exist inside your sketchbook. If so, you should merge their
    contents.

If you did this right, you should have the following structure (this
assumes your sketchbook directory is called "Arduino"):

      Arduino/hardware
      Arduino/hardware/meetjestad
      Arduino/libraries
      Arduino/libraries/Adafruit_SleepyDog
      Arduino/libraries/SparkFun_HTU21D_Breakout_Arduino_Library
      Arduino/libraries/arduino-lmic
      Arduino/libraries/NeoGPS
      Arduino/mjs_firmware

When using the MJS2020 board, you additionally need to install the board
files using the IDE itself:
 1. Open the Arduino IDE, and go to File -> Preferences.
 2. In the "Additional board manager URLs", add the following url:
    `https://github.com/meetjestad/mjs_boards/raw/master/package_meetjestad.net_index.json`
 3. Close the preferences, and open Tools -> Board -> Boards Manager...
 4. Find the "Meet je stad! STM32L0 Boards" in the list, and click Install.

Next, to actually compile and upload the sketch:
 1. Connect the sensor board to your computer using your USB-to-serial
    converter (original board) or USB-C cable (MJS2020 board). Make sure
    to use the correct orientation, check the labels on both boards.
 2. Start the Arduino IDE and load the `mjs_firmware` sketch from your
    sketchbook (which should be reachable through File -> Sketchbook).
    If it is not listed, the `mjs_firmware` directory from the zip
    bundle was not installed correctly.
 3. Select your board under Tools -> Board. For the original board,
    select the "MJS Meetstation", for MJS2020 select e.g. MJS2020-PROTO3
    (or another version if you have an older proto board) If it is not
    listed, the `hardware` directory from the bundle zip was not
    correctly installed.
 4. Select the right port under Tools -> Port. Hint: check the port menu
    before and after connecting the usb cable, to see what port belongs
    to it.
 5. Click the "Upload" icon in the toolbar (or select Sketch -> Upload).
    This should compile the sketch and upload it to the board.

Debugging with serial port
--------------------------
When it runs, the firmare generates some debug info about what it is
doing, which can be helpful to diagnose problems. You can access this
serial output using the "Serial Monitor" in the Arduino IDE, or any
other serial terminal program running on your computer.

To connect to the serial port, you have to select the correct port
(under Tools -> Port) first, in the same way as you would do for
uploading firmware (see above).

For the original board, the serial connection needs an USB-to-serial
adapter. The baudrate configred in the serial monitor must be 9600. When
you open the serial monitor, the board will be reset automatically, so
you will always be able to see the startup output.

For the MJS2020 board, the serial connection can be made using the
on-board USB-C connector directly. The baudrate configured in the serial
monitor does not matter (it is a virtual serial connection only). It is
important that you **open the serial port within 10 seconds of startup**
(e.g. after enabling power, or pressing the reset button). If the serial
monitor is not opened within this time, the board will decide to *not*
print anything to the serial port, even if you would open the port
later. Also, once the board goes to sleep, the serial port is completely
shutdown to save power.

Board status through LED color (MJS2020 only)
---------------------------------------------
The MJS2020 board has a multi-color LED on the board that can be used to
see what the status is, especially during startup.

The LED is:
 - red while waiting for USB/Serial (max 10s)
 - orange while doing initial sensor readings
 - yellow while waiting for TTN activation
 - cyan while waiting for a GPS fix
 - purple while reading the SPS030 sensor
 - blue when doing LoRaWAN transmission

In all other cases (including when sleeping, which will be most of the
time), the LED is off to save power.

Advanced usage
--------------
If you are more familiar with Arduino and programming, you might want to
install various components manually from git instead of through a
prebuilt bundle zip, to make it easier to track modifications to the
code.

The bundle zip file contains the following components:
 - The [Meet je stad board files](https://github.com/meetjestad/mjs_boards). See the repository
   for sources and installation instructions.
 - The [SparkFun HTU21D Humidity and Temperature Sensor
   Breakout](https://github.com/sparkfun/SparkFun_HTU21D_Breakout_Arduino_Library)
   Arduino library. This is used without mjs-specific modifications.
 - The [Adafruit_SleepyDog](https://github.com/adafruit/Adafruit_SleepyDog)
   Arduino library. This is used without mjs-specific modifications. It
   is needed for the original board only.
 - The [NeoGPS](https://github.com/meetjestad/NeoGPS/tree/mjs) Arduino
   library (mjs branch). This library is modified for mjs, to configure
   it for the GPS used.
 - The [IBM LMIC](https://github.com/meetjestad/arduino-lmic/tree/mjs) Arduino
   (mjs branch). This library is modified for mjs, to configure it to
   remove unused parts and save some space. This is needed for the
   original board only.
 - The [BasicMAC](https://github.com/meetjestad/basicmac/tree/mjs)
   library. This is a continuation of LMIC that supports the
   radio on the MJS2020 board, but is too big to fit on the original
   board. Note that the repository is not a ready-made Arduino library,
   but you need to run as script to convert it. This library is modified
   for mjs, though at some point the modifications can probably be
   merged back. This is needed for the MJS2020 board only.
 - The [arduino-sps](https://github.com/meetjestad/arduino-sps/tree/mjs)
   Arduino library. This library is modified of for mjs, to use the
   second I²C-bus on the board. This is needed for MJS2020 only.
 - The `mjs_firmware` sketch, which can be cloned or downloaded from
   this repository.

See also [the Arduino documentation on library
installation](https://www.arduino.cc/en/Guide/Libraries). Unmodified
libraries can be installed through the Arduino IDE library manager, all
libraries can be installed by cloning from github, or downloading a
zipfile from github and install it using `Sketch -> Include Library ->
Add .ZIP Library...`.

After installing the right components, the compilation and upload
process is the same as with the basic usage above.

Hardware
--------
The hardware that this sketch is intended to run on is documented at
https://github.com/meetjestad/mjs_pcb. This sketch expects a node id and
encryption keys to be preprogrammed into EEPROM (original board) or
Flash (MJS2020 board), see https://github.com/meetjestad/mjs_programmer
(original board) or https://github.com/meetjestad/mjs_bootstrap (MJS2020
board) for the tool that is used for this. All microcontrollers used in
the workshops of the Meetjestad project, are already preprogrammed and
labeled with their identifier.

History of changes
------------------
For a list of notable changes in versioned firmware releases, see the
[CHANGELOG.md][] file. For detailed changes, see the git history.


License
-------
All content in this repository is licensed under the ["Beluki"
license][license]:

> Permission is hereby granted, free of charge, to anyone obtaining a copy of
> this document and accompanying files, to do whatever they want with them
> without any restriction, including, but not limited to, copying, modification
> and redistribution.
>
> NO WARRANTY OF ANY KIND IS PROVIDED.

[license]: https://github.com/Beluki/License/blob/master/Documentation/License
