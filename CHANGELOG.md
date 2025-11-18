Changes in v7
=============
 - Support entering LoRaWAN credentials via serial also on MJS2020. This
   also fixes compilation on MJS2020 which was accidentally broken in
   v6 because credential writing was not supported.
 - Write option bytes on MJS2020 at the same time as writing
   credentials.

Changes in v6
=============
 - Support entering LoRaWAN credentials via serial if none are present
   in EEPROM (only v1 stations for now, MJS2020 stations must still be
   pre-preprogrammed with credentials).
 - Fix printing of EUIs on startup, they were accidentally
   byte-reversed.
 - Fix compilation with `WITH_LUX` defined.
 - Some cosmetic source changes.

Changes in v5
=============
 - Support for the MJS2020 board.
 - Support for the Sensirion SPS30 particle sensor (only tested on
   MJS2020).
 - Do a join directly at startup, before the first GPS is first enabled.
 - Print more human-readable station name on startup.
 - Skip reading GPS and SPS30 sensor when battery is low.
 - Fix buffer overflow when reading LoRaWAN keys.
 - Fix use of uninitialized memory and buffer overflow when reading GPS.
 - Various small code cleanups and improvements (without behavioral
   changes).

Changes in v4
=============
 - Switch to a slightly improved packet format. This new format allows
   a bigger range on lux values (not used now, but can be utilized by
   replacing the related resistors) and allows adding extra arbitrary
   data values in the packet (not used now, but can be utilized by
   modifying the code, see the comments).
 - Remove a superfluous pin write and analog reference change in the lux
   meter code.
 - Throw away one lux measurement after changing to the internal
   reference. This is recommended by the datasheet and could improve
   accuracy.
 - Accomodate adding a capacitor on the lux meter for extra measurement
   stability.
 - Allow skipping the GPS update by sending an 's' (or 'S') through
   serial. This might be useful to reduce wait times during testing
   indoors.
 - Some other code cleanups.

Changes in v3
=============
 - Improve handling of GPS readouts, leading to gps reading failures
   less often.

Changes in v2
=============
 - More detailed status messages printed on startup, for debugging
   problems.
 - Support reading a light sensor (when enabled during programming).

Changes in v1
=============
These are some of the changes since the earlier versions that did not
have a version number yet.
 - Send a firmware version number in every packet.
 - Improve GPS accuracy by keeping it on for a bit longer after a fix is obtained.
 - Increase the GPS timeout from 60 to 120 seconds.
 - Do not send the previous position when the gps cannot obtain a fix.
 - Much more changes.
