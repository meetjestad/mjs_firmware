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
