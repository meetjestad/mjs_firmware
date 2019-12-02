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
