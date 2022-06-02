
# Project Thetis Firmware Version 0.5.0 Changelog

Version 0.5 will be an update to implement features found in the Thetis RevF4 instrumentation package.

## BUG FIXES:

- RGB LED colors for GPS lock need to change from RED to BLUE
- Add SD card support
- Fix log enable button interrupt

## CHANGED FEATURES:

- Changed GPS refresh rate to 10 Hz from 1 Hz
- Changed GPS data strings to only GGA
- Changed timestamp field in data packet to individual values
  - Data written to debug console or log file will still be displayed as ISO1806 (e.g. YYYY-MM-DDTHH:MM:SS.sssZ)
-  Minor refractoring

## ADDED FEATURES:

- Initialize and use the LSM6DSO IMU
- Add USB detection for enabling DEBUG mode