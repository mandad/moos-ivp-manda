CHRobotics GP9 MOOS Interface
=============================

This driver provides an interface to a MOOS database for a CHRobotics GP9 integrated GPS/IMU system.  It interfaces over a serial port as specified in the configuration file, and is able to read and write any parameter for the device.

At the moment, only basic position and orientation are reported to the MOOSDB.

Installation Requirements
-------------------------
In order to complile the interface, the [serial library](http://wjwwood.io/serial/) by William Woodall is required.  The current linking expects the dynamic library file from the serial library to be in the moos-ivp/bin directory, although the path could be changed in CMakeLists.txt.


Author Information
------------------
Ported from a ROS Interface for the UM7 by Damian Manda - NOAA & University of New Hampshire
