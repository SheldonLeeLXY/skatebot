# Dynamixel Driver

A ROS 2 driver for Dynamixels used as motors.

These docs have details on [design](design.md) and [testing](testing.md).

## IMPORTANT - Use Protocol 2.0

The code uses Dynamixel Protocol 2.0 code.  If you use this code on a Dynamixel with protocol 1.0 installed (default for the Dynamixels we were using), you will get errors like this:

```text
[INFO] [1678372358.877233494] [Comms]: Opened port /dev/ttyUSB0.
[INFO] [1678372358.877685048] [Comms]: Baud rate set to 57600.
[ERROR] [1678372358.914336055] [DynamixelMotor]: Instance 0, dxl id 1 not found.
[ERROR] [1678372358.951040468] [DynamixelMotor]: Instance 0, dxl id 2 not found.
[ERROR] [1678372358.987714068] [DynamixelMotor]: Instance 1, dxl id 3 not found.
[ERROR] [1678372359.024363278] [DynamixelMotor]: Instance 1, dxl id 4 not found.

```

To fix this, follow the instructions in the [troubleshooting notes](troubleshooting.md).

## Acknowledgments

This work is supported by the UK's Engineering and Physical Sciences Research Council (EPSRC) Programme Grant EP/S016813/1

© 2023, University of Leeds.

The author, A. Blight, has asserted his moral rights.
