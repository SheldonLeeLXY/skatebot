# Building the C++ example code

## Updating the Dynamixels to Protocol 2

Full instructions are [here](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/#firmware-recovery).  However, here are my notes that should explain it (with out all the pretty pictures!)

1. Ensure only one Dynamixel servo is connected.
2. Scan for attached servos and check protocol.  If protocol 2.0, then nothing to do, else do the rest.
3. Start the program `Dynamixel Wizard 2`.
4. Select the menu option `Tools->Firmware Recovery` or press `F7`.
5. Wizard style dialog opens.
   1. Press `Next`.
   2. Select `Mx-28(2.0)` and press `Next`.
   3. Select `V45` and press `Next`.
   4. Check using correct serial port.  Normally `ttyUSB0`. If OK, press `Next`.
   5. Unplug power for 5 seconds and plug in again. Update process should start and end automatically. Press `Next`.
   6. Press `Finish`.
   7. Perform a scan and verify that the servo is found and it is using Protocol 2.0.  This is shown on the main screen with the little orange icon with the number 2 in it next to the servo ID info.

## Build and run the DXL Monitor example

The ROS 2 example code did not work so I wanted to build the C++ examples from the SDK.  This is what I did:

1. Build and install the library.

    ```bash
    cd ~/git
    mkdir ROBOTIS-GIT
    cd ROBOTIS-GIT/
    git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
    cd DynamixelSDK/c++/build/linux64
    make
    sudo make install
    ```

2. Build and run the DXL Monitor program.

    ```bash
    cd ~/git/ROBOTIS-GIT/DynamixelSDK/c++/example/dxl_monitor/linux64/
    make
    ./dxl_monitor
    ```

3. Running the DXL monitor program.

This program was really useful as it made me realise the reason why my code didn't work.

```text
./dxl_monitor

***********************************************************************
*                            DXL Monitor                              *
***********************************************************************

Succeeded to open the port!

 - Device Name : /dev/ttyUSB0
 - Baudrate    : 57600

[CMD] scan

Scan Dynamixel Using Protocol 1.0

 [ID:001] Model No : 00029                ... SUCCESS

 [ID:002] Model No : 00029                ... SUCCESS

 [ID:003] Model No : 00029                ... SUCCESS

 [ID:004] Model No : 00029                ... SUCCESS
........................................................................................................................................................................................................................................................

Scan Dynamixel Using Protocol 2.0
............................................................................................................................................................................................................................................................

[CMD]
```

The problem was that the code examples I had been using were for protocol 2.0.  The Dynamixels use protocol 1.0 by default.
