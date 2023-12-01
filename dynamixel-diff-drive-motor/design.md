# Design Notes

## High level design

As the Dynamixel hardware uses a USB serial interface, this is what we use.  This gives us the following rough design.

```text
 __________________________
| RPi                      |     ___________       ___________       ___________
| ________________  USB    |    |           |     |           |     |           |
| | ROS2 Wrapper |--------------| Dynamixel |-----| Dynamixel |-...-| Dynamixel |
| |              | Serial  |    | Interface |     | 1         |     | N         |
| |______________|         |    |___________|     |___________|     |___________|
|__________________________|

```

The ROS 2 wrapper handles the communications with the rest of the system using the following topics:

| Topic | Pub/sub | Message |
|---|---|---|
| `/motor/cmd_vel` | Subscriber | `geometry_msgs/msg/Twist` |
| `/motor/left` | Subscriber | `pipebot_msgs/msg/MotorControl` |
| `/motor/right` | Subscriber | `pipebot_msgs/msg/MotorControl` |
| `/encoder/left` | Publisher | `pipebot_msgs/msg/Encoders` |
| `/encoder/right` | Publisher | `pipebot_msgs/msg/Encoders` |
| `/servo/turret` | subscriber | `pipebot_msgs/msg/Servo` |

The messages used are defined in the `pipebot_msgs` package.

## Communicating with the Dynamixels

The wrapper also communicates with the Dynamixel SDK to send and receive data. The Dynamixel SDK provides very low level methods to talk to one or more Dynamixels.  Some good example code can be found here:
<https://github.com/ROBOTIS-GIT/DynamixelSDK/blob/ros2/dynamixel_sdk_examples/src/read_write_node.cpp>

The ROS 2 wrapper defines an interface class with high level commands like this:

```c++
void SetRPM(const MotorInstance motor, double rpm);
```

What this command will do is send commands to a pair of Dynamixels that sets the RPM in whatever units the Dynamixels use.  It was quickly realised that there was a split of responsibilities needed, the first to setup the serial port and the second to manage the motors.

## The `DynamixelComms` class

The `DynamixelComms` class is responsible for:

* Setting up the Dynamixel `PortHandler` and `PacketHandler` objects.
* Creating, owning and destroying all instances of the `DynamixelServo` class and forwarding calls to the correct instance.
* Reporting connection status.

## The `DynamixelServo` class

The `DynamixelServo` class is responsible for:

* Implementing the communications to the attached Dynamixel servo.
* Initialising and terminating the servo.
* Sending commands to the servo to control speed, absolute position and relative position.
* Obtaining encoder pulse counts from the servo.
* Obtaining status information from the servos about voltage, temperature etc.

Much of the class was based on example code from the Dynamixel SDK repo.

## Physical arrangement

Motors are arranged like this.

|       | Left | Right |
|---|---|---|
| Front | 2 | 1 |
| Rear  | 3 | 4 |

The turret servo is ID 0.

Initially, the arrangement was not correct and it caused some very strange behaviour.  The above arrangement was verified with the Dynamixel Wizard 2.0 tool.  The Dynamixels follow the convention where motors with clockwise rotation turn the output shaft clockwise when looking at the motor from the output shaft end.  Once the above problem was corrected, the behaviour of the robot what fully tested using the ROS 2 commands below and found to be correct.

## Watchdog feature

We had a problem where the control package died and the robot kept on moving until it fell over.  We need to add a feature to stop if no message is received for a preset time.  The control package sends out messages to the motor every 200ms.  The node `teleop_twist_joy_node` sends messages between 5Hz and 20Hz. So, a timeout of 300ms seems like a good starting point.

The next question is where to put the time out.  The `twist` message calls the function `SetRPM` as do the motor topics, so putting the timer logic inside the `DynamixelComms` class looks good.  this proved to be a pain as the ROS 2 functions to manage a time need to be in a node, so the logic has to go into the `MotorNode` class instead.

## Total Encoder Count

The encoder count reported by the Dynamixel servos that are used as wheel motors works well until a mode change occurs when the encoder count is modified.  The natural place to implement logic that works around this problem is in the `DynamixelServo` class.

The encoder values are read on a timer thread and operating modes can be changed on a separate thread, so thread safety needs to be considered.  We should also consider thread safety for all the Dynamixels as they are all connected using a single serial port, so this serial port should be mutex protected.  Implementing a mutex in the `DynamixelComms` class also solved the thread safety issues in the `DynamixelServo` class.

The updating of the value `total_encoder_count_` is implemented in a new function, `DynamixelServo::UpdateEncoderCount`.  This function is called by two other functions, `DynamixelServo::SetOperatingMode` and `DynamixelServo::GetTotalEncoderCount`.

### Case 1

The function `DynamixelServo::GetTotalEncoderCount` will be called at least once a second.  The action in this case is to read current position value, work out the change in value (current - last) and add the change to the running total.

### Case 2

The function `DynamixelServo::SetOperatingMode` can change the operation mode to any of the four different modes offered by the Dynamixels.  However, we are currently only using two modes,     `kVelocityControlMode` and `kExtendedPositionControlMode`, so this makes things a little easier.

When in `kVelocityControlMode`, the encoder count goes up to +/-2^31 counts.  When in `kExtendedPositionControlMode`, the encoder count goes up to +/-1048575 counts (around 256 revs each way).  This was tested and the  Dynamixel firmware limits the values that can be input to +/-1048575.  Sending an out of range value causes the Dynamixel to respond with a data value out of range message.

By experimenting using the Dynamixel wizard 2.0 program, the following operations were needed to use `kExtendedPositionControlMode` after an extended period of using `kVelocityControlMode`.

1. Drive motor so that the value of the `Present Position` register is greater than 1048575 (2^20).
2. Turn off torque.
3. Reboot the servo to reset the encoder count to within range.
4. Change to `kExtendedPositionControlMode`.
5. Enable torque.

This approach was implemented in the function `DynamixelServo::SetOperatingMode` and found to work correctly.

### Total encoder count logic

The logic needed to store the total number of encoder counts was then implemented.  Case 1 was simple to implement and was added to a new function `UpdateEncoderCount`.  This new function was called before returning the total in the function `GetTotalEncoderCount`.  After some thought, it was realised that the case 2 logic needed two function calls to `UpdateEncoderCount`.  The first call would read the current position and add the delta to the total, exactly the same as case 1, and the second call after the reboot to set the new value of the servo position to calculate the delta next time the function `UpdateEncoderCount` was called.

### Other considerations

The other concern about the encoder values is wrap round at counts of +/-2^31.  This value is equal to a 129882 full revolutions, so for the SkateBot with a wheel diameter of 100mm, it equals a total distance travelled of 2 * PI * 0.05m * 129882 = 40,803m or 40.8km.  This is not going to happen during scientific experiments so can be ignored.

## Turret rotation

The Skatebot has a turret that rotates 180 degrees controlled by a single servo.  The servo is connected to the turret by a 3D printed gear (blacklash city!) with a ratio of around 2:1. When using the Dynamixel Wizard to control the turret servo, the values that turned the turret though 180 degrees was -200 to 4000, so a range of just over the complete single turn of 4096 was needed.  As we need to rotate the servo though more than one complete turn, we need to use extended position mode.  The exact gear ratio is 4200 / 2048 = 2.05, so we need to add some compensation into the mix somewhere.

The servo message specifies an input range of -360 to +360 degrees, with increasing values rotating the servo in an anti-clockwise direction when looking at the end of the output shaft.  After the gear ratio is applied, an input value of 180 degrees should rotate the turret 180 degrees.

Modifications were made to many files as the communications interface needed to be changed, mostly from `int16_t` to `double` to cope with the gear ratio being appiled.  The gear ratio is set in the launch file as a parameter `gear_ratio` so can be modified for other robots.
