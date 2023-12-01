# Testing

These are ROS2 commands that can be used to verify the correct operation of the Dynamixel driver.  Each command is prefixed with a comment that explains the expected response.

All tests assume that your have started the package using this command:

```text
# Start package in terminal 1.
ros2 launch dynamixel_driver driver.launch.py
```

## Motor RPM

For these servos, positive RPM results in anti-clockwise rotation and negative RPM results in a clockwise rotation when looking at the servo output shaft.

```text
# Start left motor using 20 RPM anti-clockwise.  Right motor remains stopped.
ros2 topic pub --rate=5 /motor/left pipebot_msgs/msg/MotorControl "{mode: 2, rpm: 20.0}"
# Left motor changes to 20 RPM clockwise.  Right motor remains stopped.
ros2 topic pub --rate=5 /motor/left pipebot_msgs/msg/MotorControl "{mode: 2, rpm: -20.0}"
# Stop left motor.  Right motor remains stopped.
Ctrl+C
# Start right motor using 20 RPM anti-clockwise.  Left motor remains stopped.
ros2 topic pub --rate=5 /motor/right pipebot_msgs/msg/MotorControl "{mode: 2, rpm: 20}"
# Right motor changes to 20 RPM clockwise.  Left motor remains stopped.
ros2 topic pub --rate=5 /motor/right pipebot_msgs/msg/MotorControl "{mode: 2, rpm: -20}"
# Stop right motor.  Left motor remains stopped.
Ctrl+C
```

Check that watchdog stops motor after 300ms (or so).

```text
# Send command once.  Motor should run for a short period of time and then stop.
# Left motor.
ros2 topic pub -1 /motor/left pipebot_msgs/msg/MotorControl "{mode: 2, rpm: -20}"
# Right motor.
ros2 topic pub -1 /motor/right pipebot_msgs/msg/MotorControl "{mode: 2, rpm: -20}"
```

## Motor relative position

```text
# Both motors do nothing.
ros2 topic pub -1 /motor/left pipebot_msgs/msg/MotorControl "{mode: 1, angle_radians: 0}"
# Left motor turns 180 degrees clockwise.
ros2 topic pub -1 /motor/left pipebot_msgs/msg/MotorControl "{mode: 1, angle_radians: 3.14}"
# Left motor turns 180 degrees anti-clockwise.
ros2 topic pub -1 /motor/left pipebot_msgs/msg/MotorControl "{mode: 1, angle_radians: -3.14}"
# Right motor turns 180 degrees clockwise.
ros2 topic pub -1 /motor/right pipebot_msgs/msg/MotorControl "{mode: 1, angle_radians: 3.14}"
# Right motor turns 180 degrees anti-clockwise.
ros2 topic pub -1 /motor/right pipebot_msgs/msg/MotorControl "{mode: 1, angle_radians: -3.14}"
```

This test is to prove the relative position change works correctly after a long period of travelling using velocity control.

```text
# Start motor running in terminal 2.
ros2 topic pub --rate=5 /motor/left pipebot_msgs/msg/MotorControl "{mode: 2, rpm: 60.0}"
# Monitor left encoder output in terminal 3.
ros2 topic echo /encoder/left
# Wait until encoder count exceeds 1,050,000.
# Note encoder count.
# Stop motor using Ctrl+C in terminal 2.
# Then send this command to move the servo 180 degrees clockwise.
ros2 topic pub -1 /motor/left pipebot_msgs/msg/MotorControl "{mode: 1, angle_radians: 3.14}"
# Verify that servo rotates 180 degrees clockwise.
# Verify that the encoder count increases by about 2048, +/-50 counts.
```

### Encoder

Monitor the output of one of the encoders using one of the following commands.

```text
ros2 topic echo /encoder/left
ros2 topic echo /encoder/right
```

Drive the motors forward and backward.

Verify that the encoder tick values go increase when moving forward and decrease when moving backwards.

Verify that `wheel_revs` changes correctly with every rotation of the wheel. This robot has 4096 encoder counts per revolution.

Verify that the `wheel_angle_deg` values change correctly.  Use the motor relative position commands.

### Turret Servo Position

Note: The turret is physically limited to a range of forward (0 degrees) and backwards (180 degrees).

```text
# Turret moves to central position, 180 degrees.
ros2 topic pub -1 /servo/turret pipebot_msgs/msg/Servo "{angle_degrees: 180}"
# Turret moves to 90 degrees.
ros2 topic pub -1 /servo/turret pipebot_msgs/msg/Servo "{angle_degrees: 90}"
# Turret moves to 0 degrees.
ros2 topic pub -1 /servo/turret pipebot_msgs/msg/Servo "{angle_degrees: 0}"
```
