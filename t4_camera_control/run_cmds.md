# Turn on the light
python3 /home/pipebot/pipebot_4wd_ws/src/neopixel-rpi-ros2/scripts/neopixel-on-max.py

python3 /home/pipebot/pipebot_4wd_ws/src/neopixel-rpi-ros2/scripts/neopixel-off.py

# Rotate the skate
ros2 topic pub -1 /dynamixel_driver/servo/turret pipebot_msgs/msg/Servo "{angle_degrees: 0}"

# Move the robot
ros2 topic pub -1 /motor/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# Compile
colcon build --symlink-install --packages-select t4_camera_control
colcon build --symlink-install --packages-select dynamixel_driver
source ./install/setup.bash

# Skatebot
ros2 launch control_system pre_control_skate.launch.py
ros2 launch pipebot_4wd joystick_camera.launch.py

# Camera
ros2 run t4_camera_control camera_node

libcamera-jpeg -o test.jpg

libcamera-vid --qt

# Copy the Images
scp -r 'pipebot@172.20.10.2:/home/pipebot/pipebot_4wd_ws/src/t4_camera_control/imageset/*' /Users/lixiangyu/WorkSpace/imageset/ICAIR/manhole1/1

scp -r 'pipebot@192.168.1.192:/home/pipebot/pipebot_4wd_ws/*' /Users/lixiangyu/WorkSpace