import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.cmd_vel_pub = self.create_publisher(Twist, '/motor/cmd_vel', 10)
        self.twist_cmd = Twist()

    def publish_cmd_vel(self):
        self.cmd_vel_pub.publish(self.twist_cmd)

    def read_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = None
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

def run_keyboard_teleop(self):
    self.get_logger().info("Keyboard teleoperation node started.")
    while rclpy.ok():
        key = self.read_key()

        if key == '\x03':  # Ctrl-C
            break

        if key == 'w':
            self.twist_cmd.linear.x = 0.2
            self.get_logger().info("Moving forward at 0.2 m/s")
        elif key == 's':
            self.twist_cmd.linear.x = 0.0
            self.twist_cmd.angular.z = 0.0
            self.get_logger().info("Stopping")
        elif key == 'x':
            self.twist_cmd.linear.x = -0.2
            self.get_logger().info("Moving backward at 0.2 m/s")
        else:
            self.twist_cmd.linear.x = 0.0

        if key == 'a':
            self.twist_cmd.angular.z = 0.5
            self.get_logger().info("Turning left at 0.5 rad/s")
        elif key == 'd':
            self.twist_cmd.angular.z = -0.5
            self.get_logger().info("Turning right at 0.5 rad/s")
        else:
            self.twist_cmd.angular.z = 0.0

        self.publish_cmd_vel()



def main(args=None):
    rclpy.init(args=args)
    teleop_node = KeyboardTeleop()
    teleop_node.run_keyboard_teleop()
    teleop_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

            