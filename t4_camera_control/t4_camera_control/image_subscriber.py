import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from pipebot_msgs.msg import Servo  # Import the Servo message type
import cv2
import numpy as np
import time

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera_node/image_raw/compressed',
            self.image_callback,
            10
        )
        self.servo_publisher = self.create_publisher(
            Servo,                                          # Specify the message type
            '/dynamixel_driver/servo/turret',               # Specify the topic name
            10                                              # Specify the QoS profile
        )
        
        self.angle_degrees = -90
        self.timer = None
        self.capture_image = False
        self.image_count = 1

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if self.capture_image:
            # 添加适当的延迟等待摄像头调整曝光时间
            time.sleep(2)  # 这里设置为2秒，你可以根据需要调整等待时间

            file_name = f'/home/pipebot/pipebot_4wd_ws/src/t4_camera_control/imageset/capture_image_{self.image_count}_{self.angle_degrees}.png'
            cv2.imwrite(file_name, image)
            self.get_logger().info('Capture Image Number: %d' % self.image_count)
            self.capture_image = False
            self.image_count += 1

    def publish_angle(self):
        servo_msg = Servo()
        servo_msg.angle_degrees = self.angle_degrees
        self.servo_publisher.publish(servo_msg)
        self.get_logger().info('Published angle: %d' % self.angle_degrees)

        if self.angle_degrees == 180:
            self.angle_degrees = 10
            self.capture_image = False
        elif self.angle_degrees == 10:
            self.get_logger().info('Reached 0 degrees. Shutting down...')
            self.timer.cancel()
            rclpy.shutdown()
        else:
            time.sleep(2)  # 这里设置为2秒，你可以根据需要调整等待时间
            self.angle_degrees += 30          
            self.capture_image = True  

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    image_subscriber.timer = image_subscriber.create_timer(3.0, image_subscriber.publish_angle)
    rclpy.spin(image_subscriber)

if __name__ == '__main__':
    main()
