import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class TeleopUiNode(Node):
    def __init__(self):
        super().__init__('camera_display_node')
        self.subscription = self.create_subscription(
            Image,
            '/rosbot2r/camera/color/image_raw/ffmpeg/decompressed',  # Adjust the topic name
            self.listener_callback,
            10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')  # Convert to OpenCV format
            cv2.imshow('Camera Image', cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    teleop_ui_node = TeleopUiNode()
    rclpy.spin(teleop_ui_node)
    teleop_ui_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
