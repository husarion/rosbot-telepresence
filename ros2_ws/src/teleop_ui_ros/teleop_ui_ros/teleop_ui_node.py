import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

import tkinter as tk
from PIL import Image as PilImage
from PIL import ImageTk
from geometry_msgs.msg import Twist
import threading

class TeleopUiNode(Node):
    def __init__(self):
        super().__init__('camera_display_node')
        self.subscription = self.create_subscription(
            Image,
            '/rosbot2r/camera/color/image_raw/ffmpeg/decompressed',  # Adjust the topic name
            self.listener_callback,
            10)
        self.bridge = CvBridge()

        # Tkinter UI setup
        self.root = tk.Tk()
        self.root.title("Robot Control")
        self.image_label = tk.Label(self.root)
        self.image_label.pack()
        self.create_buttons()
        self.publisher = self.create_publisher(Twist, '/rosbot2r/cmd_vel', 10)

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')  # Convert to OpenCV format
            image = PilImage.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
            self.photo = ImageTk.PhotoImage(image=image)
            self.image_label.config(image=self.photo)
            self.image_label.image = self.photo  # Keep a reference!
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {str(e)}')

    def create_buttons(self):
        # Up Button
        up_button = tk.Button(self.root, text="Up")
        up_button.pack()
        up_button.bind('<ButtonPress>', lambda event: self.publish_velocity(0.5, 0.0))
        up_button.bind('<ButtonRelease>', lambda event: self.publish_velocity(0.0, 0.0))

        # Down Button
        down_button = tk.Button(self.root, text="Down")
        down_button.pack()
        down_button.bind('<ButtonPress>', lambda event: self.publish_velocity(-0.5, 0.0))
        down_button.bind('<ButtonRelease>', lambda event: self.publish_velocity(0.0, 0.0))

        # Left Button
        left_button = tk.Button(self.root, text="Left")
        left_button.pack()
        left_button.bind('<ButtonPress>', lambda event: self.publish_velocity(0.0, 1.0))
        left_button.bind('<ButtonRelease>', lambda event: self.publish_velocity(0.0, 0.0))

        # Right Button
        right_button = tk.Button(self.root, text="Right")
        right_button.pack()
        right_button.bind('<ButtonPress>', lambda event: self.publish_velocity(0.0, -1.0))
        right_button.bind('<ButtonRelease>', lambda event: self.publish_velocity(0.0, 0.0))


    def publish_velocity(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    teleop_ui_node = TeleopUiNode()

    # Create and start the ROS2 spinning thread
    ros_thread = threading.Thread(target=lambda: rclpy.spin(teleop_ui_node), daemon=True)
    ros_thread.start()

    try:
        teleop_ui_node.root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        teleop_ui_node.root.quit()  # Stop the Tkinter main loop
        teleop_ui_node.destroy_node()
        rclpy.shutdown()
        ros_thread.join()  # Wait for the ROS thread to finish

if __name__ == '__main__':
    main()

