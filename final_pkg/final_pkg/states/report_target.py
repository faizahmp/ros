from yasmin import State
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from rclpy.duration import Duration
import rclpy
from rclpy.node import Node
import cv2
import os

class ReportTarget(State):
    def __init__(self, node: Node):
        super().__init__(outcomes=['done'])
        self.node = node
        self.bridge = CvBridge()
        self.image_received = None

        self.image_sub = self.node.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.odom_sub = self.node.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.latest_pose = None

        self.save_dir = os.path.expanduser('~/turtlebot_logs')
        os.makedirs(self.save_dir, exist_ok=True)

    def image_callback(self, msg):
        self.image_received = msg

    def odom_callback(self, msg):
        self.latest_pose = msg.pose.pose

    def execute(self, blackboard):
        self.node.get_logger().info("[ReportTarget] Logging current object and location...")
        rclpy.spin_once(self.node, timeout_sec=1.0)

        if not self.image_received:
            self.node.get_logger().warn("[ReportTarget] No image to report.")
            return 'done'

        if not self.latest_pose:
            self.node.get_logger().warn("[ReportTarget] No pose data available.")
            return 'done'

        # Convert image
        cv_image = self.bridge.imgmsg_to_cv2(self.image_received, desired_encoding='bgr8')

        # Display image
        cv2.imshow("Captured Object", cv_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        # Save image to file
        filename = os.path.join(self.save_dir, 'log_image.png')
        cv2.imwrite(filename, cv_image)

        # Extract and log pose
        pos = self.latest_pose.position
        log_text = f"[ReportTarget] Location: x={pos.x:.2f}, y={pos.y:.2f}, z={pos.z:.2f}"
        print(log_text)
        self.node.get_logger().info(f"Image saved: {filename}")

        return 'done'
