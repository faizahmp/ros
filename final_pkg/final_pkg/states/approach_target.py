from yasmin import State
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
import math
import cv2

from final_pkg.yolo_inference.yolo_utils import run_yolo_on_image

class ApproachTarget(State):
    def __init__(self, node: Node):
        super().__init__(outcomes=['reached'])
        self.node = node
        self.publisher = self.node.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.node.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()
        self.image_received = None

        self.img_center_x = 320
        self.img_center_y = 240
        self.center_tolerance = 30
        self.size_threshold = 0.9  # relative to image size
        self.image_width = 640
        self.image_height = 480

        self.nav_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')

    def image_callback(self, msg):
        self.image_received = msg

    def execute(self, blackboard):
        object_queue = blackboard.object_queue if hasattr(blackboard, 'object_queue') else []
        root_pose = blackboard.root_point if hasattr(blackboard, 'root_point') else None

        while object_queue:
            target = object_queue.pop(0)
            self.node.get_logger().info(f"[ApproachTarget] Processing object: {target['class_name']} conf={target['confidence']:.2f}")

            # Approach loop
            while rclpy.ok():
                rclpy.spin_once(self.node)
                if self.image_received is None:
                    continue

                cv_image = self.bridge.imgmsg_to_cv2(self.image_received, desired_encoding='bgr8')
                detections = run_yolo_on_image(cv_image)

                matched = None
                for det in detections:
                    if det['class_name'] == target['class_name']:
                        matched = det
                        break

                if not matched:
                    self.stop_robot()
                    self.node.get_logger().warn("[ApproachTarget] Lost sight of target.")
                    break

                center_x = matched['center']['x']
                width = matched['dimensions']['width']
                height = matched['dimensions']['height']
                error_x = center_x - self.img_center_x
                twist = Twist()

                # Align
                if abs(error_x) > self.center_tolerance:
                    twist.angular.z = -0.002 * error_x
                    self.publisher.publish(twist)
                    continue
                else:
                    twist.linear.x = 0.05
                    self.publisher.publish(twist)

                # Stop condition
                if width >= self.image_width * self.size_threshold or height >= self.image_height * self.size_threshold:
                    self.stop_robot()
                    self.node.get_logger().info("[ApproachTarget] Reached object. Capturing and logging.")
                    self.capture_image(cv_image, matched)
                    break

            # Wait for user key input
            input("Press Enter to return to root point and continue...")

            if root_pose:
                self.return_to_pose(root_pose)

        return 'reached'

    def capture_image(self, image, detection):
        filename = f"/tmp/approached_{detection['class_name']}_{int(detection['confidence'] * 100)}.png"
        bbox = detection['bounding_box']
        cv2.rectangle(image, (int(bbox['x1']), int(bbox['y1'])), (int(bbox['x2']), int(bbox['y2'])), (0, 255, 0), 2)
        cv2.imwrite(filename, image)
        self.node.get_logger().info(f"[ApproachTarget] Image saved to: {filename}")

    def return_to_pose(self, pose):
        self.node.get_logger().info("[ApproachTarget] Returning to root point...")
        self.nav_client.wait_for_server()

        nav_goal = NavigateToPose.Goal()
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose = pose
        nav_goal.pose = pose_msg

        send_goal_future = self.nav_client.send_goal_async(nav_goal)
        rclpy.spin_until_future_complete(self.node, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.node.get_logger().warn("[ApproachTarget] Navigation goal rejected.")
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        self.node.get_logger().info("[ApproachTarget] Returned to root point.")

    def stop_robot(self):
        twist = Twist()
        self.publisher.publish(twist)
        self.node.get_clock().sleep_for(Duration(seconds=1))
