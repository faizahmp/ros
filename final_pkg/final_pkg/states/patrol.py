from yasmin import State
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
import tf_transformations
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from yasmin import State
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from action_msgs.msg import GoalStatus
import tf_transformations


class Patrol(State):
    """YASMIN state that drives TurtleBot3 through a list of waypoints."""

    def __init__(self, node: Node):
        super().__init__(outcomes=["reached", "finished"])
        self.node = node

        # Connect to the Nav2 action server
        self.client = ActionClient(self.node, FollowWaypoints, "follow_waypoints")
        while not self.client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().info(
                "'follow_waypoints' action server not available – waiting..."
            )

        # Waypoints: (x [m], y [m], yaw [rad])
        self.waypoints = [
            (1.94, -0.08, 0.0),
            (2.01, 1.77, 2.89),
            (0.67, 1.77, -1.67),
            (0.06, -0.03, 0.0),
        ]
        self.current_index = 0

        # Action bookkeeping
        self.goal_handle = None
        self.result_future = None

    # ------------------------------------------------------------------ helpers

    def _build_pose(self, x: float, y: float, theta: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.node.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y

        qx, qy, qz, qw = tf_transformations.quaternion_from_euler(0.0, 0.0, theta)
        pose.pose.orientation.x = qx  # (x, y, z, w) order!
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def _feedback_cb(self, msg):
        # You can log intermediate feedback here if desired
        pass

    def _send_goal(self, pose: PoseStamped) -> bool:
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = [pose]

        send_future = self.client.send_goal_async(goal_msg, feedback_callback=self._feedback_cb)
        rclpy.spin_until_future_complete(self.node, send_future)
        self.goal_handle = send_future.result()

        if not self.goal_handle.accepted:
            self.node.get_logger().error("Goal rejected by Nav2.")
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    # ------------------------------------------------------------------ state API

    def execute(self, blackboard):
        """Called by YASMIN when the state becomes active."""
        self.node.get_logger().info("[Patrol] Starting / continuing patrol…")

        # All waypoints visited?
        if self.current_index >= len(self.waypoints):
            self.node.get_logger().info("[Patrol] Patrol finished.")
            return "finished"

        # Build and send the next goal
        pose = self._build_pose(*self.waypoints[self.current_index])
        if not self._send_goal(pose):
            return "finished"

        # Block until Nav2 reports the result
        while not self.result_future.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)

        result = self.result_future.result()
        status: int = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().info(
                f"[Patrol] Waypoint {self.current_index + 1} reached."
            )
            self.current_index += 1
            return "reached"

        self.node.get_logger().error(
            f"[Patrol] Navigation failed with status {status}."
        )
        return "finished"
