# ------------- ORIGINAL IMPORTS (unchanged) ----------------------------------
from yasmin import State
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from rclpy.duration import Duration
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
import cv2
import os
import math
import time
from rclpy.qos import qos_profile_sensor_data

from final_pkg.yolo_inference.yolo_utils import run_yolo_on_image
# -----------------------------------------------------------------------------


# ------------- NEW IMPORTS FOR SIREN (only these three lines added) ----------
import numpy as np
import subprocess
# -----------------------------------------------------------------------------


class RotateDetect(State):
    """Rotate in place while streaming camera frames through YOLO + siren."""

    ABNORMAL_CLASSES = {"bottle", "cup"}          # ← leave unchanged

    # ─────────── SIREN PARAMETERS (all new, rest of class untouched) ──────────
    _SR       = 44100          # sample-rate 44.1 kHz
    _DUR      = 1.5            # seconds
    _F_START  = 650            # sweep start [Hz]
    _F_END    = 1300           # sweep end   [Hz]
    _AMPL     = 16000          # 16-bit amplitude
    # -------------------------------------------------------------------------

    def __init__(self, node: Node):
        super().__init__(outcomes=["found", "next"])
        self.node = node

        # --------------------------- original init ----------------------------
        self._cmd_pub = self.node.create_publisher(Twist, "cmd_vel", 10)
        self._image_sub = self.node.create_subscription(
            Image, "/image_raw", self._image_cb, qos_profile_sensor_data
        )

        self._bridge = CvBridge()
        self._latest_image = None

        self._save_dir = os.path.expanduser("~/turtlebot_images")
        os.makedirs(self._save_dir, exist_ok=True)

        self._steps = 8
        self._step_angle_deg = 45
        self._ang_speed = math.radians(15)
        self._step_time = self._step_angle_deg / math.degrees(self._ang_speed)
        # ---------------------------------------------------------------------

        # ------------- PRE-GENERATE SIREN (new, does not affect logic) -------
        self._siren_pcm = self._generate_siren()
        # ---------------------------------------------------------------------

    # ────────────────────────── SIREN HELPERS (new) ───────────────────────────
    def _generate_siren(self) -> bytes:
        sr, dur = self._SR, self._DUR
        t = np.linspace(0, dur, int(sr * dur), endpoint=False)
        sweep_up = np.linspace(self._F_START, self._F_END, t.size // 2)
        sweep_dn = np.linspace(self._F_END, self._F_START, t.size // 2)
        freq = np.concatenate([sweep_up, sweep_dn])
        phase = 2 * np.pi * np.cumsum(freq) / sr
        samples = (self._AMPL * np.sin(phase)).astype(np.int16)
        return samples.tobytes()

    def _play_siren(self):
        """Fire-and-forget siren via ALSA aplay."""
        subprocess.Popen(
            [
                "aplay",
                "-q",
                "-f", "S16_LE",
                "-r", str(self._SR),
                "-c", "1",
                "-t", "raw",
                "-"
            ],
            stdin=subprocess.PIPE
        ).stdin.write(self._siren_pcm)
    # -------------------------------------------------------------------------

    # --------------- rest of original class stays as-is ----------------------
    def _image_cb(self, msg: Image):
        self._latest_image = msg

    def _publish_twist(self, z_vel: float):
        twist = Twist()
        twist.angular.z = z_vel
        self._cmd_pub.publish(twist)

    def _stop_robot(self):
        self._publish_twist(0.0)

    def _save_debug_frame(self, frame, step_idx, detections):
        for det in detections:
            bb = det["bounding_box"]
            cv2.rectangle(
                frame,
                (int(bb["x1"]), int(bb["y1"])),
                (int(bb["x2"]), int(bb["y2"])),
                (0, 255, 0),
                2,
            )
        path = os.path.join(self._save_dir, f"detect_{step_idx+1:02d}.png")
        cv2.imwrite(path, frame)
        self.node.get_logger().debug(f"[RotateDetect] Saved {path}")

    def execute(self, blackboard):
        self.node.get_logger().info("[RotateDetect] Begin 360° scan …")

        for step in range(self._steps):
            self.node.get_logger().info(
                f"[RotateDetect] Step {step + 1}/{self._steps} "
                f"(≈ {self._step_angle_deg}°)"
            )

            self._publish_twist(self._ang_speed)
            start_time = time.time()

            while time.time() - start_time < self._step_time:
                rclpy.spin_once(self.node, timeout_sec=0.05)

                if self._latest_image is None:
                    continue

                frame = self._bridge.imgmsg_to_cv2(
                    self._latest_image, desired_encoding="bgr8"
                )
                detections = run_yolo_on_image(frame)

                self._save_debug_frame(frame.copy(), step, detections)

                abnormal = [
                    d for d in detections if d["class_name"] in self.ABNORMAL_CLASSES
                ]
                if abnormal:
                    self.node.get_logger().info(
                        f"[RotateDetect] Abnormal object detected: "
                        f"{abnormal[0]['class_name']} "
                        f"({abnormal[0]['confidence']:.2f})"
                    )
                    self._stop_robot()

                    # -------- single added line to trigger siren --------------
                    self._play_siren()
                    # ----------------------------------------------------------

                    blackboard.detected = abnormal[0]
                    return "found"

            self._stop_robot()
            self.node.get_clock().sleep_for(Duration(seconds=0.3))

        self.node.get_logger().info("[RotateDetect] No abnormal objects found.")
        return "next"
