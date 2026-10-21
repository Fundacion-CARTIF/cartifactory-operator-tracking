#!/usr/bin/env python3
from typing import Optional
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data

from sensor_msgs.msg import Image
from hand_hri_mediapipe_msgs.msg import PipelineStats


class PipelineMonitorNode(Node):
    """
    Aggregates event-driven stats from:
      - /stats/detector
      - /stats/matcher

    Also computes:
      - fps_input from image_topic
      - fps_processed from debug_image_topic
      - avg_inference_ms (approx end-to-end processing latency) from debug_image.header.stamp
      - camera_available from image_topic timeout

    Publishes:
      - /stats/pipeline
    """

    def __init__(self):
        super().__init__("pipeline_monitor")

        self.declare_parameter("detector_stats_topic", "/stats/detector")
        self.declare_parameter("matcher_stats_topic", "/stats/matcher")
        self.declare_parameter("pipeline_stats_topic", "/stats/pipeline")

        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("detections_topic", "/detections")

        self.declare_parameter("fps_ema_alpha", 0.2)
        self.declare_parameter("latency_window_size", 30)
        self.declare_parameter("camera_timeout_sec", 5.0)
        self.declare_parameter("camera_check_period_sec", 1.0)

        self.detector_stats_topic = self.get_parameter("detector_stats_topic").value
        self.matcher_stats_topic = self.get_parameter("matcher_stats_topic").value
        self.pipeline_stats_topic = self.get_parameter("pipeline_stats_topic").value

        self.image_topic = self.get_parameter("image_topic").value
        self.detections_topic = self.get_parameter("detections_topic").value
        self.debug_image_topic = f"{self.detections_topic}/image"

        self.fps_ema_alpha = float(self.get_parameter("fps_ema_alpha").value)
        self.latency_window_size = int(self.get_parameter("latency_window_size").value)
        self.camera_timeout_sec = float(self.get_parameter("camera_timeout_sec").value)
        self.camera_check_period_sec = float(self.get_parameter("camera_check_period_sec").value)

        qos_stats = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.sub_detector = self.create_subscription(
            PipelineStats, self.detector_stats_topic, self.on_detector_stats, qos_stats
        )
        self.sub_matcher = self.create_subscription(
            PipelineStats, self.matcher_stats_topic, self.on_matcher_stats, qos_stats
        )

        self.pub_pipeline = self.create_publisher(
            PipelineStats, self.pipeline_stats_topic, qos_stats
        )

        self.sub_input_image = self.create_subscription(
            Image, self.image_topic, self.on_input_image, qos_profile_sensor_data
        )
        self.sub_debug_image = self.create_subscription(
            Image, self.debug_image_topic, self.on_debug_image, qos_profile_sensor_data
        )

        self._detector_stats: Optional[PipelineStats] = None
        self._matcher_stats: Optional[PipelineStats] = None
        self._last_published_state = None

        self._last_input_t = None
        self._last_processed_t = None
        self._fps_input_ema = 0.0
        self._fps_processed_ema = 0.0

        self._latency_ms_values = deque(maxlen=max(1, self.latency_window_size))
        self._avg_latency_ms = 0.0

        self._last_camera_msg_time = None
        self._camera_available = False

        self._camera_timer = self.create_timer(
            self.camera_check_period_sec,
            self._check_camera_timeout
        )

        self.get_logger().info(
            f"Pipeline monitor ready.\n"
            f"  detector_stats_topic: {self.detector_stats_topic}\n"
            f"  matcher_stats_topic:  {self.matcher_stats_topic}\n"
            f"  pipeline_stats_topic: {self.pipeline_stats_topic}\n"
            f"  image_topic:          {self.image_topic}\n"
            f"  debug_image_topic:    {self.debug_image_topic}"
        )

    def _update_fps_ema(self, now_sec: float, last_t: Optional[float], current_ema: float):
        if last_t is None:
            return current_ema, now_sec

        dt = max(now_sec - last_t, 1e-6)
        inst_fps = 1.0 / dt
        a = min(max(self.fps_ema_alpha, 0.0), 1.0)
        new_ema = (1.0 - a) * current_ema + a * inst_fps
        return new_ema, now_sec

    def on_input_image(self, msg: Image):
        _ = msg
        now_sec = time_now_sec(self)
        self._fps_input_ema, self._last_input_t = self._update_fps_ema(
            now_sec, self._last_input_t, self._fps_input_ema
        )
        self._last_camera_msg_time = now_sec

        prev = self._camera_available
        self._camera_available = True
        if prev != self._camera_available:
            self._maybe_publish_global()
        else:
            self._maybe_publish_global()

    def on_debug_image(self, msg: Image):
        now_sec = time_now_sec(self)
        self._fps_processed_ema, self._last_processed_t = self._update_fps_ema(
            now_sec, self._last_processed_t, self._fps_processed_ema
        )

        src_stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1e9
        latency_ms = max((now_sec - src_stamp_sec) * 1000.0, 0.0)

        self._latency_ms_values.append(latency_ms)
        if len(self._latency_ms_values) > 0:
            self._avg_latency_ms = sum(self._latency_ms_values) / len(self._latency_ms_values)

        self._maybe_publish_global()

    def _check_camera_timeout(self):
        now_sec = time_now_sec(self)

        prev = self._camera_available
        if self._last_camera_msg_time is None:
            self._camera_available = False
        else:
            self._camera_available = (now_sec - self._last_camera_msg_time) <= self.camera_timeout_sec

        if prev != self._camera_available:
            self._maybe_publish_global()

    def on_detector_stats(self, msg: PipelineStats):
        self._detector_stats = msg
        self._maybe_publish_global()

    def on_matcher_stats(self, msg: PipelineStats):
        self._matcher_stats = msg
        self._maybe_publish_global()

    def _build_global_msg(self) -> PipelineStats:
        msg = PipelineStats()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.node_name = "pipeline_monitor"

        det = self._detector_stats
        mat = self._matcher_stats

        msg.frames_dropped = int(det.frames_dropped) if det is not None else 0

        msg.fps_input = float(int(round(self._fps_input_ema)))
        msg.fps_processed = float(int(round(self._fps_processed_ema)))
        msg.avg_inference_ms = float(int(round(self._avg_latency_ms)))

        msg.action_goals_received = int(mat.action_goals_received) if mat is not None else 0
        msg.action_goals_accepted = int(mat.action_goals_accepted) if mat is not None else 0
        msg.action_goals_rejected = int(mat.action_goals_rejected) if mat is not None else 0
        msg.action_goals_canceled = int(mat.action_goals_canceled) if mat is not None else 0
        msg.action_goals_succeeded = int(mat.action_goals_succeeded) if mat is not None else 0
        msg.action_goals_failed = int(mat.action_goals_failed) if mat is not None else 0

        msg.match_requests = int(mat.match_requests) if mat is not None else 0
        msg.match_success = int(mat.match_success) if mat is not None else 0
        msg.match_fail = int(mat.match_fail) if mat is not None else 0

        msg.camera_available = bool(self._camera_available)

        return msg

    def _state_fingerprint(self, msg: PipelineStats):
        return (
            int(msg.frames_dropped),

            int(msg.action_goals_received),
            int(msg.action_goals_accepted),
            int(msg.action_goals_rejected),
            int(msg.action_goals_canceled),
            int(msg.action_goals_succeeded),
            int(msg.action_goals_failed),

            int(msg.match_requests),
            int(msg.match_success),
            int(msg.match_fail),

            int(round(float(msg.fps_input))),
            int(round(float(msg.fps_processed))),
            int(round(float(msg.avg_inference_ms))),

            bool(msg.camera_available),
        )

    def _maybe_publish_global(self):
        global_msg = self._build_global_msg()
        new_state = self._state_fingerprint(global_msg)

        if self._last_published_state is None or new_state != self._last_published_state:
            self.pub_pipeline.publish(global_msg)
            self._last_published_state = new_state


def time_now_sec(node: Node) -> float:
    now = node.get_clock().now().to_msg()
    return float(now.sec) + float(now.nanosec) / 1e9


def main():
    rclpy.init()
    node = PipelineMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()