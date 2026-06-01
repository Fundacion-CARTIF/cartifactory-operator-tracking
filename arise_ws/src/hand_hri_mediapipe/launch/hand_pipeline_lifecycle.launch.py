from ast import literal_eval

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _parse_t_tcp_cam(value: str):
    """Parse and validate the TCP-to-camera homogeneous transform.

    The launch argument is received as a string, so it must be converted into a
    Python list before being passed as a ROS2 parameter. The target node expects
    a flat row-major 4x4 matrix with 16 numeric values.
    """
    try:
        parsed_value = literal_eval(value)
    except Exception as exc:
        raise ValueError(
            "Invalid T_tcp_cam argument. Expected a Python/YAML-like list with 16 values."
        ) from exc

    if not isinstance(parsed_value, (list, tuple)):
        raise ValueError("Invalid T_tcp_cam argument. Expected a list with 16 values.")

    if len(parsed_value) != 16:
        raise ValueError(
            f"Invalid T_tcp_cam argument. Expected 16 values, got {len(parsed_value)}."
        )

    return [float(item) for item in parsed_value]


def launch_setup(context, *args, **kwargs):
    """Create launch nodes after evaluating runtime launch arguments."""
    _ = args, kwargs

    camera_name = LaunchConfiguration("camera_name").perform(context)
    t_tcp_cam = _parse_t_tcp_cam(LaunchConfiguration("T_tcp_cam").perform(context))

    color_topic = f"/camera/{camera_name}/color/image_raw"
    depth_topic = f"/camera/{camera_name}/aligned_depth_to_color/image_raw"
    caminfo_topic = f"/camera/{camera_name}/aligned_depth_to_color/camera_info"

    tracked_topic = "/humans/hands/tracked"
    debug_image_topic = "/humans/hands/debug_image"
    tcp_pose_topic = "/ur/tcp_pose"

    hand_2d = Node(
        package="hand_hri_mediapipe",
        executable="hand_mediapipe_lifecycle_node",
        name="hand_mediapipe_node",
        output="screen",
        parameters=[{
            "image_topic": color_topic,
            "publish_debug_image": True,
            "debug_image_topic": debug_image_topic,
            "max_hands": 2,
            "model_complexity": 1,
            "min_detection_confidence": 0.6,
            "min_tracking_confidence": 0.6,
            "swap_handedness": True,
            "hand_selection": "both",
            "hand_command_topic": "/hand_change",
        }],
        emulate_tty=True,
    )

    hand_3d = Node(
        package="hand_hri_mediapipe",
        executable="hand_landmarks_3d_lifecycle_node",
        name="hand_landmarks_3d_node",
        output="screen",
        parameters=[{
            "tracked_topic": tracked_topic,
            "depth_topic": depth_topic,
            "caminfo_topic": caminfo_topic,
            "publish_pointcloud": False,
            "norm_margin": 0.05,
            "clamp_norm": True,
            "min_depth_m": 0.05,
            "max_depth_m": 2.0,
            "depth_hole_fill_radius": 2,
            "stale_hand_seconds": 1.0,
            "sphere_diameter": 0.012,
            "line_width": 0.004,
            "points_alpha": 0.9,
            "lines_alpha": 0.85,
        }],
        emulate_tty=True,
    )

    hand_palm_plane = Node(
        package="hand_hri_mediapipe",
        executable="hand_palm_plane_lifecycle_node",
        name="hand_palm_plane_node",
        output="screen",
        parameters=[{
            "tracked_topic": tracked_topic,
            "landmarks3d_suffix": "landmarks3d",
            "plane_suffix": "palm_plane_cam",
            "plane_indices": [0, 5, 9, 13, 17],
            "min_inliers": 3,
            "rmse_accept_mm": 20.0,
            "publish_last_good_if_rejected": True,
            "plane_flip_guard_enable": True,
            "plane_limit_step_enable": True,
            "plane_max_step_deg": 5.0,
            "plane_ema_enable": True,
            "plane_ema_alpha": 0.2,
            "plane_initial_face_camera": True,
            "stale_hand_seconds": 1.0,
            "publish_markers": True,
            "normal_marker_length_m": 0.10,
            "publish_mesh_marker": True,
            "mesh_use_all_valid_points": False,
        }],
        emulate_tty=True,
    )

    hand_cam_to_base = Node(
        package="hand_hri_mediapipe",
        executable="hand_cam_to_base_lifecycle_node",
        name="hand_cam_to_base_node",
        output="screen",
        parameters=[{
            "tracked_topic": tracked_topic,
            "tcp_pose_topic": tcp_pose_topic,
            "frame_out": "base",
            "landmarks3d_suffix_in": "landmarks3d",
            "landmarks3d_suffix_out": "landmarks3d_base",
            "plane_suffix_in": "palm_plane_cam",
            "plane_suffix_out": "palm_plane_base",
            "T_tcp_cam": t_tcp_cam,
            "filter_plane_normal": True,
            "filter_alpha": 0.15,
            "limit_step_enable": True,
            "max_step_deg": 2.0,
            "flip_guard_enable": True,
            "stale_hand_seconds": 1.0,
            "publish_markers": True,
        }],
        emulate_tty=True,
    )

    orchestrator = Node(
        package="hand_hri_mediapipe",
        executable="orchestrator",
        name="orchestrator",
        output="screen",
        parameters=[{
            "poll_period_s": 5.0,
            "lf_action_topic": "/lifecycle_control",
            "hand_action_topic": "/hand_change_action",
            "hand_topic": "/hand_change",
            "pipeline_stats_topic": "/stats/hand_pipeline",
            "image_topic": color_topic,
            "debug_image_topic": debug_image_topic,
            "tracked_topic": tracked_topic,
            "tcp_pose_topic": tcp_pose_topic,
            "fps_ema_alpha": 0.2,
            "camera_timeout_sec": 5.0,
            "tcp_timeout_sec": 5.0,
        }],
        emulate_tty=True,
    )

    return [
        hand_2d,
        hand_3d,
        hand_palm_plane,
        hand_cam_to_base,
        orchestrator,
    ]


def generate_launch_description():
    declare_camera_name = DeclareLaunchArgument(
        "camera_name",
        default_value="cameraD455",
        description="Camera namespace used to build RGB, depth and camera_info topics.",
    )

    declare_t_tcp_cam = DeclareLaunchArgument(
        "T_tcp_cam",
        default_value=(
            "["
            "1.000, -0.005, -0.005, -0.046, "
            "0.005,  1.000,  0.001, -0.064, "
            "0.005, -0.001,  1.000,  0.124, "
            "0.000,  0.000,  0.000,  1.000"
            "]"
        ),
        description=(
            "Homogeneous transform from robot TCP frame to camera frame. "
            "Flat row-major 4x4 matrix with 16 values."
        ),
    )

    return LaunchDescription([
        declare_camera_name,
        declare_t_tcp_cam,
        OpaqueFunction(function=launch_setup),
    ])
