#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    camera_name = LaunchConfiguration("camera_name")

    declare_camera_name = DeclareLaunchArgument(
        "camera_name",
        default_value="cameraD435f",
        description="Nombre de la cámara RealSense"
    )

    color_topic = ["/camera/", camera_name, "/color/image_raw"]
    depth_topic = ["/camera/", camera_name, "/aligned_depth_to_color/image_raw"]
    caminfo_topic = ["/camera/", camera_name, "/aligned_depth_to_color/camera_info"]

    hand_2d = Node(
        package="hand_hri_mediapipe",
        executable="hand_mediapipe_node",
        name="hand_mediapipe_node",
        output="screen",
        # prefix="/home/roboticaindustrial/Documentos/Robot_Camara3d/codigos-transformaciones-camara-robot/ur_ws/.venv/bin/python ",
        parameters=[
            {
                "image_topic": color_topic,
                "publish_debug_image": True,
                "debug_image_topic": "/humans/hands/debug_image",
                "max_hands": 1,
                "model_complexity": 1,
                "min_detection_confidence": 0.6,
                "min_tracking_confidence": 0.6,
                "swap_handedness": True,
            }
        ],
        emulate_tty=True,
    )
    hand_3d = Node(
        package="hand_hri_mediapipe",
        executable="hand_landmarks_3d_node",
        name="hand_landmarks_3d_node",
        output="screen",
        # prefix="/home/roboticaindustrial/Documentos/Robot_Camara3d/codigos-transformaciones-camara-robot/ur_ws/.venv/bin/python ",
        parameters=[
            {
                "tracked_topic": "/humans/hands/tracked",
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
            }
        ],
        emulate_tty=True,
    )

    hand_palm_plane = Node(
        package="hand_hri_mediapipe",
        executable="hand_palm_plane_node",
        name="hand_palm_plane_node",
        output="screen",
        # prefix="/home/roboticaindustrial/Documentos/Robot_Camara3d/codigos-transformaciones-camara-robot/ur_ws/.venv/bin/python ",
        parameters=[
            {
                "tracked_topic": "/humans/hands/tracked",

                "landmarks3d_suffix": "landmarks3d",
                "plane_suffix": "palm_plane_cam",

                "plane_indices": [0, 5, 9, 13, 17],

                "min_inliers": 3,
                "rmse_accept_mm": 20.0,

                # SOLO NORMAL ESTABLE (cyan)
                "plane_flip_guard_enable": True,
                "plane_limit_step_enable": True,
                "plane_max_step_deg": 5.0,
                "plane_ema_enable": True,
                "plane_ema_alpha": 0.2,
                "plane_initial_face_camera": True,

                "publish_markers": True,
                "normal_marker_length_m": 0.10,

                "publish_mesh_marker": True,
                "mesh_use_all_valid_points": False,
            }
        ],
        emulate_tty=True,
    )

    hand_cam_to_base = Node(
        package="hand_hri_mediapipe",
        executable="hand_cam_to_base_node",
        name="hand_cam_to_base_node",
        output="screen",
        parameters=[{
            "tracked_topic": "/humans/hands/tracked",
            "tcp_pose_topic": "/ur/tcp_pose",
            "frame_out": "base",
        }],
    )

    return LaunchDescription([
        declare_camera_name,
        hand_2d,
        hand_3d,
        hand_palm_plane,
        hand_cam_to_base,
    ])