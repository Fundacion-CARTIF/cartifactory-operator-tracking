from setuptools import setup
from glob import glob

package_name = "hand_hri_mediapipe"

setup(
    name=package_name,
    version="0.0.1",
    packages=[
        package_name,
    ],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="roboticsUR",
    description="MediaPipe Hands ROS4HRI publisher",
    license="Apache License 2.0",
    entry_points={
        "console_scripts": [
            "hand_mediapipe_lifecycle_node = hand_hri_mediapipe.hand_mediapipe_lifecycle_node:main",
            "hand_landmarks_3d_lifecycle_node = hand_hri_mediapipe.hand_landmarks_3d_lifecycle_node:main",
            "hand_palm_plane_lifecycle_node = hand_hri_mediapipe.hand_palm_plane_lifecycle_node:main",
            "hand_cam_to_base_lifecycle_node = hand_hri_mediapipe.hand_cam_to_base_lifecycle_node:main",
            "hand_mediapipe_node = hand_hri_mediapipe.hand_mediapipe_node:main",
            "hand_landmarks_3d_node = hand_hri_mediapipe.hand_landmarks_3d_node:main",
            "hand_palm_plane_node = hand_hri_mediapipe.hand_palm_plane_node:main",
            "hand_cam_to_base_node = hand_hri_mediapipe.hand_cam_to_base_node:main",
            "orchestrator = hand_hri_mediapipe.orchestrator_node:main",
            "test_client = hand_hri_mediapipe.action_client_test:main",
            "hand_client = hand_hri_mediapipe.action_hand_test:main",
            'db_client = hand_hri_mediapipe.database_node:main',
        ],
    },
)