import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    human_detector_share = get_package_share_directory('human_detector')
    models_dir = os.path.join(human_detector_share, 'models')

    required_model_files = {
        'weights': os.path.join(models_dir, 'yolov4-tiny.weights'),
        'config': os.path.join(models_dir, 'yolov4-tiny.cfg'),
        'names': os.path.join(models_dir, 'coco.names'),
    }

    for description, file_path in required_model_files.items():
        if not os.path.exists(file_path):
            raise FileNotFoundError(
                f'Missing detector {description} file: {file_path}'
            )

    wave_interact = Node(
        package='human_detector',
        executable='wave_interact',
        name='wave_interact',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'goal_topic': '/interaction_goal_pose',
            'rviz_goal_topic': '/goal_pose',
            'cmd_vel_topic': '/cmd_vel',
            'person_target_max_age': 3.0,
            'base_linear_stop_threshold': 0.02,
            'base_angular_stop_threshold': 0.03,
            'base_stop_stable_duration': 0.40,
            'base_stop_timeout': 30.0,
            'arm_home_tolerance': 0.08,
            'arm_home_stable_duration': 0.30,
            'arm_home_timeout': 600.0,
            'arm_trajectory_timeout': 600.0,
            'arm_controller_action': (
                '/arm_r_controller/follow_joint_trajectory'
            ),
        }],
    )

    person_detector = Node(
        package='human_detector',
        executable='person_detector_node',
        name='person_detector_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'weights_path': required_model_files['weights'],
            'config_path': required_model_files['config'],
            'names_path': required_model_files['names'],
            'camera_topic': '/zedm/image',
            'depth_topic': '/zedm/depth/image_raw',
            'camera_info_topic': '/zedm/camera_info',
            'optical_frame': 'zedm_left_camera_optical_frame',
            'confidence_threshold': 0.50,
            'nms_threshold': 0.40,
            'detection_hold_frames': 3,
            'publish_annotated': True,
            'process_every_n_frames': 1,
            'network_input_size': 320,
            'depth_patch_radius': 10,
            'minimum_depth': 0.40,
            'maximum_depth': 8.00,
            'maximum_depth_age': 2.00,
        }],
    )

    return LaunchDescription([
        wave_interact,
        TimerAction(period=2.0, actions=[person_detector]),
    ])