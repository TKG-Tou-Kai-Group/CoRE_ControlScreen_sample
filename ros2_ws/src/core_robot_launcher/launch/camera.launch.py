# my_camera_launch.py

import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from subprocess import getoutput

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_node',
            namespace='normal_cam',
            output='screen',
            parameters=[
              {'video_device': "/dev/normal_camera_0"},
              {'pixel_format': 'mjpeg2rgb'},
              {'image_width': 1280},
              {'image_height': 720},
              {'framerate': 30.0}
            ],
        ),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_node',
            namespace='wide_view_camera_0',
            output='screen',
            parameters=[
              {'video_device': "/dev/wide_view_camera_0"},
              {'pixel_format': 'mjpeg2rgb'},
              {'image_width': 640},
              {'image_height': 360},
              {'framerate': 30.0}
            ],
        ),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_node',
            namespace='wide_view_camera_1',
            output='screen',
            parameters=[
              {'video_device': "/dev/wide_view_camera_1"},
              {'pixel_format': 'mjpeg2rgb'},
              {'image_width': 640},
              {'image_height': 360},
              {'framerate': 30.0}
            ],
        ),
    ])
