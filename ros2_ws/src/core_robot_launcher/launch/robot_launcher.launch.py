from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Create launch description
    ld = LaunchDescription()
    
    # Add screen node from core_control_screen package
    screen_node = Node(
        package='core_control_screen',
        executable='screen',
        name='screen',
        output='screen',
        remappings=[
            ('image_raw', '/normal_cam/image_raw'),
            ('top_view_image', '/top_view/image'),
        ],
    )
    
    # Add creator node from top_view_creator package
    creator_node = Node(
        package='top_view_creator',
        executable='creator',
        name='creator',
        output='screen',
        remappings=[
            ('image1', '/wide_view_camera_0/image_raw'),
            ('image2', '/wide_view_camera_1/image_raw'),
        ],
        parameters=[{
            'camera_height1': 0.7,
            'camera_height2': 0.7,
            'camera_x1': 0.4,
            'camera_x2': -0.4,
            'camera_y1': 0.4,
            'camera_y2': -0.4,
            'camera_angle1': 45.0,
            'camera_angle2': -135.0,
            'fov_degrees': 190.0,
            'output_width': 300,
            'output_height': 300,
            'ground_width': 4.0,
            'ground_height': 4.0,
        }]
    )
    
    # Include camera.launch.py from core_robot_launcher package
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('core_robot_launcher'), 
                         'launch', 
                         'camera.launch.py')
        ])
    )
    
    # Add all nodes and includes to the launch description
    ld.add_action(screen_node)
    ld.add_action(creator_node)
    ld.add_action(camera_launch)
    
    return ld