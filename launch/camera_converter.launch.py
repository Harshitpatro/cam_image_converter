from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments for topic names
        DeclareLaunchArgument(
            'input_topic',
            default_value='/image_raw',
            description='Input camera topic'
        ),
        DeclareLaunchArgument(
            'output_topic',
            default_value='/image_converter/output_image',
            description='Output image topic'
        ),
        # Launch usb_cam node with laptop webcam parameters
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[{
                'video_device': '/dev/video0',
                'image_width': 640,
                'image_height': 480,
                'framerate': 30.0,
                # Changed from 'mjpeg' to 'yuyv' because mjpeg is unsupported by your driver
                'pixel_format': 'yuyv',
                'camera_frame_id': 'laptop_camera',
                'io_method': 'mmap',
                'auto_exposure': True,
                'exposure': 100,
            }],
        ),
        # Launch image_converter node
        Node(
            package='image_converter_pkg',
            executable='image_converter_node',
            name='image_conversion',
            output='screen',
            parameters=[{
                'input_topic': LaunchConfiguration('input_topic'),
                'output_topic': LaunchConfiguration('output_topic')
            }]
        ),
    ])
