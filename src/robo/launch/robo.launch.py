import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

###
#Serial No is needed
#To get Serial number:
#  rs-enumerate-devices  -S

def generate_launch_description():
    # config the serial number and base frame id of each camera

    t265_serial_no = LaunchConfiguration('t265_serial_no', default='852212110070')
    d435_serial_no = LaunchConfiguration('d435_serial_no', default='848312071481')
    t265_conf=get_package_share_directory('robo')+'/config/t265.yaml'
    d435i_conf=get_package_share_directory('robo')+'/config/d435i.yaml'

    n=[]
    n.append(Node(
        package='realsense_node',
        node_executable='realsense_node',
        node_namespace="/t265",
        output='screen',
        parameters=[t265_conf,{'serial_no':t265_serial_no, 
                    'base_frame_id': 't265_link'}],
        #arguments=[ '__log_level:=debug'],
        ))
    n.append(Node(
        package='realsense_node',
        node_executable='realsense_node',
        node_namespace="/d435",
        output='screen',
        parameters=[d435i_conf,{'serial_no':d435_serial_no, 
                    'base_frame_id': 'd435_link'}],
              #  arguments=[('__log_level:=debug')],
        ))
    
    n.append(Node(
            package='depthimage_to_laserscan',
            node_executable='depthimage_to_laserscan_node',
            node_name='scan',
            output='screen',
            parameters=[{'output_frame':'base_scan'}],
            remappings=[('depth','/d435/camera/depth/image_rect_raw'),
                        ('depth_camera_info', '/d435/camera/depth/camera_info')],
            ))

    """
    <node pkg="tf2_ros" type="static_transform_publisher" name="tf_camera1" args="0 0 0  0 0.62831853 0 t265_link t265_link_T" />
    <node pkg="tf2_ros" type="static_transform_publisher" name="tf_camera2" args="0 -0.01 -0.025 0 0.419 0 t265_link d435_link" />
    <node pkg="tf2_ros" type="static_transform_publisher" name="tf_camera3" args="0 0 -0.03 0 0 0 t265_link_T scan" />
    <node pkg="tf2_ros" type="static_transform_publisher" name="tf_camera4" args="-0.055 0 -0.03  0 0 0 t265_link_T base_link" />
    """
    tf_static=[
    "0      0     0  0 0.62831853 0 t265_link t265_link_T",
    "0  -0.01 -0.025 0 0.419      0 t265_link d435_link",
    "0      0 -0.03  0 0          0 t265_link_T scan",
    "-0.055 0 -0.03  0 0          0 t265_link_T base_link",
    ]
    
    for t in tf_static:
        n.append(Node(
                package='tf2_ros',
                node_executable='static_transform_publisher',
                output='screen',
                arguments=t.split()
                ))

    return LaunchDescription([
        n
    ])