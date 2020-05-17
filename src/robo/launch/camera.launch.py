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
import rclpy
###
#Serial No is needed
#To get Serial number:
#  rs-enumerate-devices  -S

def generate_launch_description():
    # config the serial number and base frame id of each camera
    t265_frame_prefix = "t265_"
    d435_frame_prefix = "d435_"
    #DeclareLaunchArgument(name="t265_serial_no",default_value="852212110070",description="t265 S/N")
    #DeclareLaunchArgument(name="d435_serial_no",default_value="848312071481",description="d435 S/N")
    
    t265_serial_no = LaunchConfiguration('t265_serial_no',default="852212110070")
    d435_serial_no = LaunchConfiguration('d435_serial_no',default="848312071481")

    t265_conf=get_package_share_directory('robo')+'/config/t265.yaml'
    d435_conf=get_package_share_directory('robo')+'/config/d435i.yaml'

    frames=["base_frame_id","depth_frame_id","infra1_frame_id","infra2_frame_id","color_frame_id","fisheye_frame_id","imu_gyro_frame_id","imu_accel_frame_id","depth_optical_frame_id","infra1_optical_frame_id","infra2_optical_frame_id","color_optical_frame_id","fisheye_optical_frame_id","gyro_optical_frame_id","accel_optical_frame_id"]
    t265_conf2={'serial_no':t265_serial_no}
    d435_conf2={'serial_no':d435_serial_no}
    for f in frames:
        t265_conf2[f]= t265_frame_prefix+f
        d435_conf2[f]= d435_frame_prefix+f
           
    rclpy.logging._root_logger.info(str(t265_conf2))
    n=[]
    n.append(Node(
        package='realsense_node',
        node_executable='realsense_node',
        node_namespace="/t265",
        output='screen',
        parameters=[t265_conf,t265_conf2],
        #arguments=[ '__log_level:=debug'],
        ))
    if True: n.append(Node(
        package='realsense_node',
        node_executable='realsense_node',
        node_namespace="/d435",
        output='screen',
        parameters=[d435_conf,d435_conf2],
              #  arguments=[('__log_level:=debug')],
        ))
    
    if False :n.append(Node(
            package='depthimage_to_laserscan',
            node_executable='depthimage_to_laserscan_node',
            node_name='scan',
            output='screen',
            parameters=[{'output_frame':'base_scan'}],
            remappings=[('depth','/d435/camera/depth/image_rect_raw'),
                        ('depth_camera_info', '/d435/camera/depth/camera_info')],
            ))

    if False: n.append(Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0.05', '0', '0', '0', "base_scan", "odom"]
            ))

    return launch.LaunchDescription(n)
