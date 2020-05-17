# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Demo for spawn_entity.

Launches Gazebo and spawns a model
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    n=[]
    n.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource([get_package_share_directory('gazebo_ros'), '/launch/gazebo.launch.py']),
                launch_arguments = {'world':get_package_share_directory('robo')+'/config/sim.world'}.items()
             ))

    tf_static=[
        "0.04 0 0.06  0 1.5 0 base_link d435_link",
        "0.04 0 0.06  0 0 0 base_link base_scan",
    ]

    for t in tf_static:
        n.append(Node(
                package='tf2_ros',
                node_executable='static_transform_publisher',
                output='screen',
                arguments=t.split(),
                parameters=[{'use_sim_time':True}]
                ))


    n.append(Node(
            package='depthimage_to_laserscan',
            node_executable='depthimage_to_laserscan_node',
            node_name='scan',
            output='screen',
            parameters=[{'output_frame':'base_scan'}],
            remappings=[('depth','/d435/camera/depth/image_raw'),
                        ('depth_camera_info', '/d435/camera/depth/camera_info')],
            ))

    return LaunchDescription(n)
