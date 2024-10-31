# Copyright (c) 2024，D-Robotics.
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from ament_index_python.packages import get_package_prefix
import os

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'hobot_rtsp_url_num',
            default_value='1',
            description='rtsp device number'),
        DeclareLaunchArgument(
            'hobot_rtsp_url_0',
            default_value='rtsp://admin:admin123@10.112.148.57:554/0',
            description='framerate'),
        DeclareLaunchArgument(
            'hobot_transport_0',
            default_value='tcp',
            description='rtsp data transport tcp/upd'),

        Node(
            package='hobot_rtsp_client',
            executable='hobot_rtsp_client',
            name='hobot_rtsp_client',
            parameters=[
                {"rtsp_url_num": LaunchConfiguration('hobot_rtsp_url_num')},
                {"rtsp_url_0": LaunchConfiguration('hobot_rtsp_url_0')},
                {"transport_0": LaunchConfiguration('hobot_transport_0')}
            ],
            arguments=['--ros-args', '--log-level', 'info']
        )
    ])
