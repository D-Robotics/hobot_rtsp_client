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

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory


def generate_launch_description():
    camera_node = None

    print("using rtsp camera")
    # using rtsp cam publish image

    rtsp_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_rtsp_client'),
                'launch/hobot_rtsp_client.launch.py')),
        launch_arguments={
            'hobot_rtsp_url_num': '1',
            'hobot_rtsp_url_0': 'rtsp://admin:admin123@10.112.148.57:554/0',
            'hobot_transport_0': 'tcp'
        }.items()
    )

        # h264->nv12
    h264_codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_codec'),
                'launch/hobot_codec_decode.launch.py')),
        launch_arguments={
            #'codec_in_mode': 'shared_mem',
            'codec_in_mode': 'ros',
            'codec_in_format': 'h264',
            #'codec_out_mode': 'ros',
            'codec_out_mode': 'shared_mem',
            'codec_out_format': 'nv12',
            #'codec_sub_topic': '/hbmem_img',
            'codec_sub_topic': '/rtsp_image_ch_0',
            'codec_pub_topic': '/image_decode'
        }.items()
    )

    # nv12->jpeg
    jpeg_codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_codec'),
                'launch/hobot_codec_encode.launch.py')),
        launch_arguments={
            'codec_in_mode': 'shared_mem',
            #'codec_in_mode': 'ros',
            'codec_in_format': 'nv12',
            'codec_out_mode': 'ros',
            'codec_out_format': 'jpeg',
            'codec_jpg_quality': '85.0',
            'codec_sub_topic': '/image_decode',
            'codec_pub_topic': '/image_mjpeg'
        }.items()
    )
    # web
    web_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('websocket'),
                'launch/websocket.launch.py')),
        launch_arguments={
            'websocket_image_topic': '/image_mjpeg',
            'websocket_only_show_image': 'True'
        }.items()
    )


    return LaunchDescription([
        # 启动零拷贝环境配置node
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('hobot_shm'),
                    'launch/hobot_shm.launch.py'))
        ),
        rtsp_node,
        h264_codec_node,
        jpeg_codec_node,
        web_node
        # image codec
    ])
