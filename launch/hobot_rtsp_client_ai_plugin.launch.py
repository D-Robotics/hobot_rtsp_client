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
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode, ParameterFile

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory


def generate_launch_description():

    print("using rtsp camera")
    # using rtsp cam publish image

    rtsp_url_num_cmd = DeclareLaunchArgument(
        'hobot_rtsp_url_num',
        default_value='1',
        description='rtsp device number')
    
    rtsp_url_0_cmd = DeclareLaunchArgument(
        'hobot_rtsp_url_0',
        default_value='rtsp://admin:admin123@10.112.148.57:554/0',
        description='framerate')
    
    transport_0_cmd = DeclareLaunchArgument(
        'hobot_transport_0',
        default_value='udp',
        description='rtsp data transport tcp/upd')
    
    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name', 
        default_value='tros_container',
        description='the name of the container that launches all nodes')

    videobox_cmd_group = GroupAction([
        # component container
        Node(
            #condition=False,
            name='tros_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            exec_name='tros_container',
            parameters=[
                # 一定要传入这个参数，否则composition模式下参数传入不到nav2
                {'autostart':True}],
            arguments=['--ros-args', '--log-level', "warn"],
            # prefix=['xterm -e gdb -ex run --args'],
            output='screen')
    ])


    load_composable_nodes = LoadComposableNodes(
        #condition=IfCondition(use_composition),
        target_container=LaunchConfiguration('container_name'),
        composable_node_descriptions=[
            
            # RTSP码流接收
            ComposableNode(
                package='hobot_rtsp_client',
                plugin='HobotRtspClientNode',
                name='rtsp_client_node',
                parameters=[
                    {"rtsp_url_num": LaunchConfiguration('hobot_rtsp_url_num')},
                    {"rtsp_url_0": LaunchConfiguration('hobot_rtsp_url_0')},
                    {"transport_0": LaunchConfiguration('hobot_transport_0')}
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),

            # 感知节点：occ network
            
            # 解码节点
            ComposableNode(
                package='hobot_codec',
                plugin='HobotCodecNode',
                name='h264_decode',
                parameters=[
                    {'in_format': 'h264'},
                    {'in_mode': 'ros'},
                    {'sub_topic': '/rtsp_image_ch_0'},  # 畸变矫正后的左图
                    {'out_mode': 'ros'},
                    {'output_framerate': 10},
                    {'out_format': 'nv12'},
                    {'pub_topic': '/image'}
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),

            # 编码节点：畸变矫正后的左图编码成 jpeg 格式，用于可视化
            ComposableNode(
                package='dnn_node_example',
                plugin='DnnExampleNode',
                name='dnn_example',
                parameters=[
                    {"config_file": 'config/yolov8workconfig.json'},
                    {"dump_render_img": 0},
                    {"feed_type": 1},
                    {"is_shared_mem_sub": 0},
                    {"msg_pub_topic_name": "/hobot_dnn_detection"}

                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ) 
        ]
    )


    return LaunchDescription([
        rtsp_url_num_cmd,
        rtsp_url_0_cmd,
        transport_0_cmd,
        declare_container_name_cmd,
        videobox_cmd_group,
        load_composable_nodes,
    ])
