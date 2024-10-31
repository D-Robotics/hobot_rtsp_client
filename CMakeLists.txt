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

cmake_minimum_required(VERSION 3.5)
project(hobot_rtsp_client)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wl,--no-as-needed)
endif()

set(CMAKE_BUILD_TYPE RelWithDebInfo)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(img_msgs REQUIRED)


include_directories("include"
  "include/rtspclient"
  "include/mediapipemanager"
  ./deps/live555/include/
  ./deps/hobotlog/include/
)


link_directories(
  ./deps/live555/lib
  ./deps/hobotlog/lib
)

set(LIVE_LIBRARIES
  hobotlog
  liveMedia
  BasicUsageEnvironment
  groupsock
  UsageEnvironment
)


add_executable(${PROJECT_NAME}
  src/rtspclient/AudioG711Sink.cpp
  src/rtspclient/Base64.cpp
  src/rtspclient/H264Sink.cpp
  src/rtspclient/H265Sink.cpp
  src/rtspclient/hevc_sps.cpp
  src/rtspclient/rtspclient.cpp
  src/rtspclient/sps_pps.c
  src/rtspclient/SPSInfoMgr.cpp
  src/hobot_rtsp_client_exec.cpp
  src/hobot_rtsp_client.cpp
  src/hobot_rtsp_client_node.cpp)


target_link_libraries(${PROJECT_NAME}
  ${LIVE_LIBRARIES}
)

ament_target_dependencies(
  ${PROJECT_NAME} 
  rclcpp
  img_msgs
)

# Install libraries
install(DIRECTORY ./deps/live555/lib/
DESTINATION lib/)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
