// Copyright (c) 2024，D-Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#ifndef HOBOT_RTSP_CLIENT_NODE_HPP_
#define HOBOT_RTSP_CLIENT_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "hobot_rtsp_client.hpp"
#include "img_msgs/msg/h26_x_frame.hpp"


namespace hobot_rtsp_client
{

class HobotRtspClientNode : public rclcpp::Node
{
public:
  explicit HobotRtspClientNode(const rclcpp::NodeOptions & node_options);
  ~HobotRtspClientNode();

  void init();
  void get_params();

 private:

  std::vector<std::shared_ptr<RtspClient>> client_;
  std::vector<rclcpp::TimerBase::SharedPtr> timer_;
  std::vector<rtsp_para_st> rtsp_config;

  // parameters
  std::string frame_id_;
  std::string io_method_name_;  // hbmem zero mem copy
  int m_bIsInit;
};
}  // namespace hobot_rtsp_client
#endif  // HOBOT_RTSP_CLIENT_NODE_HPP_
