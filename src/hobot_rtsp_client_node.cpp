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

#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "hobot_rtsp_client_node.hpp"

namespace hobot_rtsp_client
{

HobotRtspClientNode::HobotRtspClientNode(const rclcpp::NodeOptions & node_options)
: Node("hobot_rtsp_client", node_options){
  // declare params
  get_params();
  init();
}

HobotRtspClientNode::~HobotRtspClientNode()
{
  RCLCPP_WARN(this->get_logger(), "Shutting down");
  client_.clear();
  //m_timer.clear();
}


void HobotRtspClientNode::get_params()
{
  this->declare_parameter("rtsp_url_num", 1);
  //this->declare_parameter("stimeout",5000000);
  //this->declare_parameter("max_delay",3000000);


  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);

  // 获取数组参数
  auto rtsp_url_num = parameters_client->get_parameter("rtsp_url_num", 0);



  RCLCPP_INFO(rclcpp::get_logger("rtsp_client"), "rtsp_url_num: %d", rtsp_url_num);
  std::vector<std::string> rtsp_urls;
  if (rtsp_url_num > 0) {

    rtsp_config.resize(rtsp_url_num);
    //for (int i = 0; i < rtsp_url_num.as_int(); ++i) {
    for (int i = 0; i < rtsp_url_num; ++i) {
      std::string param_name = "rtsp_url_" + std::to_string(i);
      this->declare_parameter(param_name, "");  // 480);
      rtsp_config[i].rtsp_url = parameters_client->get_parameter(param_name, std::string(""));
      RCLCPP_INFO(rclcpp::get_logger("rtsp_client"), "%s:%s",param_name.c_str(),rtsp_config[i].rtsp_url.c_str());
      param_name = "transport_" + std::to_string(i);
      this->declare_parameter(param_name, "udp");
      rtsp_config[i].rtsp_transport = parameters_client->get_parameter(param_name, std::string(""));
      RCLCPP_INFO(rclcpp::get_logger("rtsp_client"), "%s:%s",param_name.c_str(),rtsp_config[i].rtsp_transport.c_str());
      if (rtsp_config[i].rtsp_transport == "tcp") {
        rtsp_config[i].tcp_flag = true;
      } else {
        rtsp_config[i].tcp_flag = false;
      }
      //parameters_client->get_parameter<bool>("tcp_flag", rtsp_config[i].tcp_flag);
      rtsp_config[i].topic = "rtsp_image_ch_" + std::to_string(i);
      RCLCPP_INFO(rclcpp::get_logger("rtsp_client"), "topic:%s",rtsp_config[i].topic.c_str());
    }
  }
  //auto stimeout = parameters_client->get_parameter("stimeout", 0);
  //auto max_delay = parameters_client->get_parameter("max_delay", 0);
  int stimeout = 5000000;
  int max_delay = 3000000;
  for (auto& cfg : rtsp_config) {
    cfg.stimeout = stimeout;
    cfg.max_delay = max_delay;
    cfg.frame_max_size = 200; //200K
    cfg.save_stream = false;
  }

  return;
}


void HobotRtspClientNode::init()
{
  if (m_bIsInit) return;
  auto client_ptr = std::make_shared<RtspClient>();
  client_ptr->set_rclNode(this);
  client_ptr->configure(rtsp_config);
  client_ptr->init();
  // start the camera
  client_ptr->start();
  client_.push_back(client_ptr);
  RCLCPP_INFO_STREAM(this->get_logger(), "RtspClientNode init");
}

}  // namespace usb_cam


//#include "rclcpp_components/register_node_macro.hpp"
//RCLCPP_COMPONENTS_REGISTER_NODE(usb_cam::HobotUsbCamNode)
