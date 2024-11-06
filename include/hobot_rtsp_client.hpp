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



#ifndef HOBOT_RTSP_CLIENT_HPP_
#define HOBOT_RTSP_CLIENT_HPP_

extern "C" {
#include <libavcodec/avcodec.h>
#include <linux/videodev2.h>
#include "libavformat/avformat.h"
#include "libavutil/avutil.h"
}

#include <chrono>
#include <memory>
#include <algorithm>
#include <sstream>
#include <iostream>
#include <string>
#include <vector>


#include <rclcpp/rclcpp.hpp>
#include "img_msgs/msg/h26_x_frame.hpp"
#include "rtspclient/rtspclient.h"
#include "rtspclient/buffmanager.h"


namespace hobot_rtsp_client
{

typedef struct
{
  std::string client_name;  // can be anything
  std::string rtsp_url;
  std::string rtsp_transport;
  std::string topic;
  int stimeout;
  int max_delay;
  bool zero_copy;
  bool tcp_flag;
  int frame_max_size;
  bool save_stream;
} rtsp_para_st;

class ImagePublishManager : public rtsp_client::BufferManager {
public:
    ImagePublishManager() : frameh26x_pub_(new img_msgs::msg::H26XFrame) {}
    virtual ~ImagePublishManager() {}

    virtual int UpdateData(std::shared_ptr<rtsp_client::VideoBuffer_ST> frame_ptr) override;
    int SetPublisher(rclcpp::Publisher<img_msgs::msg::H26XFrame>::SharedPtr publisher) {
      h26x_publisher_ = publisher;
      return 0;
    }
 private:
  img_msgs::msg::H26XFrame::UniquePtr frameh26x_pub_ = nullptr;
  rclcpp::Publisher<img_msgs::msg::H26XFrame>::SharedPtr h26x_publisher_ = nullptr;
};

class RtspClient
{
public:
  RtspClient();
  ~RtspClient();

  /// @brief Configure device, should be called before start
  int configure(std::vector<rtsp_para_st>& parameters);

  int set_rclNode(rclcpp::Node* node_ptr);


  /// @brief Start the configured device
  int init();

  /// @brief Start the configured device
  int start();

  int stop();

  void set_callback(std::function<void(char*, int)> func) {
    callback_func_ = func;
  }

  void connect();
  //void GetDeocdeFrame(std::shared_ptr<MediaPipeLine> pipeline, int channel);
  void CheckRtspState();


 private: 

  int m_fd;
  unsigned int m_number_of_buffers;

  int videoindex = -1;
  std::vector<rtsp_para_st> para_config;
  int status = 0; //0:尚未初始化，1：初始化成功，2：初始化失败，3：初始化成功，4：连接成功，5：连接失败，6：异常断开，7：断开连接，8：重连。

  std::function<void(char*, int)> callback_func_;
  std::vector<std::thread> threads_;
  std::vector<ourRTSPClient *> rtsp_clients_;
  TaskScheduler *scheduler_;
  UsageEnvironment *env_;
  bool running_;
  
  char eventLoopWatchVariable = 0;
  std::vector<bool> rtsp_clients_stat_;
  std::shared_ptr<std::thread> check_thread_;
  std::shared_ptr<std::thread> connect_thread_;

  std::vector<std::shared_ptr<ImagePublishManager>> q_buff_manager_;

  //std::queue<std::shared_ptr<VideoBuffer_ST>> q_buff_empty_;
  //std::vector<std::queue<std::shared_ptr<VideoBuffer_ST>>> q_v_buff_;

  rclcpp::Node* pre_node_ptr = nullptr;


};


}  // namespace usb_cam

#endif  // HOBOT_RTSP_CLIENT_HPP_
