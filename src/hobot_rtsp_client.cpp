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

#define CLEAR(x) memset(&(x), 0, sizeof(x))

#include <sys/sysinfo.h>
//#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
//#include "sensor_msgs/distortion_models.hpp"

extern "C" {
#include <linux/videodev2.h>  // Defines V4L2 format constants
#include <malloc.h>  // for memalign
#include <sys/mman.h>  // for mmap
#include <sys/stat.h>  // for stat
#include <unistd.h>  // for getpagesize()
#include <fcntl.h>  // for O_* constants and open()
}

#include <chrono>
#include <ctime>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <fstream>

#include "hobotlog/hobotlog.hpp"
#include "hobot_rtsp_client.hpp"
#include "rtspclient/SPSInfoMgr.h"
#include "rtspclient/rtspclient.h"
#include "rtspclient/buffmanager.h"



namespace hobot_rtsp_client
{

#define PUB_QUEUE_NUM 5

using vision::SPSInfoMgr;

RtspClient::RtspClient() {
}

RtspClient::~RtspClient() {
}

int RtspClient::init() {
  SetLogLevel(HOBOT_LOG_WARN);
  running_ = false;
  SPSInfoMgr::GetInstance().Init();
  return 0;
}


int RtspClient::configure(std::vector<rtsp_para_st>& parameters) {
    for (auto para : parameters) {
        para_config.push_back(para);
        RCLCPP_INFO(rclcpp::get_logger("rtsp_client"), "rtsp_url:%s",para.rtsp_url.c_str());
        RCLCPP_INFO(rclcpp::get_logger("rtsp_client"), "rtsp_transport:%s",para.rtsp_transport.c_str());
    }
    rtsp_clients_stat_.resize(para_config.size());
    q_buff_manager_.resize(para_config.size());
    return 0;
}

int RtspClient::start()
{
  connect_thread_ = std::make_shared<std::thread>(&RtspClient::connect, this);
  check_thread_ = std::make_shared<std::thread>(&RtspClient::CheckRtspState, this);
  return 0;
}

int RtspClient::stop()
{
  LOGW << "RtspPlugin Stop";
  running_ = false;
  LOGW << "process_thread_ Stop";
  eventLoopWatchVariable = 1;
  check_thread_->join();
  connect_thread_->join();
  //MediaPipeManager::GetInstance().DeInit();
  return 0;
}


void RtspClient::connect() {
  scheduler_ = BasicTaskScheduler::createNew();
  env_ = BasicUsageEnvironment::createNew(*scheduler_);
  LOGE << "\n\para_config size : " << para_config.size();
  int channel_number_ = para_config.size();
  for (int i = 0; i < channel_number_; ++i) {
      ourRTSPClient *client = nullptr;
      q_buff_manager_[i] = std::make_shared<ImagePublishManager>(); 
      q_buff_manager_[i]->SetPublisher(pre_node_ptr->create_publisher<img_msgs::msg::H26XFrame>(
                        para_config[i].topic.c_str(), PUB_QUEUE_NUM));
      client = openURL(*env_, "RTSPClient", para_config[i].rtsp_url.c_str(),
                      para_config[i].tcp_flag, para_config[i].frame_max_size,
                      ("channel" + std::to_string(i) + ".stream"),
                      para_config[i].save_stream, i);
      client->SetBuffManager(q_buff_manager_[i]);
      rtsp_clients_.push_back(client);
  }

  running_ = true;
  // All subsequent activity takes place within the event loop:
  env_->taskScheduler().doEventLoop(&eventLoopWatchVariable);
  for (int i = 0; i < channel_number_; ++i) {
      ourRTSPClient *client = rtsp_clients_[i];
      // operators cause crash if client is invalid
      if (rtsp_clients_stat_.at(i)) {
          client->sendTeardownCommand(*client->scs.session, NULL);
          Medium::close(client->scs.session);
      }
  }
  env_->reclaim();
  delete scheduler_;
}


void RtspClient::CheckRtspState() {
  sleep(10);  // wait for rtsp stream connect success
  LOGW << "Start CheckRtspState thread,running flag:" << running_;
  while (running_) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    uint64_t time_now = (uint64_t)tv.tv_sec;
    for (uint32_t i = 0; i < para_config.size(); ++i) {
      if (time_now - q_buff_manager_[i]->GetlastReadDataTime() < 10) {
        continue;
      }

      if (rtsp_clients_[i]) {  // if not start, maybe the destructor   
        rtsp_clients_[i]->Stop();
      }

      // reopen rtsp url
      ourRTSPClient *client = nullptr;
      client = openURL(*env_, "RTSPClient", para_config[i].rtsp_url.c_str(),
                       para_config[i].tcp_flag, para_config[i].frame_max_size,
                       ("channel" + std::to_string(i) + ".stream"),
                       para_config[i].save_stream, i);
      LOGI << "after reopen rtsp stream, channel:" << i;
      q_buff_manager_[i]->reset();
      client->SetBuffManager(q_buff_manager_[i]);
      rtsp_clients_[i] = client;
    }
    sleep(1);
  }
}

int RtspClient::set_rclNode(rclcpp::Node* node_ptr){
  pre_node_ptr = node_ptr;
  return 0;
}

int ImagePublishManager::UpdateData(std::shared_ptr<rtsp_client::VideoBuffer_ST> frame_ptr) {
  if (frameh26x_pub_ && h26x_publisher_) {
    frameh26x_pub_->index = frame_ptr->frame_id;
    frameh26x_pub_->dts.sec = frame_ptr->dts.tv_sec;
    frameh26x_pub_->dts.nanosec = frame_ptr->dts.tv_nsec;
    frameh26x_pub_->pts.sec = frame_ptr->pts.tv_sec;
    frameh26x_pub_->pts.nanosec = frame_ptr->pts.tv_nsec;
    memcpy(frameh26x_pub_->encoding.data(), frame_ptr->format.c_str(), frame_ptr->format.length());
    frameh26x_pub_->width = width_;
    frameh26x_pub_->height = height_;

    frameh26x_pub_->data.resize(frame_ptr->data_size);
    memcpy(&frameh26x_pub_->data[0], frame_ptr->buff, frame_ptr->data_size);
    h26x_publisher_->publish(*frameh26x_pub_);
  }
  std::unique_lock<std::mutex> lk(queue_mtx_);
  q_buff_empty_.push(frame_ptr);
  return 0;

}

}  // namespace usb_cam
