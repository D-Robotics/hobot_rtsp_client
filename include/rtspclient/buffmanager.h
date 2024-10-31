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
#ifndef INCLUDE_BUFFMANAGER_H_
#define INCLUDE_BUFFMANAGER_H_

#include <fstream>
#include <iostream>
#include <memory>
#include <vector>
#include <string>
#include <tuple>
#include <queue>
#include <map>
#include <thread>
#include <mutex>

namespace rtsp_client {

typedef struct video_buffer_s {
  uint64_t timestamp;
  uint32_t frame_id;
  int width;
  int height;
  int stride;
  uint32_t buff_size;
  uint32_t data_size;
  bool key_frame;
  std::string format;
  struct timespec pts;
  struct timespec dts;
  void* buff;
  ~video_buffer_s() {
    if (buff != NULL) {
      free(buff);
      buff = NULL;
    }
  }
} VideoBuffer_ST;


class BufferManager {
 public:
  BufferManager() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    last_recv_data_time_ = (uint64_t)tv.tv_sec;
  };
  virtual ~BufferManager() {
    std::unique_lock<std::mutex> lk(queue_mtx_);
    while(!q_buff_empty_.empty()) {
      q_buff_empty_.pop();
    }
    while(!q_buff_.empty()) {
      q_buff_.pop();
    }
    //q_buff_empty_.clear();
    //q_buff_.clear();

  }
  virtual int UpdateData(std::shared_ptr<VideoBuffer_ST> frame_ptr) {
    return 0;
  }
  int Input(std::shared_ptr<VideoBuffer_ST> frame_ptr) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    last_recv_data_time_ = (uint64_t)tv.tv_sec;
    return UpdateData(frame_ptr);
  }

  std::shared_ptr<VideoBuffer_ST> GetBuffDef() {
    std::shared_ptr<VideoBuffer_ST> buf_ptr;
    std::unique_lock<std::mutex> lk(queue_mtx_);
    if (q_buff_empty_.size() > 0) {
      buf_ptr = q_buff_empty_.front();
      q_buff_empty_.pop();
    } else {
      buf_ptr = std::make_shared<VideoBuffer_ST>();
      buf_ptr->buff_size = buff_size;
      buf_ptr->buff = malloc(buf_ptr->buff_size);
    }
    return buf_ptr;
  }
    
  int SetBuffDef(std::shared_ptr<VideoBuffer_ST> buf_ptr) {
    std::unique_lock<std::mutex> lk(queue_mtx_);
    q_buff_empty_.push(buf_ptr);
    return 0;
  }

  int SetResolution(int width, int height) {
    width_ = width;
    height_ = height;
    return 0;
  }

  int reset() {
    std::unique_lock<std::mutex> lk(queue_mtx_);
    while(!q_buff_.empty()) {
      q_buff_.pop();
    }
    //q_buff_.clear();
    struct timeval tv;
    gettimeofday(&tv, NULL);
    last_recv_data_time_ = (uint64_t)tv.tv_sec;
    return 0;
  }

  uint64_t GetlastReadDataTime() { return last_recv_data_time_; }

 protected:
  std::queue<std::shared_ptr<VideoBuffer_ST>> q_buff_empty_;
  std::queue<std::shared_ptr<VideoBuffer_ST>> q_buff_;
  std::mutex queue_mtx_;
  int buff_size = 256*1024;
  int width_;
  int height_;
  uint64_t last_recv_data_time_;

};

}  // namespace rtsp_client

#endif  // INCLUDE_BUFFMANAGER_H_