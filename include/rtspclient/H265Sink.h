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
#ifndef INCLUDE_RTSPCLIENT_H265SINK_H_
#define INCLUDE_RTSPCLIENT_H265SINK_H_

#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include "BasicUsageEnvironment.hh"
#include "H265VideoRTPSource.hh"
#include "liveMedia.hh"
#include "buffmanager.h"

// Define a data sink (a subclass of "MediaSink") to receive the data for each
// subsession (i.e., each audio or video 'substream'). In practice, this might
// be a class (or a chain of classes) that decodes and then renders the incoming
// audio or video. Or it might be a "FileSink", for outputting the received data
// into a file (as is done by the "openRTSP" application). In this example code,
// however, we define a simple 'dummy' sink that receives incoming data, but
// does nothing with it.

class H265Sink : public MediaSink {
 public:
  static H265Sink *createNew(
      UsageEnvironment &env,
      MediaSubsession
          &subsession,  // identifies the kind of data that's being received
      char const *streamId = NULL, int buffer_size = 200000,
      int buffer_count = 4,
      const std::string sdpline = "",
      const std::string vpsline = "",
      const std::string spsline = "",
      const std::string ppsline = "");

  virtual ~H265Sink();
  void SetFileName(std::tuple<bool, std::string> file);
  int SaveToFile(void *data, const int data_siz);
  void SetChannel(int channel);
  int GetChannel(void) const;
  void AddBufManager(std::shared_ptr<rtsp_client::BufferManager> buff_manager);
  void Stop();

 private:
  H265Sink(UsageEnvironment &env, MediaSubsession &subsession,
           char const *streamId, int buffer_size, int buffer_count,
           const std::string sdpline = "",
      const std::string vpsline = "",
      const std::string spsline = "",
      const std::string ppsline = "");
  // called only by "createNew()"
  // virtual ~H265Sink();

  static void afterGettingFrame(void *clientData, unsigned frameSize,
                                unsigned numTruncatedBytes,
                                struct timeval presentationTime,
                                unsigned durationInMicroseconds);
  void afterGettingFrame(unsigned frameSize, unsigned numTruncatedBytes,
                         struct timeval presentationTime,
                         unsigned durationInMicroseconds);
  void parseSDPLine();

 private:
  // redefined virtual functions:
  virtual Boolean continuePlaying();
  Boolean isNeedToWait(unsigned frameSize);

 private:
  MediaSubsession &subsession_;
  int buffer_size_;
  int buffer_count_;
  u_int8_t *buffers_vir_;
  uint64_t buffers_pyh_;

  char *stream_id_;
  std::string file_name_;
  bool save_file_;
  std::ofstream outfile_;
  int channel_;
  bool first_frame_;
  bool waiting_;
  uint64_t frame_count_;
  std::shared_ptr<rtsp_client::BufferManager> buff_manager_ = nullptr;
  std::shared_ptr<rtsp_client::VideoBuffer_ST> frame_ptr_ = nullptr;

  char *video_buffer_;
  int buffer_len_;
  int data_len_;

  char *vps_data_;
  int vps_len_ = 0;
  char *sps_data_;
  int sps_len_ = 0;
  char *pps_data_;
  int pps_len_ = 0;
  char* sei_data_;
  int sei_len_ = 0;

  std::string sdp_line_;
  std::string vps_line_;
  std::string sps_line_;
  std::string pps_line_;
  bool recv_sps_ = false;
  bool recv_pps_ = false;
  bool recv_vps_ = false;
};
#endif
