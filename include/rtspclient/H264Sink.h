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
#ifndef INCLUDE_RTSPCLIENT_H264SINK_H_
#define INCLUDE_RTSPCLIENT_H264SINK_H_

#include <fstream>
#include <iostream>
#include <memory>
#include <vector>
#include <string>
#include <tuple>

#include "BasicUsageEnvironment.hh"
#include "H264VideoRTPSource.hh"
#include "liveMedia.hh"
#include "buffmanager.h"

// Define a data sink (a subclass of "MediaSink") to receive the data for each
// subsession (i.e., each audio or video 'substream'). In practice, this might
// be a class (or a chain of classes) that decodes and then renders the incoming
// audio or video. Or it might be a "FileSink", for outputting the received data
// into a file (as is done by the "openRTSP" application). In this example code,
// however, we define a simple 'dummy' sink that receives incoming data, but
// does nothing with it.

class H264Sink : public MediaSink {
 public:
  static H264Sink *createNew(
      UsageEnvironment &env,
      MediaSubsession
          &subsession,  // identifies the kind of data that's being received
      char const *streamId = NULL, int buffer_size = 200000,
      int buffer_count = 4,
      const std::string sdpline = "");

  virtual ~H264Sink();
  void SetFileName(std::tuple<bool, std::string> file);
  int SaveToFile(void *data, const int data_siz);
  void SetChannel(int channel);
  int GetChannel(void) const;
  void AddBufManager(std::shared_ptr<rtsp_client::BufferManager> buff_manager);
  void Stop();

 private:
  H264Sink(UsageEnvironment &env, MediaSubsession &subsession,
           char const *streamId, int buffer_size,
           int buffer_count, const std::string sdpline);
  // called only by "createNew()"
  // virtual ~H264Sink();

  static void afterGettingFrame(void *clientData, unsigned frameSize,
                                unsigned numTruncatedBytes,
                                struct timeval presentationTime,
                                unsigned durationInMicroseconds);
  void afterGettingFrame(unsigned frameSize, unsigned numTruncatedBytes,
                         struct timeval presentationTime,
                         unsigned durationInMicroseconds);

  bool separateTo2(const std::string handle_string,
                   const std::string& p_strSeparator,
                   std::string& p_strFirst, std::string& p_strSecond);
  void parseSDPLine();

 private:
  // redefined virtual functions:
  virtual Boolean continuePlaying();
  Boolean isNeedToWait(unsigned frameSize);

 private:
  MediaSubsession &subsession_;
  int buffer_size_;
  int buffer_count_;

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
  bool init_decoder_ = false;

  char *video_buffer_;
  int buffer_len_;
  int data_len_;

  char* sps_data_;
  int sps_len_;
  char* pps_data_;
  int pps_len_;

  std::string sdp_line_;
  bool recv_sps_ = false;
  bool recv_pps_ = false;

};

#endif
