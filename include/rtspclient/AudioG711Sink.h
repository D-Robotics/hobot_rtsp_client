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
#ifndef INCLUDE_RTSPCLIENT_AUDIOG711SINK_H_
#define INCLUDE_RTSPCLIENT_AUDIOG711SINK_H_

#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <tuple>

#include "BasicUsageEnvironment.hh"
#include "liveMedia.hh"

// Define a data sink (a subclass of "MediaSink") to receive the data for each
// subsession (i.e., each audio or video 'substream'). In practice, this might
// be a class (or a chain of classes) that decodes and then renders the incoming
// audio or video. Or it might be a "FileSink", for outputting the received data
// into a file (as is done by the "openRTSP" application). In this example code,
// however, we define a simple 'dummy' sink that receives incoming data, but
// does nothing with it.

class AudioG711Sink : public MediaSink {
 public:
  static AudioG711Sink *createNew(
      UsageEnvironment &env,
      MediaSubsession
          &subsession,  // identifies the kind of data that's being received
      char const *streamId = NULL, int buffer_size = 2048,
      int buffer_count = 2);  // identifies the stream itself (optional)

  virtual ~AudioG711Sink();
  void SetFileName(std::tuple<bool, std::string> file);
  int SaveToFile(void *data, const int data_siz);
  void SetChannel(int channel);
  int GetChannel(void) const;
  void Stop();

 private:
  AudioG711Sink(UsageEnvironment &env, MediaSubsession &subsession,
                char const *streamId, int buffer_size, int buffer_count);
  // called only by "createNew()"
  // virtual ~AudioSink();

  static void afterGettingFrame(void *clientData, unsigned frameSize,
                                unsigned numTruncatedBytes,
                                struct timeval presentationTime,
                                unsigned durationInMicroseconds);
  void afterGettingFrame(unsigned frameSize, unsigned numTruncatedBytes,
                         struct timeval presentationTime,
                         unsigned durationInMicroseconds);

 private:
  // redefined virtual functions:
  virtual Boolean continuePlaying();

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

  char *buffer_;
  // int data_len_;
};
#endif
