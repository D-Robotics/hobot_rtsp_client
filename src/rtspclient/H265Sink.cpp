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

#include "rtspclient/H265Sink.h"

#include "hobotlog/hobotlog.hpp"
#include "rtspclient/Base64.h"
#include "rtspclient/SPSInfoMgr.h"

// Implementation of "H265Sink":
#define DUMMY_SINK_RECEIVE_BUFFER_SIZE 200000
void H265Sink::SetFileName(std::tuple<bool, std::string> file) {
  save_file_ = std::get<0>(file);
  file_name_ = std::get<1>(file);
}

int H265Sink::SaveToFile(void *data, const int data_size) {
  if (!outfile_.is_open()) {
    file_name_ = "channel" + std::to_string(channel_) + "_input.h265";
    outfile_.open(file_name_, std::ios::app | std::ios::out | std::ios::binary);
  }
  if (outfile_.is_open()) {
    outfile_.write(reinterpret_cast<char *>(data), data_size);
  }

  return 0;
}

void H265Sink::SetChannel(int channel) { channel_ = channel; }

int H265Sink::GetChannel(void) const { return channel_; }

H265Sink *H265Sink::createNew(UsageEnvironment &env,
                              MediaSubsession &subsession, char const *streamId,
                              int buffer_size, int buffer_count,
                              const std::string sdpline,
                              const std::string vpsline,
                              const std::string spsline,
                              const std::string ppsline) {
  return new H265Sink(env, subsession, streamId, buffer_size, buffer_count,
                      sdpline, vpsline, spsline, ppsline);
}

H265Sink::H265Sink(UsageEnvironment &env, MediaSubsession &subsession,
                   char const *streamId, int buffer_size, int buffer_count,
                   const std::string sdpline, const std::string vpsline,
                   const std::string spsline, const std::string ppsline)
    : MediaSink(env),
      subsession_(subsession),
      buffer_size_(buffer_size),
      buffer_count_(buffer_count),
      save_file_(false),
      channel_(-1),
      first_frame_(true),
      waiting_(true),
      frame_count_(0),
      sdp_line_(sdpline),
      vps_line_(vpsline),
      sps_line_(spsline),
      pps_line_(ppsline) {
  int ret = 0;
  stream_id_ = strDup(streamId);

  video_buffer_ = new char[buffer_size_];
  buffer_len_ = buffer_size_;
  data_len_ = 0;
  if (video_buffer_ == NULL) {
  } else {
    memset(video_buffer_, 0, buffer_len_);
  }

  vps_data_ = new char[128];
  vps_len_ = 0;
  if (vps_data_ == NULL) {
  } else {
    memset(vps_data_, 0, 128);
  }

  sps_data_ = new char[128];
  sps_len_ = 0;
  if (sps_data_ == NULL) {
  } else {
    memset(sps_data_, 0, 128);
  }

  pps_data_ = new char[128];
  pps_len_ = 0;
  if (pps_data_ == NULL) {
  } else {
    memset(pps_data_, 0, 128);
  }

  sei_data_ = new char[128];
  sei_len_ = 0;
  if (sei_data_ == NULL) {
  } else {
    memset(sei_data_, 0, 128);
  }

  parseSDPLine();
}

H265Sink::~H265Sink() {
  LOGI << "H265Sink::~H265Sink(), channel:" << channel_;
  delete[] stream_id_;
  delete[] video_buffer_;
  delete[] vps_data_;
  delete[] sps_data_;
  delete[] pps_data_;
  delete[] sei_data_;

  if (outfile_.is_open()) {
    outfile_.close();
  }
  LOGI << "leave ~H265Sink(), channel:" << channel_;
}

void H265Sink::afterGettingFrame(void *clientData, unsigned frameSize,
                                 unsigned numTruncatedBytes,
                                 struct timeval presentationTime,
                                 unsigned durationInMicroseconds) {
  H265Sink *sink = reinterpret_cast<H265Sink *>(clientData);
  sink->afterGettingFrame(frameSize, numTruncatedBytes, presentationTime,
                          durationInMicroseconds);
}

// If you don't want to see debugging output for each received frame, then
// comment out the following line:
#define DEBUG_PRINT_EACH_RECEIVED_FRAME 0

void H265Sink::afterGettingFrame(unsigned frameSize, unsigned numTruncatedBytes,
                                 struct timeval presentationTime,
                                 unsigned /*durationInMicroseconds*/) {
  // We've just received a frame of data.  (Optionally) print out information
  // about it:
#if DEBUG_PRINT_EACH_RECEIVED_FRAME
  if (stream_id_ != NULL) envir() << "Stream \"" << stream_id_ << "\"; ";
  envir() << subsession_.mediumName() << "/" << subsession_.codecName()
          << ":\tReceived " << frameSize << " bytes";
  if (numTruncatedBytes > 0)
    envir() << " (with " << numTruncatedBytes << " bytes truncated)";
  char uSecsStr[6 + 1];  // used to output the 'microseconds' part of the
                         // presentation time
  snprintf(uSecsStr, sizeof(uSecsStr), "%06u",
           (unsigned)presentationTime.tv_usec);
  envir() << ".\tPresentation time: "
          << reinterpret_cast<int>(presentationTime.tv_sec) << "." << uSecsStr;
  if (subsession_.rtpSource() != NULL &&
      !subsession_.rtpSource()->hasBeenSynchronizedUsingRTCP()) {
    envir() << "!";  // mark the debugging output to indicate that this
                     // presentation time is not RTCP-synchronized
  }
#ifdef DEBUG_PRINT_NPT
  envir() << "\tNPT: " << subsession_.getNormalPlayTime(presentationTime);
#endif
  envir() << "\n";
#endif

  if (!buff_manager_) return;
  // unsigned char start_code[4] = {0x00, 0x00, 0x00, 0x01};
  waiting_ = isNeedToWait(frameSize);
  if (waiting_) {
    if (!first_frame_) {
      frame_count_++;
    }
    // Then continue, to request the next frame of data:
    continuePlaying();
    return;
  }

  if (first_frame_) {
    memcpy(frame_ptr_->buff, video_buffer_, data_len_);
    frame_ptr_->data_size = data_len_;
    frame_ptr_->frame_id = frame_count_;
    frame_ptr_->format = "h265";
    frame_ptr_->pts.tv_sec = presentationTime.tv_sec;
    frame_ptr_->pts.tv_nsec = presentationTime.tv_usec*1000;
    frame_ptr_->dts.tv_sec = presentationTime.tv_sec;
    frame_ptr_->dts.tv_nsec = presentationTime.tv_usec*1000;
    buff_manager_->Input(frame_ptr_);

    if (save_file_) {
      SaveToFile(video_buffer_, data_len_);
    }
    data_len_ = 0;
    frame_count_++;
    first_frame_ = false;
    continuePlaying();
    return;
  }

  if (frame_ptr_) {
    frame_ptr_->data_size = frameSize + 4;
    frame_ptr_->frame_id = frame_count_;
    frame_ptr_->format = "h265";
    frame_ptr_->pts.tv_sec = presentationTime.tv_sec;
    frame_ptr_->pts.tv_nsec = presentationTime.tv_usec*1000;
    frame_ptr_->dts.tv_sec = presentationTime.tv_sec;
    frame_ptr_->dts.tv_nsec = presentationTime.tv_usec*1000;
    buff_manager_->Input(frame_ptr_);
    if (save_file_) {
      SaveToFile(frame_ptr_->buff, frameSize + 4);
    }
    frame_ptr_ = nullptr;

  } 

  frame_count_++;
  // Then continue, to request the next frame of data:
  continuePlaying();
}

void H265Sink::AddBufManager(std::shared_ptr<rtsp_client::BufferManager> buff_manager) {
  buff_manager_ = buff_manager;
}

#if 1
Boolean H265Sink::isNeedToWait(unsigned frameSize) {
  if (frame_ptr_ == nullptr) {
    frame_ptr_ = buff_manager_->GetBuffDef();
  }
  u_int8_t *buffer = (u_int8_t *)frame_ptr_->buff;
  int nNalType = 0;
  nNalType = (buffer[4] & 0x7E) >> 1;
  if (!first_frame_) {
    if (nNalType == 39) return true;
    return false;
  }
  LOGW << "channel:" << channel_ << " recv stream nal type:" << nNalType;
  if (nNalType == 32) {
    vps_len_ = frameSize + 4;
    memcpy(vps_data_, buffer, vps_len_);
    recv_vps_ = true;
  } else if (nNalType == 33) {  // vps
    sps_len_ = frameSize + 4;
    memcpy(sps_data_, buffer, sps_len_);
    recv_sps_ = true;
  } else if (nNalType == 34) {
    pps_len_ = frameSize + 4;
    memcpy(pps_data_, buffer, pps_len_);
    recv_pps_ = true;
  } else if (nNalType == 39) {
    sei_len_ = frameSize + 4;
    memcpy(sei_data_, buffer, sei_len_);
  } else if (nNalType == 19) {
    if ((recv_sps_ && recv_pps_ && recv_vps_) ||  // get sps pps from rtp packet
        (sps_len_ != 0 && pps_len_ != 0 &&
         vps_len_ != 0)) {  // get sps pps from sdp
      data_len_ = 0;
      memcpy(video_buffer_, vps_data_, vps_len_);
      data_len_ += vps_len_;
      memcpy(video_buffer_ + data_len_, sps_data_, sps_len_);
      data_len_ += sps_len_;
      memcpy(video_buffer_ + data_len_, pps_data_, pps_len_);
      data_len_ += pps_len_;
      if (sei_len_ != 0) {
        memcpy(video_buffer_ + data_len_, sei_data_, sei_len_);
        data_len_ += sei_len_;
      }

      memcpy(video_buffer_ + data_len_, buffer, frameSize + 4);
      data_len_ += frameSize + 4;

      int width = 0;
      int height = 0;
      vision::SPSInfoMgr::GetInstance().AnalyticsSps(
          (unsigned char *)sps_data_ + 4, sps_len_ - 4, width, height, "H265");
      buff_manager_->SetResolution(width, height);
      LOGW << "channel:" << channel_
           << " to analytics sps info, get width:" << width
           << ", height:" << height << ", start to decode";
      return false;
    } else {
      LOGE << "channel:" << channel_
           << " can not get sps pps info from rtp and sdp info";
    }
  }
  return true;
}

Boolean H265Sink::continuePlaying() {
  if (fSource == NULL) return False;  // sanity check (should not happen)
  static unsigned char start_code[4] = {0x00, 0x00, 0x00, 0x01};
  if (frame_ptr_ == nullptr) {
    frame_ptr_ = buff_manager_->GetBuffDef();
  }
  u_int8_t *buffer = (u_int8_t *)frame_ptr_->buff;
  // Request the next frame of data from our input source. "afterGettingFrame()"
  // will get called later, when it arrives:
  memcpy(reinterpret_cast<void *>(buffer), start_code, 4);
  buffer += 4;
  fSource->getNextFrame(buffer, buffer_size_, afterGettingFrame, this,
                        onSourceClosure, this);
  return True;
}

void H265Sink::parseSDPLine() {
  if (vps_data_ == NULL || sps_data_ == NULL || pps_data_ == NULL) return;
  LOGW << "H265Sink recv vps line:" << vps_line_;
  LOGW << "H265Sink recv sps line:" << sps_line_;
  LOGW << "H265Sink recv pps line:" << pps_line_;
  static unsigned char start_code[4] = {0x00, 0x00, 0x00, 0x01};
  memcpy(vps_data_, start_code, 4);
  memcpy(sps_data_, start_code, 4);
  memcpy(pps_data_, start_code, 4);

  vps_len_ = 0;
  sps_len_ = 0;
  pps_len_ = 0;
  int out_vps_len = 124;
  int nRet = base64::base64Decode(vps_line_.c_str(), vps_line_.length(),
                                  vps_data_ + 4, out_vps_len);
  if (0 != nRet) {
    LOGE << "H265Sink parse vps line fail";
    return;
  }

  int out_sps_len = 124;
  nRet = base64::base64Decode(sps_line_.c_str(), sps_line_.length(),
                                  sps_data_ + 4, out_sps_len);
  if (0 != nRet) {
    LOGE << "H265Sink parse sps line fail";
    return;
  }

  int out_pps_len = 124;
  nRet = base64::base64Decode(pps_line_.c_str(), pps_line_.length(),
                              pps_data_ + 4, out_pps_len);
  if (0 != nRet) {
    LOGE << "H265Sink parse pps line fail";
    return;
  }

  vps_len_ += 4 + out_vps_len;
  sps_len_ += 4 + out_sps_len;
  pps_len_ += 4 + out_pps_len;
  LOGE << "H265Sink parse vps sps pps line success";
#if 0
  printf(
      "\n ******** media data  sps[0] :%02x, \n sps[1] :%02x,  sps[2]:%02x, "
      "sps[3]:%02x, sps[4]:%02x, sps[5]:%02x\n",
      sps_data_[0], sps_data_[1], sps_data_[2], sps_data_[3], sps_data_[4],
      sps_data_[5]);
  printf(
      "\n ******** media data  pps[0] :%02x, \n pps[1] :%02x,  pps[2] :%02x, "
      "pps[3] :%02x,pps[4] :%02x, "
      "\n\n",
      pps_data_[0], pps_data_[1], pps_data_[2], pps_data_[3], pps_data_[4]);

  printf("----------parse sdp get sps len:%d, pps len:%d\n", sps_len_,
         pps_len_);
#endif
}

#else
Boolean H265Sink::isNeedToWait(unsigned frameSize) {
  if (first_frame_) {
    LOGW << "channel:" << channel_ << " recv stream";
    int nNal1 = 0;
    nNal1 = (video_buffer_[4] & 0x7E) >> 1;
    if (nNal1 != 32) {  // not vps
      data_len_ = 0;
      return true;
    }
    if (data_len_ == 4) {  // first nal
      LOGW << "channle:" << channel_
           << ", first frame recv h265 nal1 type:" << nNal1
           << " frame size:" << frameSize;
      data_len_ += frameSize;
      vps_len_ = data_len_;
      LOGW << "channle:" << channel_ << "vps len:" << vps_len_;
      return true;
    }

    int nNal2 = 0;
    char *buffer = video_buffer_ + vps_len_;
    nNal2 = (buffer[4] & 0x7E) >> 1;
    if (nNal2 != 33) {  // sps
      data_len_ = 0;
      vps_len_ = 0;
      sps_len_ = 0;
      return true;
    }

    if (sps_len_ == 0 && vps_len_ > 0) {
      LOGW << "channle:" << channel_
           << ", first frame recv h265 nal2 type:" << nNal2
           << " framesize:" << frameSize;
      int width = 0;
      int height = 0;
      vision::SPSInfoMgr::GetInstance().AnalyticsSps(
          (unsigned char *)video_buffer_ + data_len_, frameSize, width, height,
          "H265");
      pipe_line_->SetDecodeResolution(width, height);
      pipe_line_->Init();
      pipe_line_->Start();
      data_len_ += frameSize;
      sps_len_ = data_len_;
      LOGW << "channle:" << channel_ << "sps len:" << sps_len_;
      return true;
    }

    buffer = video_buffer_ + sps_len_;
    int nal_type = (buffer[4] & 0x7E) >> 1;
    data_len_ += frameSize;
    LOGW << "channle:" << channel_
         << ", first frame recv h265 nal3 type:" << nal_type;
    if (nal_type == 34) {  // pps
      return false;
    } else {
      data_len_ = 0;
      vps_len_ = 0;
      sps_len_ = 0;
      return true;
    }
  }
  u_int8_t *buffer =
      buffers_vir_ + (frame_count_ % buffer_count_) * buffer_size_;
  int nNalUnitType = 0;
  nNalUnitType = (buffer[4] & 0x7E) >> 1;
  if (nNalUnitType == 32) {
    first_frame_ = true;
    data_len_ = 4;
    data_len_ += frameSize;
    vps_len_ = data_len_;
    memcpy(video_buffer_, buffer, data_len_);
    return true;
  }
  return false;
}

Boolean H265Sink::continuePlaying() {
  if (fSource == NULL) return False;  // sanity check (should not happen)
  static unsigned char start_code[4] = {0x00, 0x00, 0x00, 0x01};
  if (first_frame_) {
    memcpy(video_buffer_ + data_len_, start_code, 4);
    data_len_ += 4;
    fSource->getNextFrame((unsigned char *)video_buffer_ + data_len_,
                          buffer_len_ - data_len_, afterGettingFrame, this,
                          onSourceClosure, this);
    return True;
  }

  u_int8_t *buffer =
      buffers_vir_ + (frame_count_ % buffer_count_) * buffer_size_;
  // Request the next frame of data from our input source. "afterGettingFrame()"
  // will get called later, when it arrives:
  memcpy(reinterpret_cast<void *>(buffer), start_code, 4);
  buffer += 4;

  fSource->getNextFrame(buffer, buffer_size_, afterGettingFrame, this,
                        onSourceClosure, this);
  return True;
}
#endif
