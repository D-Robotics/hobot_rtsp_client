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

#include "rtspclient/H264Sink.h"

#include "hobotlog/hobotlog.hpp"
#include "rtspclient/Base64.h"
#include "rtspclient/SPSInfoMgr.h"

#define DUMMY_SINK_RECEIVE_BUFFER_SIZE 200000
void H264Sink::SetFileName(std::tuple<bool, std::string> file) {
  save_file_ = std::get<0>(file);
  file_name_ = std::get<1>(file);
}

int H264Sink::SaveToFile(void *data, const int data_size) {
  if (!outfile_.is_open()) {
    file_name_ = "channel" + std::to_string(channel_) + "_input.h264";
    outfile_.open(file_name_, std::ios::app | std::ios::out | std::ios::binary);
  }
  if (outfile_.is_open()) {
    outfile_.write(reinterpret_cast<char *>(data), data_size);
  }
  return 0;
}

void H264Sink::SetChannel(int channel) { channel_ = channel; }

int H264Sink::GetChannel(void) const { return channel_; }

H264Sink *H264Sink::createNew(UsageEnvironment &env,
                              MediaSubsession &subsession, char const *streamId,
                              int buffer_size, int buffer_count,
                              const std::string sdpline) {
  return new H264Sink(env, subsession, streamId, buffer_size, buffer_count,
                      sdpline);
}

H264Sink::H264Sink(UsageEnvironment &env, MediaSubsession &subsession,
                   char const *streamId, int buffer_size, int buffer_count,
                   const std::string sdpline)
    : MediaSink(env),
      subsession_(subsession),
      buffer_size_(buffer_size),
      buffer_count_(buffer_count),
      save_file_(false),
      channel_(-1),
      first_frame_(true),
      waiting_(true),
      frame_count_(0),
      sdp_line_(sdpline) {
  int ret = 0;
  stream_id_ = strDup(streamId);
  video_buffer_ = new char[buffer_size_];
  buffer_len_ = buffer_size_;
  data_len_ = 0;
  if (video_buffer_ == NULL) {
  } else {
    memset(video_buffer_, 0, buffer_len_);
  }

  sps_data_ = new char[64];
  sps_len_ = 0;
  if (sps_data_ == NULL) {
  } else {
    memset(sps_data_, 0, 64);
  }

  pps_data_ = new char[64];
  pps_len_ = 0;
  if (pps_data_ == NULL) {
  } else {
    memset(pps_data_, 0, 64);
  }

  parseSDPLine();
}

H264Sink::~H264Sink() {
  LOGI << "H264Sink::~H264Sink(), channel:" << channel_;
  delete[] stream_id_;
  delete[] video_buffer_;
  delete[] sps_data_;
  delete[] pps_data_;

  if (outfile_.is_open()) {
    outfile_.close();
  }
  LOGI << "leave ~H264Sink(), channel:" << channel_;
}

void H264Sink::afterGettingFrame(void *clientData, unsigned frameSize,
                                 unsigned numTruncatedBytes,
                                 struct timeval presentationTime,
                                 unsigned durationInMicroseconds) {
  H264Sink *sink = reinterpret_cast<H264Sink *>(clientData);
  if (!sink) {
    LOGE << "H264Sink::afterGettingFrame get H264Sink is null, error!!!";
    return;
  }
  sink->afterGettingFrame(frameSize, numTruncatedBytes, presentationTime,
                          durationInMicroseconds);
}

// If you don't want to see debugging output for each received frame, then
// comment out the following line:
#define DEBUG_PRINT_EACH_RECEIVED_FRAME 0

void H264Sink::afterGettingFrame(unsigned frameSize, unsigned numTruncatedBytes,
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
    u_int8_t *buffer = (u_int8_t*)video_buffer_;
    int nNaltype = 0;
    nNaltype |= (buffer[4] & 0x1f);
    if (nNaltype == 0x05) {
      if (frame_ptr_ == nullptr) {
        frame_ptr_ = buff_manager_->GetBuffDef();
      }    
      data_len_ = 0;
      memcpy(frame_ptr_->buff, sps_data_, sps_len_);
      data_len_ += sps_len_;
      memcpy(frame_ptr_->buff + data_len_, pps_data_, pps_len_);
      data_len_ += pps_len_;
      memcpy(frame_ptr_->buff + data_len_, video_buffer_, frameSize + 4);
      data_len_ += frameSize + 4;

      frame_ptr_->data_size = data_len_;
      frame_ptr_->frame_id = frame_count_;
      frame_ptr_->format = "h264";
      frame_ptr_->pts.tv_sec = presentationTime.tv_sec;
      frame_ptr_->pts.tv_nsec = presentationTime.tv_usec*1000;
      frame_ptr_->dts.tv_sec = presentationTime.tv_sec;
      frame_ptr_->dts.tv_nsec = presentationTime.tv_usec*1000;
      int ret = buff_manager_->Input(frame_ptr_);
      if (save_file_) {
        SaveToFile(frame_ptr_->buff, data_len_);
      }
      frame_ptr_ = nullptr;
      if (ret != 0) {
        LOGE << "HB_VDEC_SendStream failed.";
      }
      first_frame_ = false;
    }
    data_len_ = 0;
    frame_count_++;
    continuePlaying();
    return;
  }
  u_int8_t *buffer = (u_int8_t*)video_buffer_;
  int nNaltype = 0;
  nNaltype |= (buffer[4] & 0x1f);
  if ((nNaltype == 1) || (nNaltype == 2) || (nNaltype == 3) ||
      (nNaltype == 4) || (nNaltype == 5) || (nNaltype == 7)) {
    if (nNaltype == 5) {
      if (frame_ptr_ == nullptr) {
        frame_ptr_ = buff_manager_->GetBuffDef();
      }    
      data_len_ = 0;
      memcpy(frame_ptr_->buff, sps_data_, sps_len_);
      data_len_ += sps_len_;
      memcpy(frame_ptr_->buff + data_len_, pps_data_, pps_len_);
      data_len_ += pps_len_;
      memcpy(frame_ptr_->buff + data_len_, video_buffer_, frameSize + 4);
      data_len_ += frameSize + 4;

      frame_ptr_->data_size = data_len_;
      frame_ptr_->frame_id = frame_count_;
      frame_ptr_->format = "h264";
      frame_ptr_->pts.tv_sec = presentationTime.tv_sec;
      frame_ptr_->pts.tv_nsec = presentationTime.tv_usec*1000;
      frame_ptr_->dts.tv_sec = presentationTime.tv_sec;
      frame_ptr_->dts.tv_nsec = presentationTime.tv_usec*1000;
      int ret = buff_manager_->Input(frame_ptr_);
      if (save_file_) {
        SaveToFile(frame_ptr_->buff, data_len_);
      }
      frame_ptr_ = nullptr;
      if (ret != 0) {
        LOGE << "HB_VDEC_SendStream failed.";
      }
    } else if ((nNaltype == 7)&& (frameSize < 64)) {

    } else {
      if (frame_ptr_ == nullptr) {
        frame_ptr_ = buff_manager_->GetBuffDef();
      }    
      memcpy(frame_ptr_->buff, video_buffer_, frameSize + 4);
      frame_ptr_->data_size = frameSize + 4;
      frame_ptr_->frame_id = frame_count_;
      frame_ptr_->format = "h264";
      frame_ptr_->pts.tv_sec = presentationTime.tv_sec;
      frame_ptr_->pts.tv_nsec = presentationTime.tv_usec*1000;
      frame_ptr_->dts.tv_sec = presentationTime.tv_sec;
      frame_ptr_->dts.tv_nsec = presentationTime.tv_usec*1000;
      int ret = buff_manager_->Input(frame_ptr_);
      if (save_file_) {
        SaveToFile(frame_ptr_->buff, data_len_);
      }
      frame_ptr_ = nullptr;
      if (ret != 0) {
        LOGE << "HB_VDEC_SendStream failed.";
      }
    }
  }

  frame_count_++;
  // Then continue, to request the next frame of data:
  continuePlaying();
}

void H264Sink::AddBufManager(std::shared_ptr<rtsp_client::BufferManager> buff_manager) {
  buff_manager_ = buff_manager;
}

Boolean H264Sink::isNeedToWait(unsigned frameSize) {
  if (!first_frame_) return false;
  u_int8_t *buffer = (u_int8_t*)video_buffer_;
  int nNaltype = 0;
  nNaltype |= (buffer[4] & 0x1f);
  LOGW << "channel:" << channel_ << " recv stream nal type:" << nNaltype;
  if (nNaltype == 0x07) {
    sps_len_ = frameSize + 4;
    memcpy(sps_data_, buffer, sps_len_);
    recv_sps_ = true;
  } else if (nNaltype == 0x08) {
    pps_len_ = frameSize + 4;
    memcpy(pps_data_, buffer, pps_len_);
    recv_pps_ = true;
  } else if (nNaltype == 0x05) {
    if ((recv_sps_ && recv_pps_) ||          // get sps pps from rtp packet
        (sps_len_ != 0 && pps_len_ != 0)) {  // get sps pps from sdp
#if ngy
      data_len_ = 0;
      memcpy(video_buffer_, sps_data_, sps_len_);
      data_len_ += sps_len_;
      memcpy(video_buffer_ + data_len_, pps_data_, pps_len_);
      data_len_ += pps_len_;
      memcpy(video_buffer_ + data_len_, buffer, frameSize + 4);
      data_len_ += frameSize + 4;
#endif
      LOGW << "channel:" << channel_
           << " to analytics sps info, and start to decode";
      int width = 0;
      int height = 0;
      vision::SPSInfoMgr::GetInstance().AnalyticsSps(
          (unsigned char *)sps_data_ + 4, sps_len_ - 4, width, height);
      buff_manager_->SetResolution(width, height);
      return false;
    } else {
      LOGE << "channel:" << channel_
           << " can not get sps pps info from rtp and sdp info";
    }
  }
  return true;
}

Boolean H264Sink::continuePlaying() {
  if (fSource == NULL) return False;  // sanity check (should not happen)
  static unsigned char start_code[4] = {0x00, 0x00, 0x00, 0x01};
  u_int8_t *buffer = (u_int8_t *)video_buffer_;
  // Request the next frame of data from our input source. "afterGettingFrame()"
  // will get called later, when it arrives:
  memcpy(reinterpret_cast<void *>(buffer), start_code, 4);
  buffer += 4;
  fSource->getNextFrame(buffer, buffer_size_-4, afterGettingFrame, this,
                        onSourceClosure, this);
  return True;
}

bool H264Sink::separateTo2(const std::string handle_string,
                           const std::string &p_strSeparator,
                           std::string &p_strFirst, std::string &p_strSecond) {
  if (p_strSeparator.empty() || handle_string.empty()) return false;

  size_t nEndPos = handle_string.find(p_strSeparator);
  if (nEndPos == std::string::npos) return false;

  p_strFirst = handle_string.substr(0, nEndPos);
  p_strSecond = handle_string.substr(nEndPos + p_strSeparator.length());

  return true;
}

void H264Sink::parseSDPLine() {
  // string strSDP =
  // "sprop-parameter-sets=Z0IAKeKQCgDLYC3AQEBpB4kRUA==,aM48gA=="; string strSps
  // = "Z0IAKeKQCgDLYC3AQEBpB4kRUA=="; string strPps = "aM48gA==";
  if (sdp_line_.empty() || sps_data_ == NULL || pps_data_ == NULL) return;
  std::string sps;
  std::string pps;
  bool bRet = separateTo2(sdp_line_, ",", sps, pps);
  if (!bRet) return;
  LOGW << "H264Sink recv sdp_line_ line:" << sdp_line_;
  LOGW << "H264Sink parse get sps line:" << sps;
  LOGW << "H264Sink parse get pps line:" << pps;

  static unsigned char start_code[4] = {0x00, 0x00, 0x00, 0x01};
  memcpy(sps_data_, start_code, 4);
  memcpy(pps_data_, start_code, 4);

  sps_len_ = 0;
  pps_len_ = 0;
  int out_sps_len = 60;
  int nRet = base64::base64Decode(sps.c_str(), sps.length(), sps_data_ + 4,
                                  out_sps_len);
  if (0 != nRet) {
    LOGE << "parse sps from sdp fail, channel:" << channel_;
    memset(sps_data_, 0, sps_len_);
    sps_len_ = 0;
    return;
  }

  int out_pps_len = 60;
  nRet = base64::base64Decode(pps.c_str(), pps.length(), pps_data_ + 4,
                              out_pps_len);
  if (0 != nRet) {
    LOGE << "parse pps from sdp fail, channel:" << channel_;
    memset(pps_data_, 0, pps_len_);
    pps_len_ = 0;
    return;
  }

  sps_len_ += 4;
  sps_len_ += out_sps_len;
  pps_len_ += 4;
  pps_len_ += out_pps_len;
  LOGW << "parse sps pps from sdp success, channel:" << channel_
       << ", sps len:" << sps_len_ << ", pps len:" << pps_len_;
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
#endif
}
