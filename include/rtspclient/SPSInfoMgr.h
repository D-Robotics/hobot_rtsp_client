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
#ifndef INCLUDE_RTSPCLIENT_SPSINFOMGR_H_
#define INCLUDE_RTSPCLIENT_SPSINFOMGR_H_

#include <string.h>

#include <mutex>
#include <string>
namespace vision {
class SPSInfoMgr {
 public:
  static SPSInfoMgr &GetInstance();
  ~SPSInfoMgr() = default;

 public:
  int Init();

  int AnalyticsSps(const unsigned char *spsinfo, const int sps_len, int &width,
                   int &height, const std::string type_name = "H264");

 private:
  int AnalyticsSps_H264(const unsigned char *spsinfo, const int sps_len,
                        int &width, int &height);

  int AnalyticsSps_H265(const unsigned char *spsinfo, const int sps_len,
                        int &width, int &height);

 private:
  SPSInfoMgr();
  SPSInfoMgr(const SPSInfoMgr &);
  SPSInfoMgr &operator=(const SPSInfoMgr &);

  static SPSInfoMgr *instance_;
  std::mutex manager_mutex_;
  bool initialized_;
};
}  // namespace vision

#endif  // INCLUDE_RTSPCLIENT_SPSINFOMGR_H_
