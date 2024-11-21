English| [简体中文](./README_cn.md)

# Function Introduction

The **hobot_rtsp_client** accepts H264 and H265 streams from IPC through the RTSP protocol. Support ROS standard format subscription and publish img_msgs/msg/H26XFrame topics.
# Instructions for Use

## Install the Package

Run the following commands in the terminal of the RDK system to quickly install:

tros humble:
```bash
sudo apt update
sudo apt install -y tros-humble-hobot-rtsp-client
```

## Start Stream
Prepare the RTSP data source for IPC in advance;

Or start the RTSP streaming example of RDK system, refer to [video stream](https://developer.d-robotics.cc/rdk_doc/en/Basic_Application/multi_media/pydev_vio_demo)
```bash
cd /app/pydev_demo/08_decode_rtsp_stream/
root@ubuntu:/app/pydev_demo/08_decode_rtsp_stream# sudo ./live555MediaServer &
```

Run the following instructions in the terminal of the RDK system.

tros humble:
```bash
# Configure the tros.b humble environment:
source /opt/tros/humble/setup.bash
# Launch to start
ros2 run hobot_rtsp_client hobot_rtsp_client --ros-args -p rtsp_url_num:=1 -p rtsp_url_0:='rtsp://127.0.0.1/1080P_test.h264' -p transport_0:='udp'
```

when 4 channel video stream
```bash
# Configure the tros.b humble environment:
source /opt/tros/humble/setup.bash
# Launch to start
ros2 run hobot_rtsp_client hobot_rtsp_client --ros-args -p rtsp_url_num:=4 -p rtsp_url_0:='rtsp://127.0.0.1/1080P_test.h264' -p transport_0:='udp'  -p rtsp_url_1:='rtsp://127.0.0.1/1080P_test.h264' -p transport_1:='udp'  -p rtsp_url_2:='rtsp://127.0.0.1/1080P_test.h264' -p transport_2:='udp'  -p rtsp_url_3:='rtsp://127.0.0.1/1080P_test.h264' -p transport_3:='udp'
```


If the following information is output, it indicates that the node has been successfully launched:

```text
[hobot_codec_republish-2] [WARN] [1732169402.355433988] [hobot_codec_decoder]: Sub imgRaw fps = -1774563328
[hobot_codec_republish-2] [WARN] [1732169402.906547961] [hobot_codec_decoder]: sub h264 1920x1080, fps: 24.7706, pub nv12, fps: 9.17431, comm delay [-8.8148]ms, codec delay [171.2000]ms
[mono2d_body_detection-4] [WARN] [1732169402.906916796] [mono2d_body_det]: SharedMemImgProcess Recved img encoding: nv12, h: 1080, w: 1920, step: 1920, index: 2508, stamp: 1732169402_735947000, data size: 3133440, comm delay [170.9541]ms
[hobot_codec_republish-3] [WARN] [1732169403.274412126] [hobot_codec_encoder]: sub nv12 1920x1088, fps: 10.8055, pub jpeg, fps: 10.8055, comm delay [164.9091]ms, codec delay [7.6364]ms
[mono2d_body_detection-4] [WARN] [1732169403.321086039] [mono2d_body_det]: input fps: 10.81, out fps: 10.81, infer time ms: 92, post process time ms: 10
[hobot_codec_republish-2] [WARN] [1732169403.946849482] [hobot_codec_decoder]: sub h264 1920x1080, fps: 25, pub nv12, fps: 10.5769, comm delay [-7.0000]ms, codec delay [168.2727]ms
```
## View Effect

Here, image visualization is implemented using a web-based approach. As the data being published is H264 or H265, it needs to be decoded into NV12, then encoded into JPEG images, and finally published through web service. Please refer to hobot_rtsp_client_websocket.launch.py

tros humble:
```shell
source /opt/tros/humble/setup.bash
# Start launch
ros2 launch hobot_rtsp_client hobot_rtsp_client_websocket.launch.py hobot_rtsp_url_num:=1 hobot_rtsp_url_0:='rtsp://127.0.0.1/1080P_test.h264' hobot_transport_0:='udp'
```

Open a browser (chrome/firefox/edge) on your PC and enter <http://IP:8000> (where IP is the RDK IP address), then click on the Web display in the top left corner to see the real-time image.
     ![web_rtsp](./image/web_rtsp.png "Real-time Image")


# API Description

## Topics

### Published Topics
| Name         | Message Type                         | Description                                      |
| ------------ | ------------------------------------  | -------------------------------------------------|
| /rtsp_image_ch_[0~15] | img_msgs/msg/H26XFrame      | Video streaming topics published, received in H264 or H264 format via RTSP, supporting up to 16 channels |

## Parameters
| Parameter Name | Description                | Type   | Supported Configurations         | Required | Default Value       |
| -------------- | -------------------------- | ------ | ---------------------------------| -------- | ------------------- |
| rtsp_url_num       | rtsp link number         | int | Configure the corresponding number of routes according to business needs       | yes       | 0   |
| rtsp_url_[0~15]      | url of rtsp                 | string    | The URL of 0~rtspuurl_num takes effect     | yes      | ""                  | 
| transport_[0~15]   | transport mode of rtp    | string    | The URL of 0~rtspuurl_num takes effect      | No       | "udp"                 |

# FAQs

1. RTSP link failed?
   
   a. Check if the RDK device is connected to the IPC network. If not, check the network connection and if the IP is in the same network segment.

   b. Check if the URL of RTSP is correct.

   c. Check if the RTSP link requires a username and password, and if so, verify if they are correct.
