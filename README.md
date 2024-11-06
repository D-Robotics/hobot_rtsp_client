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
[INFO] [launch]: All log files can be found below /root/.ros/log/2023-07-18-18-30-54-961749-ubuntu-16326
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [hobot_usb_cam-1]: process started with pid [16328]
[hobot_usb_cam-1] If you need calibration msg, please make sure the calibration file path is correct and the calibration file exists!
[hobot_usb_cam-1] [WARN] [1689676255.349211776] [hobot_usb_cam]: get camera calibration parameters failed
[hobot_usb_cam-1] [WARN] [1689676255.349432974] [hobot_usb_cam]: Start to open device /dev/video8.
[hobot_usb_cam-1] [WARN] [1689676255.596271213] [hobot_usb_cam]: Open & Init device /dev/video8 success.
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
