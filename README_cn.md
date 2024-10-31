[English](./README.md) | 简体中文

# 功能介绍

hobot_rtsp_client 通过rtsp协议接受IPC的h264和h265码流。支持ROS标准格式方式订阅，发布img_msgs/msg/H26XFrame话题

# 使用方法


## 功能安装

在RDK系统的终端中运行如下指令，即可快速安装：

tros humbel 版本
```bash
sudo apt update
sudo apt install -y tros-humble-hobot-rtsp-client
```

## 启动码流
预先准备IPC的RTSP的数据源；

或者启动RDK系统的RTSP推流示例，参考[流媒体](https://developer.d-robotics.cc/rdk_doc/Basic_Application/multi_media/pydev_vio_demo)
```bash
cd /app/pydev_demo/08_decode_rtsp_stream/
root@ubuntu:/app/pydev_demo/08_decode_rtsp_stream# sudo ./live555MediaServer &
```

在RDK系统的终端中运行如下指令。

tros humble 版本
```bash
# 配置 tros.b humble 环境：
source /opt/tros/humble/setup.bash
# launch 方式启动
ros2 run hobot_rtsp_client hobot_rtsp_client --ros-args -p rtsp_url_num:=1 -p rtsp_url_0:='rtsp://127.0.0.1/1080P_test.h264' -p transport_0:='udp'
```

4路连接的启动方式：

tros humble 版本
```bash
# 配置 tros.b humble 环境：
source /opt/tros/humble/setup.bash
# launch 方式启动
ros2 run hobot_rtsp_client hobot_rtsp_client --ros-args -p rtsp_url_num:=4 -p rtsp_url_0:='rtsp://127.0.0.1/1080P_test.h264' -p transport_0:='udp'  -p rtsp_url_1:='rtsp://127.0.0.1/1080P_test.h264' -p transport_1:='udp'  -p rtsp_url_2:='rtsp://127.0.0.1/1080P_test.h264' -p transport_2:='udp'  -p rtsp_url_3:='rtsp://127.0.0.1/1080P_test.h264' -p transport_3:='udp'
```


如程序输出如下信息，说明节点已成功启动

```text
[INFO] [launch]: All log files can be found below /root/.ros/log/2022-06-11-15-16-13-641715-ubuntu-8852
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [mipi_cam-1]: process started with pid [8854]
...
```

## 查看效果

这里采用web端方式实现图像可视化，由于发布的是h264或者h265数据，需要解码成NV12，再编码JPEG图像，最后通过webservice发布。请参考hobot_rtsp_client_websocket.launch.py

#### tros humble 版本
```shell
source /opt/tros/humble/setup.bash
# 启动
ros2 launch hobot_rtsp_client hobot_rtsp_client_websocket.launch.py hobot_rtsp_url_num:=1 hobot_rtsp_url_0:='rtsp://127.0.0.1/1080P_test.h264' hobot_transport_0:='udp'
```

打开同一网络电脑的浏览器，访问IP地址（浏览器输入http://IP:8000，IP为地平线RDK IP地址），点击左上方`Web 展示端`即可看到RTSP输出的实时画面：
     ![web_rtsp](./image/web_rtsp.png "实时图像")


# 接口说明

## 话题

### 发布话题
| 名称         | 消息类型                             | 说明                                     |
| ------------ | ------------------------------------ | ---------------------------------------- |
| /rtsp_image_ch_[0~15] | img_msgs/msg/H26XFrame      | 发布的视频流话题，RTSP接收到的h264或者h264格式，最大支持16路, |

## 参数
| 参数名      | 解释             | 类型   | 支持的配置                 | 是否必须 | 默认值             |
| ------------| -----------------| -------| --------------------------| -------- | -------------------|
| rtsp_url_num    | rtsp链接路数       | int | 根据业务需要配置相应的路数   | 是       | 0  |
| rtsp_url_[0~15]   |      rtsp的url        | string    |    0~rtsp_url_num的url生效    | 是      | ' '                 |
| transport_[0~15]| rtp码流出书协议'tcp'/'udp' | string    |    0~rtsp_url_num的url生效        | 否       | 'udp'                |


# 常见问题

1. RTSP链接失败？

   a. 检查RDK设备与IPC的网络是否相通，如果不通检查一下网路连线，IP是否在同一个网段。

   b. 检查rtsp的url是否正确。

   c. 检查rtsp链接是否需要用户名和密码，如果需要检查是否正确。
