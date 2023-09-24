
### ROS1下的集群自定义通信 [UDP]

> 消息的详细定义，以 **gen_mavlinkHPP/message_definitions/v1.0/commnet.xml**为准.
> 请务必保证 **/cluster_msgs/msg**、 **gen_mavlinkHPP/message_definitions/v1.0/commnet.xml** 以及 **关联到的cpp** 文件中ROS MSG定义的一致性(即在修改了消息的定义后，同步修改).
> 注意修改 `common.yaml` 参数.

### Func
  1. 在XML文件中自定义消息.
  2. 生成 `mavlink/***.h` 头文件.
  3. 生成 `ros_udp_***.cpp/h` 程序文件.
  
### ROS msgs定义、 Ros Topic & Mavlink 关联说明.

```
### 通用格式
uint8 sysid     # 发送端 ID 
uint8 compid    # 接收端 ID 
# ID的特殊定义   [100-地面站]    [99-所有无人机]

@@@ ***
@@@ ***
```

### 依赖缺失

首次使用，可能缺少模块

```
# mavlink
pip3 install --user future
sudo apt install python3-lxml libxml2-utils python3-tk -y

# Ubuntu 2004 & ROS1-Noedic
sudo apt install ros-noetic-mavros* ros-noetic-serial ros-noetic-geographic-msgs ros-noetic-geometry-msgs -y
# OR Ubuntu 1804 & ROS1-Melodic
sudo apt install ros-melodic-mavros* ros-melodic-serial ros-melodic-geographic-msgs ros-melodic-geometry-msgs -y

```



