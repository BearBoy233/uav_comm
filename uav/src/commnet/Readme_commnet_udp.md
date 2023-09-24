
### commnet 无人机集群通信 [UDP版本]

- Function 
1. 自定义Mavlink格式数据[commnet] 与 ROSmsg消息[cluster_msgs]之间转换 
2. UDP 协议通信
  

#### 信息传递示意流

- 订阅 & 打包 & 发送
`(/commnet/send/...)` -> `commnet.cpp` -> `[UDP data transfer module]`
[cluster/AA.h] -> [MAVLink Packets]

- 接收 & 解析 & 发布
`[UDP data transfer module]` -> `commnet.cpp`  -> `(/commnet/receive/...)`
[MAVLink Packets] -> [cluster/AA.h]

#### roslaunch 参数含义

`roslaunch commnet udp_gcs.launch`
`roslaunch commnet udp_uav.launch`

