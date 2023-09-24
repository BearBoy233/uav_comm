
### 无人机集群通用 ROS Msgs

ROS msgs定义、 Ros Topic & Mavlink 关联说明.

> 消息修改后 **/cluster_msgs/msg**、 **gen_mavlinkHPP/message_definitions/v1.0/commnet.xml** 以及 **关联到的cpp** 都需要修改.

```
### 通用格式
uint8 sysid     # 发送端 ID 
uint8 compid    # 接收端 ID 
# ID的特殊定义   [100-地面站]    [99-所有无人机]

*** ***
*** ***
```

---
消息的详细定义，以 **gen_mavlinkHPP/message_definitions/v1.0/commnet.xml**为准。
