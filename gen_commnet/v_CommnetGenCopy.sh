
echo 'gen UDP commnet h&cpp ...'
python3 -m auto_commnet_gen ../gen_mavlinkHPP/message_definitions/v1.0/commnet.xml

echo ' '
echo 'copy ros_udp_commnet.h to /uav/src/commnet/include '
cp generated/commnet/ros_udp_commnet.h ../uav/src/commnet/include

echo 'copy ros_udp_Subtopic|Pubtopic.cpp to /uav/src/commnet/src '
cp generated/commnet/ros_udp_pubtopic.cpp ../uav/src/commnet/src
cp generated/commnet/ros_udp_subtopic.cpp ../uav/src/commnet/src
