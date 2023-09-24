
echo 'gen mavlink hpp ...'
python3 -m pymavlink.tools.mavgen --lang=C --wire-protocol=2.0 --output=generated/include/mavlink/v2.0 message_definitions/v1.0/commnet.xml
echo ' '
echo 'copy hpp to /commnet/include/mavlink ...'
cp -r generated/include/mavlink/v2.0/. ../uav/src/commnet/include/mavlink

echo 'gen mavlink doc ...'
cd doc/
# python3 mavlink_gitbook.py

echo 'gen UDP commnet h&cpp ...'
cd ../../gen_commnet/
python3 -m auto_commnet_gen ../gen_mavlinkHPP/message_definitions/v1.0/commnet.xml
echo ' '
echo 'copy ros_udp_commnet.h to /uav/src/commnet/include '
cp generated/commnet/ros_udp_commnet.h  ../uav/src/commnet/include
echo 'copy ros_udp_Subtopic|Pubtopic.cpp to /uav/src/commnet/src '
cp generated/commnet/ros_udp_pubtopic.cpp  ../uav/src/commnet/src
cp generated/commnet/ros_udp_subtopic.cpp  ../uav/src/commnet/src

