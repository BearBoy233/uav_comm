
echo 'gen mavlink hpp ...'
python3 -m pymavlink.tools.mavgen --lang=C --wire-protocol=2.0 --output=generated/include/mavlink/v2.0 message_definitions/v1.0/commnet.xml

echo ' '
echo 'copy hpp to /commnet/include/mavlink ...'
cp -r generated/include/mavlink/v2.0/. ../uav/src/commnet/include/mavlink

