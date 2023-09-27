"""
This program connects with simtocontroller via localhost.
Writes control input for publishing
Subscribes to state information.

In the future, this will be connected to a controller
"""
import clproto
from communication_interfaces.sockets import (ZMQCombinedSocketsConfiguration,
                                              ZMQContext,
                                              ZMQPublisherSubscriber)
from state_representation import JointState

client_config = ZMQCombinedSocketsConfiguration(ZMQContext(), "127.0.0.1", "5002", "5001", False, False)
client = ZMQPublisherSubscriber(client_config)
client.open()

joints = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

control_input = JointState().Zero("robot", ['ur5e_' + joint for joint in joints])
control_input.set_velocity(0, 0)

i = 0
print_state_output = False  # for debug, prints state of robot occasionally

while True:
    client.send_bytes(clproto.encode(control_input, clproto.MessageType.JOINT_STATE_MESSAGE))
    message = client.receive_bytes()
    if message:
        state_output = clproto.decode(message)
        if state_output and print_state_output:
            i+=1
            if i % 100 == 0:
                print(state_output)