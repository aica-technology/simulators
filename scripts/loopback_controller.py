"""
This program connects with simtocontroller via localhost.
Writes control input for publishing
Subscribes to state information.

In the future, this will be connected to a controller
"""
import clproto
from communication_interfaces.sockets import (ZMQCombinedSocketsConfiguration,
                                              ZMQContext,
                                              ZMQPublisherSubscriber,
                                              ZMQSocketConfiguration,
                                              ZMQSubscriber)
from state_representation import JointState

context = ZMQContext()
client_config = ZMQCombinedSocketsConfiguration(context, "127.0.0.1", "5002", "5001", False, False)
wrench_sub = ZMQSubscriber(ZMQSocketConfiguration(context, "127.0.0.1", "5003", False))
client = ZMQPublisherSubscriber(client_config)
wrench_sub.open()
client.open()

joints = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

control_input = JointState().Zero("robot", ['ur5e_' + joint for joint in joints])

i = 0
j = 0
print_state_output = False  # for debug, prints state of robot occasionally
print_wrench_output = False

while True:
    client.send_bytes(clproto.encode(control_input, clproto.MessageType.JOINT_STATE_MESSAGE))
    message = client.receive_bytes()
    wrench_message = wrench_sub.receive_bytes()
    if message:
        state_output = clproto.decode(message)
        if state_output and print_state_output:
            i += 1
            if i % 100 == 0:
                print(state_output)

    if wrench_message:
        force_torque_data = clproto.decode(wrench_message)
        if force_torque_data and print_wrench_output:
            j += 1
            if j % 100 == 0:
                print(force_torque_data)
