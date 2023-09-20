"""
This program connects with simtocontroller via localhost.
Writes control input for publishing
Subscribes to state information.

In the future, this will be connected to a controller
"""
import zmq, clproto
from state_representation import JointState

from network_interfaces.zmq import network
from utilities import receive_encoded_state

context = zmq.Context(1)
publisher = network.configure_publisher(context, '127.0.0.1:6001', False)  # state
subscriber = network.configure_subscriber(context, '127.0.0.1:6000', False)  # command

joints = ["shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"]

control_input = JointState().Zero("robot", ['ur5e_' + joint for joint in joints])
control_input.set_velocity(-2, 0)

i=0
print_state_output = False # for debug, prints state of robot occasionally

while True:
    publisher.send(clproto.encode(control_input, clproto.MessageType.JOINT_STATE_MESSAGE))
    message = receive_encoded_state(subscriber)
    if message:
        state_output = clproto.decode(message)
        if state_output and print_state_output:
            i+=1
            if i % 10000 == 0:
                print(state_output)
                i=0